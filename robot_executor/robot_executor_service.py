#!/usr/bin/env python3
"""
robot_executor_service.py — AI-VL Phase 2 skill executor (Unitree, ROS2 transport).

Closes the voice loop: it receives the skill JSON that AI-VL's /command produced
(`{robot, skill, params}`) over HTTP and MAKES THE ROBOT DO IT by publishing the
matching Unitree command over ROS2. Runs inside the unitree_ros2 devcontainer (that
is where ROS2 + the robot's DDS connection live); the AI-VL backend forwards to it.

    frontend button -> backend /api/execute -> THIS /execute -> ROS2 -> 🤖 robot

Design:
- **Transport-abstracted** (the recorded Phase-2 decision): a thin `RobotTransport`
  interface with a `Go2Ros2Transport` implementation (rclpy). A G1 / SDK transport can
  be added later without touching the HTTP layer.
- **Safety:** `SAFE_MODE` (default on) blocks every skill that can make the robot lose
  its support (damping, zero torque, Go2 acrobatics) or swap its control mode; see each
  command module's `DANGEROUS_SKILLS`. `DRY_RUN` builds and logs the command WITHOUT
  publishing — use it to test the plumbing without moving the robot.
- **Answers are read, not assumed:** each request carries a unique
  `header.identity.id` and the G1 transport waits for the matching `/api/*/response`,
  so a command the robot REJECTS is reported as a failure with its reason instead of a
  silent "ok" (this is what the official Unitree clients do — see base_client.hpp).

Run (inside the devcontainer, after sourcing the ROS2 env so DDS + unitree_api are up):
    source /workspace/setup.sh
    python3 /workspace/robot_executor/robot_executor_service.py
Config comes from robot_executor/.env (see .env.example) or the environment.

Endpoints:
    GET  /health              -> {ok, robot, safe_mode, dry_run}
    GET  /state?robot=g1      -> {ok, robot, state:{fsm_id, fsm_mode, action_list, ...}}
                                 read-only; the discovery path for the app modes and
                                 arm actions Unitree does not document.
    GET  /dds                 -> {ok, iface, peers, discovery}
    POST /dds {peers, iface?} -> persist ../dds.env and RESTART to apply it, so the
                                 stack can follow the robot to another network (e.g.
                                 its wlan0 on another VLAN) from the AI-VL page.
    POST /execute {robot, skill, params} -> {ok, robot, skill, detail, ...}
"""
import json
import os
import queue
import re
import signal
import sys
import threading
import time
from abc import ABC, abstractmethod
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import go2_commands
import g1_commands


# --------------------------------------------------------------------------- #
# Config (.env or environment)
# --------------------------------------------------------------------------- #
def _load_dotenv(path):
    """Minimal .env loader (no dependency). KEY=VALUE per line; # comments ignored."""
    if not os.path.exists(path):
        return
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def _as_bool(value, default):
    if value is None:
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on", "si", "sí")


_HERE = os.path.dirname(os.path.abspath(__file__))
_load_dotenv(os.path.join(_HERE, ".env"))

EXECUTOR_HOST = os.environ.get("EXECUTOR_HOST", "0.0.0.0")
EXECUTOR_PORT = int(os.environ.get("EXECUTOR_PORT", "8090"))
DEFAULT_ROBOT = os.environ.get("DEFAULT_ROBOT", "go2")
ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.123.161")  # informational (health/ping)
# Default OFF. This is only the FALLBACK: each /execute request may carry its own
# `safe_mode` (the page toggle), which overrides this per request.
SAFE_MODE = _as_bool(os.environ.get("SAFE_MODE"), False)
DRY_RUN = _as_bool(os.environ.get("DRY_RUN"), False)
MOVE_RATE_HZ = float(os.environ.get("MOVE_RATE_HZ", "10"))
DEFAULT_STEP_S = float(os.environ.get("DEFAULT_STEP_S", "2.0"))
# Hard ceiling on a single bounded move, so a bad duration can't run the robot away.
MAX_STEP_S = float(os.environ.get("MAX_STEP_S", "10.0"))
# How long to wait for the robot's answer on /api/*/response for a one-shot command.
# The official clients wait 5 s; keep it snappier for a UI pad, and never treat a
# missing answer as a failure (see _call).
RESPONSE_TIMEOUT_S = float(os.environ.get("RESPONSE_TIMEOUT_S", "2.0"))
# How long to wait for DDS to match the robot's api server as a reader of a command
# topic before declaring that nobody is listening. Covers normal discovery latency on a
# freshly built node without hanging a UI click.
DISCOVERY_WAIT_S = float(os.environ.get("DISCOVERY_WAIT_S", "1.5"))


# --------------------------------------------------------------------------- #
# rclpy lifecycle — bring the global context up on demand and KEEP it recoverable.
# Transports create/destroy NODES freely (self-heal). The context must never be
# re-init'd while it is already up (rclpy forbids that), but if a fault took it
# down (e.g. a DDS/network drop), the NEXT rebuild has to be able to bring it back
# — so we gate on the ACTUAL context state, not a write-once boolean.
# --------------------------------------------------------------------------- #
_RCLPY_LOCK = threading.Lock()
_rclpy_initialized = False
_node_seq = 0
# None = rclpy's DEFAULT context. Humble refuses to re-init the default context once it
# has been shut down ("Context.init() must only be called once"), which used to turn a
# recoverable fault into a permanent one — the self-heal path below then died with that
# RuntimeError on every command. When that happens we switch to a context of our own,
# which CAN be replaced, and everything (nodes, executors, ok() checks) uses it.
_context = None


def _rclpy_ok():
    import rclpy
    return rclpy.ok(context=_context)


# --------------------------------------------------------------------------- #
# DDS transport config (shared with the camera bridge through ../dds.env, which
# setup.sh sources). Exposed over HTTP so the AI-VL page can point the stack at a
# robot on another network without anyone opening a container shell.
#
# The knob that matters is the unicast PEER list: DDS discovery is multicast, and
# multicast is link-local, so a robot on another VLAN stays invisible no matter how
# well the two subnets route — until it is named here by IP.
# --------------------------------------------------------------------------- #
DDS_ENV_PATH = os.path.abspath(os.path.join(_HERE, os.pardir, "dds.env"))


def _read_dds_env() -> dict:
    values = {"CYCLONEDDS_IFACE": "", "ROBOT_DDS_PEERS": ""}
    try:
        with open(DDS_ENV_PATH, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, value = line.partition("=")
                key = key.strip()
                if key in values:
                    values[key] = value.strip().strip('"').strip("'")
    except FileNotFoundError:
        pass
    # Fall back to whatever setup.sh exported into this process.
    if not values["CYCLONEDDS_IFACE"]:
        values["CYCLONEDDS_IFACE"] = os.environ.get("CYCLONEDDS_IFACE", "enp4s0")
    if not values["ROBOT_DDS_PEERS"]:
        values["ROBOT_DDS_PEERS"] = os.environ.get("ROBOT_DDS_PEERS", "")
    return values


def _valid_peer(text: str) -> bool:
    """Accept only a bare IPv4/IPv6 address. This string is interpolated into the
    CycloneDDS XML config, so anything looser would be an injection hole."""
    import ipaddress
    try:
        ipaddress.ip_address(text)
        return True
    except ValueError:
        return False


def _valid_iface(text: str) -> bool:
    return bool(re.fullmatch(r"[A-Za-z0-9_.:-]{1,32}", text))


def _write_dds_env(iface: str, peers: list) -> None:
    body = (
        "# Written by the AI-VL robot executor (POST /dds). See dds.env.example.\n"
        f"CYCLONEDDS_IFACE={iface}\n"
        f"ROBOT_DDS_PEERS={','.join(peers)}\n"
    )
    tmp = DDS_ENV_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(body)
    os.replace(tmp, DDS_ENV_PATH)  # atomic: never leave a half-written config


def _restart_self():
    """Re-exec through run_executor.sh so setup.sh rebuilds CYCLONEDDS_URI from the
    file we just wrote. CycloneDDS reads that config once per process, so a restart is
    genuinely required — there is no way to re-point DDS in place."""
    wrapper = os.path.join(_HERE, "run_executor.sh")
    for transport in list(_TRANSPORTS.values()):
        try:
            transport.shutdown()  # publishes a stop first: never leave the robot moving
        except Exception:
            pass
    print("[executor] restarting to apply the new DDS transport", flush=True)
    sys.stdout.flush()
    os.execv("/bin/bash", ["bash", wrapper])


def _ensure_rclpy_initialized():
    """(Re)initialize the rclpy context whenever it isn't currently up.

    Gating on `rclpy.ok()` (the real context state) rather than a boolean that is only
    ever set to True means a transport rebuilt after a fault can revive rclpy, instead
    of being stuck forever on 'rclpy.init() has not been called'.

    The catch: re-initializing the DEFAULT context after a shutdown is NOT allowed in
    Humble — it raises "Context.init() must only be called once". A shutdown happens on
    a plain SIGTERM (rclpy installs a signal handler), so any run that gets stopped and
    then keeps working hit that RuntimeError on every later command. When the default
    context refuses, we build our own and use it from then on."""
    import rclpy
    global _rclpy_initialized, _context
    with _RCLPY_LOCK:
        if not rclpy.ok(context=_context):
            try:
                rclpy.init(context=_context)
            except RuntimeError:
                from rclpy.context import Context
                _context = Context()
                rclpy.init(context=_context)
                print("[executor] default rclpy context was spent; switched to a fresh "
                      "one", flush=True)
        _rclpy_initialized = True


def _next_node_name(tag="robot"):
    global _node_seq
    with _RCLPY_LOCK:
        _node_seq += 1
        return f"aivl_robot_executor_{tag}_{_node_seq}"


def _create_node(tag):
    """A node on whichever context is currently live (see `_context`)."""
    import rclpy
    return rclpy.create_node(_next_node_name(tag), context=_context)


def _next_request_id():
    """Unique id for one unitree_api Request.

    Every official Unitree client stamps `header.identity.id` with the system uptime
    in nanoseconds (unitree_ros2 example/src/include/common/base_client.hpp) and then
    matches the response by that id. We left it at 0 on every request, which both
    makes each command look like a repeat of request 0 to the robot's api server and
    makes responses impossible to attribute. Monotonic nanoseconds give the same
    strictly-increasing, never-repeating property."""
    return time.monotonic_ns()


# --------------------------------------------------------------------------- #
# Transport interface
# --------------------------------------------------------------------------- #
class RobotTransport(ABC):
    """Actuation channel to one robot. Swap ROS2 <-> SDK behind this interface."""

    @abstractmethod
    def execute(self, skill: str, params: dict) -> dict:
        """Perform `skill`. Returns {ok, detail, ...}. Never raises for a normal
        unsupported skill — returns {ok: False, ...} instead."""

    def shutdown(self) -> None:  # optional
        pass


class Go2Ros2Transport(RobotTransport):
    """Drives the Go2 by publishing unitree_api/msg/Request to /api/sport/request."""

    def __init__(self, dry_run: bool):
        self._dry_run = dry_run
        self._move_lock = threading.Lock()
        self._move_stop = threading.Event()
        self._move_thread = None
        # Guards the node/publisher handles so the HTTP request thread and the move
        # loop thread can't rebuild/publish on them concurrently.
        self._node_lock = threading.Lock()
        self._node = None
        self._pub = None
        self._Request = None
        if not dry_run:
            self._init_ros()

    def _init_ros(self):
        from unitree_api.msg import Request
        self._Request = Request
        with self._node_lock:
            self._build_node_locked()

    def _build_node_locked(self):
        """(Re)create the ROS2 node + publisher on a live rclpy context. Caller holds
        `_node_lock`. Safe to call any number of times: it brings rclpy back up if a
        fault took it down and destroys a stale node first, so the executor recovers
        on its own instead of getting stuck on 'rclpy.init() has not been called'."""
        import rclpy
        _ensure_rclpy_initialized()   # re-inits the context if a fault brought it down
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
            self._pub = None
        # Fresh node name each build so a rebuild never clashes with a not-yet-freed
        # old node of the same name.
        self._node = _create_node("robot")
        self._pub = self._node.create_publisher(self._Request, "/api/sport/request", 10)

    def _publish(self, api_id: int, parameter: dict | None):
        """Build + publish one sport Request (or just log it in dry-run).

        Self-healing: if the node/context was torn down by a DDS/network blip (the
        classic '40s' drop), the first publish throws — we rebuild the node once and
        retry, so a transient fault costs a single message instead of wedging the
        executor into an endless 502 loop."""
        param_str = json.dumps(parameter) if parameter is not None else ""
        if self._dry_run:
            print(f"[DRY_RUN] would publish /api/sport/request "
                  f"api_id={api_id} parameter={param_str!r}", flush=True)
            return
        req = self._Request()
        req.header.identity.api_id = api_id
        req.header.identity.id = _next_request_id()
        req.parameter = param_str
        import rclpy
        with self._node_lock:
            if self._node is None or not _rclpy_ok():
                self._build_node_locked()
            try:
                self._pub.publish(req)
            except Exception as e:
                print(f"[executor] publish failed ({e}); rebuilding node and retrying",
                      flush=True)
                self._build_node_locked()
                self._pub.publish(req)

    def _stop_move_loop(self):
        """Signal any running move loop to end and join it."""
        self._move_stop.set()
        thread = self._move_thread
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
        self._move_thread = None

    def _run_move_loop(self, vx, vy, vyaw, deadline):
        """Re-publish Move at MOVE_RATE_HZ until stopped or the deadline passes. A
        held velocity needs re-sending; a bounded move also acts as a DEADMAN: if it
        reaches its deadline (no fresh command arrived) it halts the robot on its own.

        StopMove is published ONLY when the deadline is actually reached — NOT when
        the loop is superseded by a newer move. That lets a teleop pad refresh a
        short bounded move every tick for smooth continuous motion (each refresh
        cancels the previous loop without injecting a stop), while a frozen/crashed
        client still stops the robot within one `deadline`. An explicit 'stop' skill
        halts it immediately (it calls StopMove itself)."""
        period = 1.0 / MOVE_RATE_HZ
        reached_deadline = False
        try:
            while not self._move_stop.is_set():
                if deadline is not None and time.monotonic() >= deadline:
                    reached_deadline = True
                    break
                self._publish(go2_commands.MOVE_API_ID,
                              {"x": vx, "y": vy, "z": vyaw})
                time.sleep(period)
        finally:
            if reached_deadline:
                self._publish(go2_commands.STOPMOVE_API_ID, None)

    def _start_move(self, vx, vy, vyaw, duration, continuous):
        with self._move_lock:
            self._stop_move_loop()
            self._move_stop = threading.Event()
            deadline = None
            if not continuous:
                step = duration if duration else DEFAULT_STEP_S
                step = max(0.1, min(step, MAX_STEP_S))
                deadline = time.monotonic() + step
            self._move_thread = threading.Thread(
                target=self._run_move_loop, args=(vx, vy, vyaw, deadline), daemon=True)
            self._move_thread.start()

    def execute(self, skill: str, params: dict) -> dict:
        intent = go2_commands.resolve(skill, params or {})
        kind = intent["kind"]

        if kind == "unsupported":
            return {"ok": False, "detail": intent["reason"]}

        if kind == "stop":
            with self._move_lock:
                self._stop_move_loop()
            self._publish(go2_commands.STOPMOVE_API_ID, None)
            return {"ok": True, "detail": "StopMove", "api_id": go2_commands.STOPMOVE_API_ID}

        if kind == "move":
            self._start_move(intent["vx"], intent["vy"], intent["vyaw"],
                             intent["duration"], intent["continuous"])
            mode = "continuous (until 'stop')" if intent["continuous"] else \
                f"{intent['duration'] or DEFAULT_STEP_S:.1f}s step"
            return {"ok": True, "detail": f"Move vx={intent['vx']} vy={intent['vy']} "
                    f"vyaw={intent['vyaw']} ({mode})", "api_id": go2_commands.MOVE_API_ID}

        # single
        self._publish(intent["api_id"], intent["parameter"])
        return {"ok": True, "detail": f"sport api_id={intent['api_id']}",
                "api_id": intent["api_id"], "parameter": intent["parameter"]}

    def shutdown(self):
        """Tear down defensively — each step independently — so a broken context can
        always be reset (used both on exit and to evict a failed transport for rebuild)."""
        try:
            with self._move_lock:
                self._stop_move_loop()
        except Exception:
            pass
        if self._dry_run:
            return
        # Best-effort stop, then destroy THIS node under the node lock. Do NOT touch
        # the global rclpy context here (process-wide) — that's shut down once at exit.
        try:
            self._publish(go2_commands.STOPMOVE_API_ID, None)
        except Exception:
            pass
        with self._node_lock:
            try:
                if self._node is not None:
                    self._node.destroy_node()
            except Exception:
                pass
            finally:
                self._node = None
                self._pub = None


class G1Ros2Transport(RobotTransport):
    """Drives the Unitree G1 (humanoid) over ROS2. Unlike the Go2 (one sport topic),
    the G1 spans THREE request topics — locomotion + FSM + loco gestures on
    /api/sport/request, preset arm actions on /api/arm/request, and TTS/audio on
    /api/voice/request — all as unitree_api/msg/Request. Mirrors the Go2 transport's
    node lifecycle, self-heal, and bounded-move deadman loop."""

    _PUB_DUR = 1.0  # per-tick SetVelocity duration (s); re-published each move tick

    def __init__(self, dry_run: bool):
        self._dry_run = dry_run
        self._move_lock = threading.Lock()
        self._move_stop = threading.Event()
        self._move_thread = None
        self._node_lock = threading.Lock()
        self._node = None
        self._pubs = {}          # name -> publisher (sport/arm/voice/motion)
        self._Request = None
        self._Response = None
        self._tts_index = 0
        # Request id -> queue waiting for that response. Guarded by its own lock so a
        # response callback (spin thread) never touches the node lock.
        self._pending = {}
        self._pending_lock = threading.Lock()
        self._spin_exec = None
        self._spin_thread = None
        if not dry_run:
            self._init_ros()

    def _init_ros(self):
        from unitree_api.msg import Request, Response
        self._Request = Request
        self._Response = Response
        with self._node_lock:
            self._build_node_locked()

    def _build_node_locked(self):
        """(Re)create the node, one publisher per G1 request topic and one subscription
        per response topic, on a live rclpy context. Self-healing, same contract as the
        Go2 transport. A spin thread is what lets the response callbacks actually run —
        publishers alone need no spinning, subscriptions do."""
        import rclpy
        _ensure_rclpy_initialized()
        self._stop_spin_locked()
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
            self._pubs = {}
        self._node = _create_node("g1")
        self._pubs = {
            name: self._node.create_publisher(self._Request, topic, 10)
            for name, topic in g1_commands.TOPICS.items()
        }
        for topic in g1_commands.RESPONSE_TOPICS.values():
            self._node.create_subscription(self._Response, topic, self._on_response, 10)
        self._start_spin_locked()

    # --- Response plumbing -------------------------------------------------- #
    def _start_spin_locked(self):
        from rclpy.executors import SingleThreadedExecutor
        self._spin_exec = SingleThreadedExecutor(context=_context)
        self._spin_exec.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._spin_loop, args=(self._spin_exec,), daemon=True)
        self._spin_thread.start()

    def _spin_loop(self, executor):
        try:
            executor.spin()
        except Exception:
            pass  # executor shut down or the context went away — nothing to report

    def _stop_spin_locked(self):
        executor, thread = self._spin_exec, self._spin_thread
        self._spin_exec = self._spin_thread = None
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def _on_response(self, msg):
        """Hand a response to whoever is waiting for that request id (spin thread)."""
        try:
            rid = msg.header.identity.id
            code = msg.header.status.code
            data = msg.data
        except AttributeError:
            return
        with self._pending_lock:
            waiter = self._pending.get(rid)
        if waiter is not None:
            try:
                waiter.put_nowait((code, data))
            except queue.Full:
                pass

    def _next_tts_index(self):
        self._tts_index += 1
        return self._tts_index

    def _publish_req(self, pub_name: str, api_id: int, parameter: dict | None,
                     request_id: int):
        """Publish one Request on the named topic (self-healing + one retry)."""
        param_str = json.dumps(parameter) if parameter is not None else ""
        if self._dry_run:
            print(f"[DRY_RUN] would publish {g1_commands.TOPICS.get(pub_name, pub_name)} "
                  f"api_id={api_id} parameter={param_str!r}", flush=True)
            return
        req = self._Request()
        req.header.identity.api_id = api_id
        req.header.identity.id = request_id
        req.parameter = param_str
        import rclpy
        with self._node_lock:
            if self._node is None or not _rclpy_ok():
                self._build_node_locked()
            try:
                self._pubs[pub_name].publish(req)
            except Exception as e:
                print(f"[executor] G1 publish failed ({e}); rebuilding node and retrying",
                      flush=True)
                self._build_node_locked()
                self._pubs[pub_name].publish(req)

    def _publish(self, pub_name: str, api_id: int, parameter: dict | None):
        """Fire-and-forget publish — used by the high-rate move loop, which must never
        block on an answer (so it also skips the matched-subscriber check)."""
        self._publish_req(pub_name, api_id, parameter, _next_request_id())

    def _wait_for_subscriber(self, pub_name: str) -> bool:
        """True once the robot's api server is a matched reader on this topic.

        A fresh DDS publisher needs a moment to match its remote reader; publishing
        before that silently drops the message. And if nothing EVER matches, the robot's
        high-level service simply is not attached to this DDS network — worth saying out
        loud rather than reporting a mystery timeout."""
        deadline = time.monotonic() + DISCOVERY_WAIT_S
        while True:
            with self._node_lock:
                publisher = self._pubs.get(pub_name)
            try:
                if publisher is not None and publisher.get_subscription_count() > 0:
                    return True
            except Exception:
                return True  # can't tell (old rclpy) -> don't block the command
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)

    def _call(self, pub_name: str, api_id: int, parameter: dict | None,
              wait: float = RESPONSE_TIMEOUT_S) -> dict:
        """Publish one command and wait for the robot's answer, like every official
        Unitree client does.

        Returns {"acked": bool, "code": int, "data": str}. `acked` False means the robot
        never answered on /api/*/response — treated as "sent", NOT as a failure, so this
        can never invent an error that the robot did not report. A non-zero `code` is a
        real rejection (e.g. 7404: arm actions need fsm 500/501/801) — which is what
        turns "the gesture did nothing" into a message an operator can act on."""
        if self._dry_run:
            self._publish_req(pub_name, api_id, parameter, 0)
            return {"acked": False, "code": 0, "data": ""}
        if not self._wait_for_subscriber(pub_name):
            # Provably nowhere to send: DDS reports no matched reader on this topic, so
            # the message would be dropped on the floor. Worth its own answer — this is
            # otherwise indistinguishable from "the robot ignored me", and it took a
            # manual `ros2 topic info` to tell the two apart.
            return {"acked": False, "code": 0, "data": "", "no_subscriber": True}
        rid = _next_request_id()
        answer = queue.Queue(maxsize=1)
        with self._pending_lock:
            self._pending[rid] = answer
        try:
            self._publish_req(pub_name, api_id, parameter, rid)
            try:
                code, data = answer.get(timeout=wait)
                return {"acked": True, "code": int(code), "data": data or ""}
            except queue.Empty:
                return {"acked": False, "code": 0, "data": ""}
        finally:
            with self._pending_lock:
                self._pending.pop(rid, None)

    def query_state(self, names=None) -> dict:
        """Ask the robot what state it is in, and for its OWN preset action list.

        This is the discovery path for everything Unitree does not document: put the
        robot in the phone app's Climb or Lie up mode, read `fsm_id` here, and that
        number becomes a named skill. `action_list` likewise reveals arm actions the
        SDK's 16-entry table omits (the app shows a Welcome and a Dance that are not in
        it). `names` limits the queries — g1_fsm_watch.py polls just the fsm ones."""
        queries = g1_commands.STATE_QUERIES
        if names:
            queries = {n: queries[n] for n in names if n in queries}
        out = {}
        for name, (pub, api_id) in queries.items():
            res = self._call(pub, api_id, None)
            if res.get("no_subscriber"):
                out[name] = {"error": "nobody is subscribed to this topic — the robot's "
                                      "high-level service is not on the DDS network"}
            elif not res.get("acked"):
                out[name] = {"error": "no answer from the robot"}
            elif res.get("code"):
                out[name] = {"error": g1_commands.error_hint(res["code"]),
                             "code": res["code"]}
            else:
                raw = res.get("data") or ""
                try:
                    out[name] = json.loads(raw)
                except (ValueError, TypeError):
                    out[name] = raw
        return out

    def _set_velocity(self, vx, vy, vyaw, duration):
        self._publish(g1_commands.SPORT, g1_commands.SET_VELOCITY_API_ID,
                      {"velocity": [vx, vy, vyaw], "duration": duration})

    def _stop_move_loop(self):
        self._move_stop.set()
        thread = self._move_thread
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
        self._move_thread = None

    def _run_move_loop(self, vx, vy, vyaw, deadline):
        """Re-publish SetVelocity at MOVE_RATE_HZ until stopped or the deadline
        passes. Each publish carries its own short duration, so a frozen client stops
        the robot on its own; StopMove is sent only when the deadline is reached (not
        when a fresh command supersedes this loop) — smooth continuous walking."""
        period = 1.0 / MOVE_RATE_HZ
        reached_deadline = False
        try:
            while not self._move_stop.is_set():
                if deadline is not None and time.monotonic() >= deadline:
                    reached_deadline = True
                    break
                self._set_velocity(vx, vy, vyaw, self._PUB_DUR)
                time.sleep(period)
        finally:
            if reached_deadline:
                self._set_velocity(0.0, 0.0, 0.0, 1.0)

    def _start_move(self, vx, vy, vyaw, duration, continuous):
        with self._move_lock:
            self._stop_move_loop()
            self._move_stop = threading.Event()
            deadline = None
            if not continuous:
                step = duration if duration else DEFAULT_STEP_S
                step = max(0.1, min(step, MAX_STEP_S))
                deadline = time.monotonic() + step
            self._move_thread = threading.Thread(
                target=self._run_move_loop, args=(vx, vy, vyaw, deadline), daemon=True)
            self._move_thread.start()

    def execute(self, skill: str, params: dict) -> dict:
        intent = g1_commands.resolve(skill, params or {})
        kind = intent["kind"]

        if kind == "unsupported":
            return {"ok": False, "detail": intent["reason"]}

        if kind == "stop":
            with self._move_lock:
                self._stop_move_loop()
            self._set_velocity(0.0, 0.0, 0.0, 1.0)
            return {"ok": True, "detail": "StopMove (SetVelocity 0)",
                    "api_id": g1_commands.SET_VELOCITY_API_ID}

        if kind == "move":
            self._start_move(intent["vx"], intent["vy"], intent["vyaw"],
                             intent["duration"], intent["continuous"])
            mode = "continuous (until 'stop')" if intent["continuous"] else \
                f"{intent['duration'] or DEFAULT_STEP_S:.1f}s step"
            return {"ok": True, "api_id": g1_commands.SET_VELOCITY_API_ID,
                    "detail": f"Move v=[{intent['vx']},{intent['vy']},{intent['vyaw']}] ({mode})"}

        if kind == "say":
            idx = self._next_tts_index()
            res = self._call(g1_commands.VOICE, g1_commands.AUDIO_TTS_API_ID,
                             {"index": idx, "text": intent["text"], "speaker_id": 0})
            return self._result(f"TTS: {intent['text'][:60]}", res,
                                g1_commands.AUDIO_TTS_API_ID, None, skill)

        # single (FSM posture, stand height, loco gesture, arm action, mode, volume)
        res = self._call(intent["pub"], intent["api_id"], intent["parameter"])
        return self._result(f"{intent['pub']} api_id={intent['api_id']}", res,
                            intent["api_id"], intent["parameter"], skill)

    def _result(self, detail: str, res: dict, api_id: int, parameter, skill: str) -> dict:
        """Turn a _call() outcome into the /execute reply.

        A rejection becomes ok=False with the robot's own reason — previously every
        command reported success and a gesture that the robot refused looked, from the
        UI, exactly like one it performed."""
        code = res.get("code") or 0
        out = {"api_id": api_id, "parameter": parameter}
        if res.get("no_subscriber"):
            reason = (
                "nobody is subscribed to this command topic — the robot's high-level "
                "control service is not attached to the DDS network. Check that the "
                "robot is powered and out of low-level/debug mode, and that the DDS "
                "interface in setup.sh matches the one facing the robot."
            )
            return {**out, "ok": False, "detail": f"{detail} — not sent: {reason}",
                    "error": reason}
        if code:
            hint = g1_commands.error_hint(code)
            if skill in g1_commands.NEEDS_OPERATION_STATE:
                hint += (" This gesture only runs in an operation state "
                         f"(fsm id {g1_commands.GESTURE_FSM_IDS}).")
            return {**out, "ok": False, "code": code,
                    "detail": f"{detail} — rejected: {hint}", "error": hint}
        if not res.get("acked"):
            # No answer on /api/*/response: sent, but unconfirmed. Say so instead of
            # claiming the robot did it.
            return {**out, "ok": True, "acked": False,
                    "detail": f"{detail} (sent, no ack from the robot)"}
        return {**out, "ok": True, "acked": True, "detail": detail}

    def shutdown(self, stop_first: bool = True):
        """Tear down. `stop_first=False` skips the zero-velocity publish, so a read-only
        user of this transport (g1_fsm_watch.py) never writes a single command."""
        try:
            with self._move_lock:
                self._stop_move_loop()
        except Exception:
            pass
        if self._dry_run:
            return
        if stop_first:
            try:
                self._set_velocity(0.0, 0.0, 0.0, 1.0)
            except Exception:
                pass
        with self._node_lock:
            self._stop_spin_locked()
            try:
                if self._node is not None:
                    self._node.destroy_node()
            except Exception:
                pass
            finally:
                self._node = None
                self._pubs = {}


class UnsupportedRobotTransport(RobotTransport):
    """Placeholder for a robot with no transport yet (e.g. G1 over ROS2/SDK)."""

    def __init__(self, robot: str):
        self._robot = robot

    def execute(self, skill: str, params: dict) -> dict:
        return {"ok": False, "detail": f"no transport implemented for robot "
                f"'{self._robot}' yet (only 'go2' is wired)"}


# --------------------------------------------------------------------------- #
# Transport registry — one live transport per robot (built lazily)
# --------------------------------------------------------------------------- #
_TRANSPORTS: dict[str, RobotTransport] = {}
_TRANSPORTS_LOCK = threading.Lock()

# Per-robot command module (skill -> intent mapping). Also the source of each
# robot's SAFE_MODE-blocked skill set.
_CMD_MODULES = {"go2": go2_commands, "g1": g1_commands}


def _dangerous_skills(robot: str) -> set:
    mod = _CMD_MODULES.get(robot)
    return getattr(mod, "DANGEROUS_SKILLS", set()) if mod else set()


def _get_transport(robot: str) -> RobotTransport:
    with _TRANSPORTS_LOCK:
        if robot not in _TRANSPORTS:
            if robot == "go2":
                _TRANSPORTS[robot] = Go2Ros2Transport(dry_run=DRY_RUN)
            elif robot == "g1":
                _TRANSPORTS[robot] = G1Ros2Transport(dry_run=DRY_RUN)
            else:
                _TRANSPORTS[robot] = UnsupportedRobotTransport(robot)
        return _TRANSPORTS[robot]


# --------------------------------------------------------------------------- #
# HTTP server
# --------------------------------------------------------------------------- #
class ExecutorHandler(BaseHTTPRequestHandler):
    def _send(self, code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):  # quieter default logging
        pass

    def _handle_dds(self, body):
        """POST /dds {peers: ["1.2.3.4", ...], iface?: "enp4s0"} — persist the DDS
        transport and restart so it takes effect. Answers BEFORE re-exec'ing, otherwise
        the caller would see a dropped connection instead of a result."""
        cfg = _read_dds_env()
        iface = str(body.get("iface") or cfg["CYCLONEDDS_IFACE"]).strip()
        if not _valid_iface(iface):
            self._send(400, {"ok": False, "error": f"invalid interface name '{iface}'"})
            return

        raw = body.get("peers", None)
        if raw is None:
            self._send(400, {"ok": False, "error": "'peers' is required (use [] to "
                                                   "go back to multicast discovery)"})
            return
        if isinstance(raw, str):
            raw = [p for p in raw.replace(",", " ").split() if p]
        if not isinstance(raw, list):
            self._send(400, {"ok": False, "error": "'peers' must be a list of IPs"})
            return
        peers = [str(p).strip() for p in raw if str(p).strip()]
        bad = [p for p in peers if not _valid_peer(p)]
        if bad:
            self._send(400, {"ok": False,
                             "error": f"not valid IP addresses: {', '.join(bad)}"})
            return

        try:
            _write_dds_env(iface, peers)
        except OSError as e:
            self._send(500, {"ok": False, "error": f"could not write {DDS_ENV_PATH}: {e}"})
            return

        self._send(200, {"ok": True, "iface": iface, "peers": peers,
                         "discovery": "unicast" if peers else "multicast",
                         "restarting": True,
                         "detail": "DDS transport saved; the executor is restarting to "
                                   "apply it (a few seconds)."})
        try:
            self.wfile.flush()
        except Exception:
            pass
        # Restart on another thread so this handler can finish and close the socket.
        threading.Thread(target=_restart_self, daemon=True).start()

    def do_GET(self):
        path = self.path.rstrip("/")
        if path == "/dds":
            cfg = _read_dds_env()
            peers = [p for p in cfg["ROBOT_DDS_PEERS"].split(",") if p.strip()]
            self._send(200, {
                "ok": True,
                "iface": cfg["CYCLONEDDS_IFACE"],
                "peers": peers,
                # No peers + a robot on another subnet = it can never be discovered.
                "discovery": "unicast" if peers else "multicast",
                "file": DDS_ENV_PATH,
            })
            return
        if path.startswith("/state"):
            # GET /state?robot=g1 -> what state the robot is in + its own action list.
            # Read-only: it publishes only Get* api_ids, so it can never move anything.
            robot = "g1"
            if "?" in self.path:
                from urllib.parse import parse_qs, urlparse
                robot = (parse_qs(urlparse(self.path).query).get("robot", ["g1"])[0]
                         or "g1").strip()
            transport = _get_transport(robot)
            if not hasattr(transport, "query_state"):
                self._send(400, {"ok": False,
                                 "error": f"no state query for robot '{robot}'"})
                return
            try:
                self._send(200, {"ok": True, "robot": robot,
                                 "state": transport.query_state()})
            except Exception as e:
                self._send(500, {"ok": False, "error": f"state query failed: {e}"})
            return
        if path == "/health":
            # rclpy is brought up lazily on the first /execute, so `rclpy_ok` is
            # False until then — informational, never a reason to fail /health.
            try:
                import rclpy
                rclpy_ok = bool(_rclpy_ok())
            except Exception:
                rclpy_ok = False
            self._send(200, {"ok": True, "service": "robot_executor",
                             "default_robot": DEFAULT_ROBOT, "safe_mode": SAFE_MODE,
                             "dry_run": DRY_RUN, "robot_ip": ROBOT_IP,
                             "rclpy_ok": rclpy_ok})
        else:
            self._send(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        path = self.path.rstrip("/")
        if path not in ("/execute", "/dds"):
            self._send(404, {"ok": False, "error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, json.JSONDecodeError):
            self._send(400, {"ok": False, "error": "invalid JSON body"})
            return

        if path == "/dds":
            self._handle_dds(body)
            return

        robot = (body.get("robot") or DEFAULT_ROBOT).strip()
        skill = (body.get("skill") or "").strip()
        params = body.get("params") or {}

        if not skill or skill == "unknown":
            self._send(400, {"ok": False, "error": "no executable skill"})
            return

        # Per-request safe_mode (page toggle) overrides the env default when present.
        req_safe = body.get("safe_mode")
        effective_safe = req_safe if isinstance(req_safe, bool) else SAFE_MODE
        if effective_safe and skill in _dangerous_skills(robot):
            self._send(403, {"ok": False, "blocked": True, "robot": robot,
                             "skill": skill,
                             "error": f"'{skill}' blocked by safe mode: it can make the "
                             "robot lose its support or change control mode. Turn safe "
                             "mode off to allow it."})
            return

        try:
            result = _get_transport(robot).execute(skill, params)
        except Exception as e:  # never let a transport error kill the server
            # Self-heal: drop the broken transport (e.g. a dead ROS2 context after a
            # network blip) so the NEXT command rebuilds it fresh instead of staying
            # stuck on 502 until a manual restart.
            with _TRANSPORTS_LOCK:
                broken = _TRANSPORTS.pop(robot, None)
            if broken is not None:
                broken.shutdown()
            self._send(502, {"ok": False, "robot": robot, "skill": skill,
                             "error": f"transport error: {e} "
                             "(dropped; will rebuild on next command)"})
            return

        code = 200 if result.get("ok") else 422
        self._send(code, {"robot": robot, "skill": skill, "dry_run": DRY_RUN, **result})


def main():
    print(f"robot_executor: robot={DEFAULT_ROBOT} safe_mode={SAFE_MODE} "
          f"dry_run={DRY_RUN} listening on {EXECUTOR_HOST}:{EXECUTOR_PORT}", flush=True)
    server = ThreadingHTTPServer((EXECUTOR_HOST, EXECUTOR_PORT), ExecutorHandler)

    # Stop cleanly on SIGTERM as well as Ctrl-C. rclpy installs its own signal handlers
    # when the context comes up, which swallowed SIGTERM: `docker stop`, systemd and
    # `timeout` all appeared to be ignored and had to escalate to SIGKILL after their
    # grace period — losing the robot stop below, and showing up as exit code 137.
    def _terminate(_sig, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _terminate)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for transport in _TRANSPORTS.values():
            transport.shutdown()
        server.server_close()
        # Shut down the global rclpy context exactly once, on exit.
        if _rclpy_initialized:
            import rclpy
            if rclpy.ok(context=_context):
                rclpy.shutdown(context=_context)


if __name__ == "__main__":
    main()
