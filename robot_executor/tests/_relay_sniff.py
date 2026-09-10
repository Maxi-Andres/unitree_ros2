#!/usr/bin/env python3
"""Passive sniffer for the executor -> relay command stream. TEMPORARY DIAGNOSTIC.

WHY THIS EXISTS: `RelayTransport._run_move_loop` posts {"verb":"stop_move"} unconditionally
when the loop exits, not only when the deadline was reached (Go2 and G1 guard it with
`if reached_deadline`). Every new move therefore supersedes the previous one by way of a
halt: move-halt-move-halt instead of smooth motion. Two strict xfails assert the correct
behaviour; this script is how you SEE it happen against the real robot.

It is read-only: an AF_PACKET raw socket, no traffic generated, nothing touched on the
robot. It must run where the executor's packets are, which is the devcontainer (host
networking, privileged), because tcpdump is not installed there and the host needs sudo.

    docker exec <devcontainer> python3 /workspace/robot_executor/tests/_relay_sniff.py

Output is one line per command, with the gap since the previous one, so an injected halt
between two moves is visible as `stop_move` sandwiched at a few milliseconds.
"""
import re
import socket
import struct
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8092
VERB = re.compile(rb'"verb"\s*:\s*"([a-z_]+)"')
VEC = re.compile(rb'"(vx|vy|vyaw)"\s*:\s*(-?[0-9.]+)')

ETH_P_ALL = 3
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(ETH_P_ALL))

print(f"# sniffing tcp/{PORT} — one line per command, Ctrl-C to stop", flush=True)
print(f"# {'hora':12s} {'+gap':>8s}  verbo", flush=True)

last = None
counts: dict = {}
try:
    while True:
        frame = s.recv(65535)
        if len(frame) < 34 or struct.unpack("!H", frame[12:14])[0] != 0x0800:
            continue
        ip = frame[14:]
        ihl = (ip[0] & 0x0F) * 4
        if ip[9] != 6:                                    # not TCP
            continue
        tcp = ip[ihl:]
        sport, dport = struct.unpack("!HH", tcp[:4])
        if PORT not in (sport, dport):
            continue
        payload = tcp[((tcp[12] >> 4) * 4):]
        m = VERB.search(payload)
        if not m:
            continue
        verb = m.group(1).decode()
        now = time.time()
        gap = "" if last is None else f"{(now - last) * 1000:7.1f}ms"
        vec = " ".join(f"{k.decode()}={v.decode()}" for k, v in VEC.findall(payload))
        counts[verb] = counts.get(verb, 0) + 1
        print(f"{time.strftime('%H:%M:%S')}.{int(now % 1 * 1000):03d} {gap:>8s}  "
              f"{verb:10s} {vec}", flush=True)
        last = now
except KeyboardInterrupt:
    pass
finally:
    print("\n# totales: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())),
          flush=True)
