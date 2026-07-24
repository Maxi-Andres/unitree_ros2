#!/usr/bin/env python3
"""
camera_sources.py — pluggable robot camera sources for the AI-VL camera bridge.

Each source subscribes to its robot's camera and calls `on_frame` with a ready
JPEG (bytes). Adding a robot = adding a CameraSource; the bridge stays the same.

Sources:
  - Go2VideoApiSource   : Go2 front camera via the video API (GetImageSample) — the
    robot returns a ready JPEG, forwarded as-is (no decode). Reliable + low latency.
  - G1ImageTopicSource  : the G1 (humanoid) has NO Unitree video API; its head
    camera is a depth cam (RealSense) that publishes a ROS2 image topic. This source
    AUTO-DISCOVERS that topic (prefers a CompressedImage/JPEG topic to spare DDS
    bandwidth, else a raw sensor_msgs/Image which it decodes) and forwards JPEG.
  - TestPatternSource   : a synthetic moving frame — verifies the pipeline WITHOUT
    a robot (also used to smoke-test runtime source switching).

All sources share fps/resolution/quality params (set at build, live-tunable via the
bridge's /config), and a close() so the bridge can switch source robot at runtime.
"""
import time

import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

# Target longer-vertical-side heights for the resolution presets ("native" = the
# source's own size / JPEG, forwarded without a resize).
_RES_HEIGHTS = {"native": None, "720p": 720, "480p": 480, "360p": 360}


def _encode_jpeg(bgr, quality):
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return buf.tobytes() if ok else None


def _image_msg_to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray (no cv_bridge dependency)."""
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    return buf.reshape(h, w, 3)  # best-effort fallback


def _reprocess(jpeg=None, bgr=None, resolution="native", quality=0):
    """Apply resolution/quality and return JPEG bytes. Given a JPEG at 'native' +
    quality 0, it's returned untouched (no decode — the fast path)."""
    h = _RES_HEIGHTS.get(resolution)
    if bgr is None:
        if h is None and not quality:
            return jpeg
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return jpeg
    if h is not None and bgr.shape[0] > h:
        w = max(1, int(bgr.shape[1] * h / bgr.shape[0]))
        bgr = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)
    return _encode_jpeg(bgr, quality or 80)


class _ParamSource:
    """Shared fps/resolution/quality state + an fps time-gate. fps is enforced by
    time (never by touching ROS timers off-thread), so set_params is safe to call
    from the HTTP control thread."""

    _POLL_HZ = 30.0

    def __init__(self, fps, resolution, quality):
        self._fps = max(1.0, min(self._POLL_HZ, float(fps)))
        self._res = resolution if resolution in _RES_HEIGHTS else "native"
        self._quality = int(quality or 0)
        self._last = 0.0

    def _due(self):
        now = time.monotonic()
        if now - self._last < 1.0 / self._fps:
            return False
        self._last = now
        return True

    def set_params(self, fps=None, resolution=None, quality=None):
        if fps is not None:
            try:
                self._fps = max(1.0, min(self._POLL_HZ, float(fps)))
            except (TypeError, ValueError):
                pass
        if resolution is not None and resolution in _RES_HEIGHTS:
            self._res = resolution
        if quality is not None:
            try:
                self._quality = int(quality)
            except (TypeError, ValueError):
                pass

    def get_params(self):
        return {"fps": self._fps, "resolution": self._res, "quality": self._quality}

    def close(self):  # overridden to tear down ROS entities
        pass


class Go2VideoApiSource(_ParamSource):
    """Go2 front camera via the video API (GetImageSample, api_id 1001)."""

    GET_IMAGE_SAMPLE_API_ID = 1001

    def __init__(self, node, on_frame, fps=15, resolution="native", quality=0,
                 logger=None):
        super().__init__(fps, resolution, quality)
        from unitree_api.msg import Request, Response
        self._node = node
        self._on_frame = on_frame
        self._log = logger
        self._Request = Request
        self._pub = node.create_publisher(Request, "/api/videohub/request", 10)
        self._sub = node.create_subscription(
            Response, "/api/videohub/response", self._on_response, 10)
        self._timer = node.create_timer(1.0 / self._POLL_HZ, self._tick)

    def _tick(self):
        if not self._due():
            return
        req = self._Request()
        req.header.identity.api_id = self.GET_IMAGE_SAMPLE_API_ID
        self._pub.publish(req)

    def _on_response(self, msg):
        data = bytes(msg.binary)
        if not (len(data) >= 2 and data[0] == 0xFF and data[1] == 0xD8):
            return
        jpg = _reprocess(jpeg=data, resolution=self._res, quality=self._quality)
        if jpg:
            self._on_frame(jpg)

    def close(self):
        for destroy, ent in ((self._node.destroy_timer, self._timer),
                             (self._node.destroy_subscription, self._sub),
                             (self._node.destroy_publisher, self._pub)):
            try:
                destroy(ent)
            except Exception:
                pass


class G1ImageTopicSource(_ParamSource):
    """G1 head camera (RealSense) published as a ROS2 image topic. Auto-discovers
    the topic when none is given: prefers a CompressedImage (JPEG, light on DDS),
    else a raw sensor_msgs/Image (decoded here). Skips depth/infrared topics."""

    def __init__(self, node, on_frame, topic="", fps=12, resolution="native",
                 quality=70, logger=None):
        super().__init__(fps, resolution, quality)
        self._node = node
        self._on_frame = on_frame
        self._log = logger
        self._sub = None
        self._disc_timer = None
        if topic:
            self._subscribe(topic)
        else:
            # Retry discovery until the camera node appears on the graph.
            self._disc_timer = node.create_timer(1.0, self._discover)

    def _discover(self):
        try:
            names = self._node.get_topic_names_and_types()
        except Exception:
            return
        best = None  # (rank, name, is_compressed); lower rank = better
        for name, types in names:
            low = name.lower()
            if any(k in low for k in ("depth", "infra", "aligned", "disparity")):
                continue
            is_comp = "sensor_msgs/msg/CompressedImage" in types
            is_img = "sensor_msgs/msg/Image" in types
            if not (is_comp or is_img):
                continue
            color = "color" in low or "rgb" in low
            raw = "image_raw" in low or "image" in low
            # Prefer: compressed+color(0) > compressed(1) > color raw(2) > any(3)
            rank = (0 if (is_comp and color) else 1 if is_comp
                    else 2 if color else 3)
            if best is None or rank < best[0]:
                best = (rank, name, is_comp)
        if best:
            _, name, is_comp = best
            if self._log:
                self._log.info(f"G1 camera: using {'compressed' if is_comp else 'raw'} "
                               f"image topic {name}")
            self._subscribe(name, compressed=is_comp)
            if self._disc_timer is not None:
                try:
                    self._node.destroy_timer(self._disc_timer)
                except Exception:
                    pass
                self._disc_timer = None

    def _subscribe(self, topic, compressed=None):
        # If not told, guess by suffix (…/compressed => CompressedImage).
        if compressed is None:
            compressed = topic.rstrip("/").endswith("compressed")
        if compressed:
            from sensor_msgs.msg import CompressedImage
            self._sub = self._node.create_subscription(
                CompressedImage, topic, self._cb_compressed, qos_profile_sensor_data)
        else:
            from sensor_msgs.msg import Image
            self._sub = self._node.create_subscription(
                Image, topic, self._cb_raw, qos_profile_sensor_data)

    def _cb_raw(self, msg):
        if not self._due():
            return
        try:
            jpg = _reprocess(bgr=_image_msg_to_bgr(msg),
                             resolution=self._res, quality=self._quality or 80)
            if jpg:
                self._on_frame(jpg)
        except Exception as e:
            if self._log:
                self._log.warn(f"G1 raw image convert error: {e}")

    def _cb_compressed(self, msg):
        if not self._due():
            return
        try:
            data = bytes(msg.data)
            fmt = (msg.format or "").lower()
            if "jpeg" in fmt or "jpg" in fmt or (
                    len(data) >= 2 and data[0] == 0xFF and data[1] == 0xD8):
                jpg = _reprocess(jpeg=data, resolution=self._res, quality=self._quality)
            else:  # e.g. png — decode then re-encode to JPEG
                bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                jpg = _reprocess(bgr=bgr, resolution=self._res,
                                 quality=self._quality or 80) if bgr is not None else None
            if jpg:
                self._on_frame(jpg)
        except Exception as e:
            if self._log:
                self._log.warn(f"G1 compressed image error: {e}")

    def close(self):
        for destroy, ent in ((self._node.destroy_subscription, self._sub),
                             (self._node.destroy_timer, self._disc_timer)):
            if ent is not None:
                try:
                    destroy(ent)
                except Exception:
                    pass
        self._sub = self._disc_timer = None


class TestPatternSource(_ParamSource):
    """Synthetic moving frame (no robot) — verifies the pipeline end to end."""

    def __init__(self, node, on_frame, fps=15, resolution="native", quality=70,
                 logger=None):
        super().__init__(fps, resolution, quality)
        self._node = node
        self._on_frame = on_frame
        self._x = 0
        self._timer = node.create_timer(1.0 / self._POLL_HZ, self._tick)

    def _tick(self):
        if not self._due():
            return
        img = np.full((360, 640, 3), 32, np.uint8)
        self._x = (self._x + 12) % 640
        cv2.rectangle(img, (self._x, 150), (self._x + 80, 230), (0, 180, 255), -1)
        cv2.putText(img, "ROBOT CAM TEST", (18, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        jpg = _reprocess(bgr=img, resolution=self._res, quality=self._quality or 70)
        if jpg:
            self._on_frame(jpg)

    def close(self):
        try:
            self._node.destroy_timer(self._timer)
        except Exception:
            pass


def build_source(robot, node, on_frame, cfg, logger=None):
    """Factory: pick the CameraSource for `robot` using cfg (env-derived)."""
    if robot == "go2":
        # Default quality 0 = forward the robot's own JPEG untouched (decode-free).
        return Go2VideoApiSource(
            node, on_frame,
            fps=float(cfg.get("GO2_VIDEO_FPS", 15)),
            resolution=cfg.get("GO2_RESOLUTION", "native"),
            quality=0, logger=logger)
    if robot == "g1":
        return G1ImageTopicSource(
            node, on_frame,
            topic=cfg.get("G1_IMAGE_TOPIC", ""),  # "" => auto-discover
            fps=float(cfg.get("G1_VIDEO_FPS", 12)),
            resolution=cfg.get("G1_RESOLUTION", "native"),
            quality=int(cfg.get("JPEG_QUALITY", 70) or 70), logger=logger)
    if robot == "test":
        return TestPatternSource(
            node, on_frame, fps=float(cfg.get("TEST_FPS", 15)),
            quality=int(cfg.get("JPEG_QUALITY", 70) or 70), logger=logger)
    raise ValueError(f"unknown robot camera source '{robot}' (go2|g1|test)")
