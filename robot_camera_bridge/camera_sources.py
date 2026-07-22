#!/usr/bin/env python3
"""
camera_sources.py — pluggable robot camera sources for the AI-VL camera bridge.

Each source subscribes to its robot's camera (a ROS2 topic) and calls `on_frame`
with a ready-to-send JPEG (bytes) per decoded frame. Adding a robot = adding a
CameraSource; the bridge (robot_camera_bridge.py) stays the same.

Sources:
  - Go2VideoApiSource   : Go2 front camera via the video API (GetImageSample) — the
    robot returns a ready JPEG, forwarded as-is (no decode). Reliable + low latency.
  - G1ImageTopicSource  : a generic sensor_msgs/Image topic (configurable name), for
    the G1's camera (or any ROS2 camera node) once it publishes one.
  - TestPatternSource   : a synthetic moving frame — verifies the whole pipeline
    (bridge -> backend -> monitor) WITHOUT a robot.

Frames are JPEG-encoded here (cv2) so the bridge just forwards bytes.
"""
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data


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
    # Best-effort fallback: assume 3 channels.
    return buf.reshape(h, w, 3)


class Go2VideoApiSource:
    """Go2 front camera via the video API (GetImageSample, api_id 1001).

    We poll `/api/videohub/request` at the target FPS; the robot answers on
    `/api/videohub/response` with a ready **JPEG** in the `binary` field, which we
    forward AS-IS (no decode, no re-encode) — lowest latency and rock-solid.

    (We deliberately do NOT use /frontvideostream: over the ROS2/cyclonedds bridge
    its large H.264 sequences deserialize corrupt on the Go2 — the video API is the
    reliable path.)
    """

    GET_IMAGE_SAMPLE_API_ID = 1001

    def __init__(self, node, on_frame, fps=15, logger=None):
        from unitree_api.msg import Request, Response
        self._on_frame = on_frame
        self._log = logger
        self._Request = Request
        self._pub = node.create_publisher(Request, "/api/videohub/request", 10)
        node.create_subscription(
            Response, "/api/videohub/response", self._on_response, 10)
        # Poll for a fresh image at the target rate (one request -> one JPEG).
        node.create_timer(1.0 / max(1.0, fps), self._request_image)

    def _request_image(self):
        req = self._Request()
        req.header.identity.api_id = self.GET_IMAGE_SAMPLE_API_ID
        self._pub.publish(req)

    def _on_response(self, msg):
        data = bytes(msg.binary)
        # Forward only valid JPEGs (magic FF D8); ignore empty/other responses.
        if len(data) >= 2 and data[0] == 0xFF and data[1] == 0xD8:
            self._on_frame(data)


class G1ImageTopicSource:
    """G1 (or any) camera published as sensor_msgs/Image on a configurable topic."""

    def __init__(self, node, on_frame, topic, jpeg_quality=70, logger=None):
        from sensor_msgs.msg import Image
        self._on_frame = on_frame
        self._quality = jpeg_quality
        self._log = logger
        node.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        try:
            jpg = _encode_jpeg(_image_msg_to_bgr(msg), self._quality)
            if jpg:
                self._on_frame(jpg)
        except Exception as e:
            if self._log:
                self._log.warn(f"G1 image convert error: {e}")


class TestPatternSource:
    """Synthetic moving frame (no robot) — verifies the pipeline end to end."""

    def __init__(self, node, on_frame, fps=15, jpeg_quality=70, logger=None):
        self._on_frame = on_frame
        self._quality = jpeg_quality
        self._x = 0
        node.create_timer(1.0 / max(1.0, fps), self._tick)

    def _tick(self):
        img = np.full((360, 640, 3), 32, np.uint8)
        self._x = (self._x + 12) % 640
        cv2.rectangle(img, (self._x, 150), (self._x + 80, 230), (0, 180, 255), -1)
        cv2.putText(img, "ROBOT CAM TEST", (18, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        jpg = _encode_jpeg(img, self._quality)
        if jpg:
            self._on_frame(jpg)


def build_source(robot, node, on_frame, cfg, logger=None):
    """Factory: pick the CameraSource for `robot` using cfg (env-derived)."""
    if robot == "go2":
        return Go2VideoApiSource(
            node, on_frame,
            fps=float(cfg.get("GO2_VIDEO_FPS", 15)), logger=logger)
    if robot == "g1":
        return G1ImageTopicSource(
            node, on_frame,
            topic=cfg.get("G1_IMAGE_TOPIC", "/camera/color/image_raw"),
            jpeg_quality=int(cfg.get("JPEG_QUALITY", 70)), logger=logger)
    if robot == "test":
        return TestPatternSource(
            node, on_frame, fps=float(cfg.get("TEST_FPS", 15)),
            jpeg_quality=int(cfg.get("JPEG_QUALITY", 70)), logger=logger)
    raise ValueError(f"unknown robot camera source '{robot}' (go2|g1|test)")
