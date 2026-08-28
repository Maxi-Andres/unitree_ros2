"""Make `camera_sources` importable with no ROS2 and no OpenCV on the machine.

`camera_sources` imports `cv2` and `rclpy.qos` at module level. Neither exists outside the
Humble devcontainer, and CI installs only `ruff` and `pytest` — so without these stubs the
whole module would be untestable anywhere the bridge is not already deployed, which is
exactly the situation these tests are meant to cover (robot powered off, no container).

The stubs are deliberately empty. The code under test is the JPEG framing loop, which never
touches either import: `_reprocess(jpeg=..., resolution="native", quality=0)` returns the
bytes untouched without decoding. If a test ever needs a real `cv2` call, that test belongs
in the devcontainer, not here — and it should fail loudly on the stub rather than silently
pass.
"""
from __future__ import annotations

import sys
import types
from pathlib import Path

BRIDGE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BRIDGE))


def _stub(name: str) -> types.ModuleType:
    mod = types.ModuleType(name)
    mod.__dict__["__stub__"] = True
    return mod


if "cv2" not in sys.modules:
    sys.modules["cv2"] = _stub("cv2")

if "rclpy" not in sys.modules:
    rclpy = _stub("rclpy")
    qos = _stub("rclpy.qos")
    # The only symbol camera_sources pulls from it.
    qos.qos_profile_sensor_data = object()
    rclpy.qos = qos
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.qos"] = qos
