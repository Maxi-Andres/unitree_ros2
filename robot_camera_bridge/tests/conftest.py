"""Make `camera_sources` importable with no ROS2, no OpenCV and no NumPy on the machine.

`camera_sources` imports `cv2`, `numpy` and `rclpy.qos` at module level. None of them exists
outside the Humble devcontainer, and CI installs only `ruff` and `pytest` — so without these
stubs the whole module would be untestable anywhere the bridge is not already deployed, which
is exactly the situation these tests are meant to cover (robot powered off, no container).

STUB EVERY THIRD-PARTY IMPORT, UNCONDITIONALLY. Do NOT probe whether a module happens to be
installed and stub only the missing ones. The first version of this file did exactly that and
it broke CI: `numpy` is installed on the dev box but not in the CI job, so the suite passed
locally and failed on push with `ModuleNotFoundError: No module named 'numpy'`. A test that
depends on what is lying around in the local environment is not a test. If you add an import
to `camera_sources`, add its stub here in the same commit.

The stubs are deliberately empty. The code under test is the JPEG framing loop, which never
touches any of them: `_reprocess(jpeg=..., resolution="native", quality=0)` returns the bytes
untouched without decoding. If a test ever reaches a real `cv2` or `numpy` call it will raise
`AttributeError` on the stub — loudly, which is what we want. A test that genuinely needs them
belongs in the devcontainer, not here.

Note for whoever tests `robot_camera_bridge.py` next: it needs `rclpy` (not just `rclpy.qos`)
and `websocket` (websocket-client) on top of these.
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


# `not in sys.modules` guards against clobbering a module something else already imported
# on purpose; it is NOT a check for whether the package is installed. See the docstring.
for _name in ("cv2", "numpy"):
    if _name not in sys.modules:
        sys.modules[_name] = _stub(_name)

if "rclpy" not in sys.modules:
    _rclpy = _stub("rclpy")
    _qos = _stub("rclpy.qos")
    # The only symbol camera_sources pulls from it.
    _qos.qos_profile_sensor_data = object()
    _rclpy.qos = _qos
    sys.modules["rclpy"] = _rclpy
    sys.modules["rclpy.qos"] = _qos
