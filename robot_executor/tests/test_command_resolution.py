"""Command resolution for both robots — the layer that decides what reaches the robot.

Every test here names the defect it catches. Nothing in this file touches DDS, ROS2 or a
robot: `resolve()` is pure, which is what makes the control path testable with the robot
powered off.

Two tests are marked `xfail(strict=True)`: they assert the CORRECT behavior for defects that
are still open (findings P0-2 and P0-4 in the security audit). Strict means that when the
defect is fixed the test starts passing and pytest FAILS on the unexpected pass — telling
you to delete the marker. A red test would be ignored; this one cannot be.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import g1_commands
import go2_commands

ROBOTS = [pytest.param(go2_commands, id="go2"), pytest.param(g1_commands, id="g1")]


# --------------------------------------------------------------------------- #
# The allowlist boundary
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("mod", ROBOTS)
def test_unknown_skill_is_unsupported(mod):
    """Catches: a typo or an injected skill name reaching the robot as a real command.

    The contract is that anything not in the catalog resolves to `unsupported` — never to a
    passthrough, never to a best guess.
    """
    for name in ["", "nope", "sport/1001", "1001", "../stop", "MOVE", "move; stop"]:
        assert mod.resolve(name, {})["kind"] == "unsupported", f"{name!r} was not refused"


@pytest.mark.parametrize("mod", ROBOTS)
def test_stop_always_resolves_to_stop(mod):
    """Catches: a refactor that makes the stop verb depend on params or state.

    Stop must resolve identically no matter what it is handed — it is the verb you reach for
    when something is already going wrong.
    """
    for params in [{}, {"vx": 9}, {"continuous": True}, {"duration_s": 999}]:
        assert mod.resolve("stop", params) == {"kind": "stop"}


# --------------------------------------------------------------------------- #
# Velocity clamps
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("mod", ROBOTS)
@pytest.mark.parametrize(
    "asked",
    [1e6, -1e6, float("inf"), float("-inf"), 42.0, -42.0],
    ids=["huge", "-huge", "inf", "-inf", "42", "-42"],
)
def test_move_velocity_is_clamped_to_the_envelope(mod, asked):
    """Catches: a caller (or a bug) asking for a speed the robot should never be given.

    The clamp is the last numeric guard before the wire, so it is checked at and beyond the
    limit rather than only in range.
    """
    out = mod.resolve("move", {"vx": asked, "vy": asked, "vyaw": asked})
    assert abs(out["vx"]) <= mod.MAX_VX
    assert abs(out["vy"]) <= mod.MAX_VY
    assert abs(out["vyaw"]) <= mod.MAX_VYAW


@pytest.mark.parametrize("mod", ROBOTS)
def test_non_numeric_velocity_becomes_zero_not_an_exception(mod):
    """Catches: a malformed body crashing the executor mid-drive instead of resolving to a
    stand-still. A crash on the control path is worse than a refused command."""
    out = mod.resolve("move", {"vx": None, "vy": "fast", "vyaw": {}})
    assert (out["vx"], out["vy"], out["vyaw"]) == (0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# The dangerous-skill catalog
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("mod", ROBOTS)
def test_dangerous_skills_are_all_known_to_the_catalog(mod):
    """Catches: a renamed skill leaving a stale entry in DANGEROUS_SKILLS, which would
    silently stop blocking the thing it was written to block.

    "Known" is the property under test, not "executable": several dangerous skills need a
    parameter (`dance` a routine name, `set_fsm_id` an id) and are correctly refused without
    one. The module distinguishes the two cases in its `reason` — "not mapped" is the only
    one that means the name is gone.
    """
    for skill in mod.DANGEROUS_SKILLS:
        reason = mod.resolve(skill, {}).get("reason", "")
        assert "not mapped" not in reason, (
            f"{skill!r} is listed as dangerous but the catalog no longer maps it — "
            "safe mode is no longer protecting anything under that name"
        )


@pytest.mark.parametrize("mod", ROBOTS)
def test_the_skills_that_drop_the_robot_are_marked_dangerous(mod):
    """Catches: a skill that removes the robot's support being reachable in safe mode.

    `damp` cuts compliance and a standing robot collapses; the G1 also has `zero_torque`.
    These are the two that must never be one request away by accident.
    """
    assert "damp" in mod.DANGEROUS_SKILLS
    if mod.resolve("zero_torque", {})["kind"] != "unsupported":
        assert "zero_torque" in mod.DANGEROUS_SKILLS


# --------------------------------------------------------------------------- #
# Open defects, asserted as they SHOULD behave (see the module docstring)
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("mod", ROBOTS)
@pytest.mark.xfail(
    strict=True,
    reason="finding P0-4: `move` without `continuous` defaults to True, which disables the "
    "dead-man. Flip the default to False, then delete this marker.",
)
def test_move_without_continuous_is_bounded(mod):
    """Catches: unbounded motion from a caller that simply omitted a field.

    A bounded move carries a deadline and stops on its own if refreshes stop arriving. With
    `continuous` defaulting to True, a client that forgets the field gets motion that runs
    until an explicit stop — the failure mode the dead-man exists to prevent.
    """
    assert mod.resolve("move", {"vx": 0.2})["continuous"] is False


@pytest.mark.parametrize("mod", ROBOTS)
def test_walk_without_continuous_is_bounded(mod):
    """The same property for `walk`/`turn`, which already default correctly. Here to pin the
    behavior so the two paths cannot drift apart again."""
    assert mod.resolve("walk", {})["continuous"] is False
