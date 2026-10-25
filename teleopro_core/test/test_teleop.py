import numpy as np

from teleopro_core import as_matrix, as_pos_quat, Teleop
from teleopro_core.transforms import make_transform, matrix_to_rotvec, rpy_to_matrix

ROBOT = make_transform(rpy_to_matrix(np.pi, 0, 0), [0.4, 0.0, 0.4])
DEVICE = make_transform(np.eye(3), [1.0, 2.0, 1.0])


def moved(dx):
    T = DEVICE.copy()
    T[0, 3] += dx
    return T


def make(**kw):
    kw.setdefault('max_lin_vel', 0.0)
    kw.setdefault('max_ang_vel', 0.0)
    return Teleop(**kw)


def test_pose_formats_roundtrip():
    xyz, quat = as_pos_quat(ROBOT)
    assert np.allclose(as_matrix((xyz, quat)), ROBOT)


def test_holds_robot_pose_until_engaged():
    t = make()
    assert t.update(0.0, DEVICE, None) is None
    assert np.allclose(t.update(0.01, DEVICE, ROBOT), ROBOT)
    # Target stays latched
    other = ROBOT.copy()
    other[2, 3] -= 0.05
    assert np.allclose(t.update(0.02, moved(0.1), other), ROBOT)
    assert not t.engaged


def test_engage_follows_device():
    t = make()
    t.engage()
    assert np.allclose(t.update(0.0, DEVICE, ROBOT), ROBOT)
    assert t.engaged
    out = t.update(0.01, moved(0.05))
    assert np.allclose(out[:3, 3], ROBOT[:3, 3] + [0.05, 0, 0])


def test_engage_waits_for_robot_pose():
    t = make()
    t.engage()
    assert t.needs_robot_pose
    assert t.update(0.0, DEVICE, None) is None
    assert not t.engaged
    t.update(0.01, DEVICE, ROBOT)
    assert t.engaged and not t.needs_robot_pose


def test_disengage_holds_last_target():
    t = make()
    t.engage()
    t.update(0.0, DEVICE, ROBOT)
    last = t.update(0.01, moved(0.05))
    t.disengage()
    assert np.allclose(t.update(0.02, moved(0.3), ROBOT), last)


def test_reengage_starts_from_robot_pose():
    t = make()
    t.engage()
    t.update(0.0, DEVICE, ROBOT)
    t.update(0.01, moved(0.05))
    t.disengage()
    t.engage()
    assert np.allclose(t.update(0.02, moved(0.2), ROBOT), ROBOT)


def test_input_lost_holds_then_reanchors():
    t = make()
    t.engage()
    t.update(0.0, DEVICE, ROBOT)
    last = t.update(0.01, moved(0.05))
    assert np.allclose(t.update(0.02, None), last)              # lost
    assert np.allclose(t.update(0.03, moved(0.5)), last)        # re-anchored
    out = t.update(0.04, moved(0.52))                            # following
    assert np.allclose(out[:3, 3], last[:3, 3] + [0.02, 0, 0])


def test_stale_stamp_counts_as_lost():
    t = make(input_timeout=0.2)
    t.engage()
    t.update(0.0, DEVICE, ROBOT, input_stamp=0.0)
    last = t.update(0.1, moved(0.05), input_stamp=0.1)
    # Stale stamp
    assert np.allclose(t.update(0.5, moved(0.3), input_stamp=0.1), last)


def test_accepts_xyz_quat_input():
    t = make()
    t.engage()
    t.update(0.0, as_pos_quat(DEVICE), as_pos_quat(ROBOT))
    out = t.update(0.01, ((1.05, 2.0, 1.0), (0, 0, 0, 1)))
    assert np.allclose(out[:3, 3], ROBOT[:3, 3] + [0.05, 0, 0])


# Twist input

ZERO = (0.0, 0.0, 0.0)


def test_twist_holds_until_engaged():
    t = make()
    assert np.allclose(t.update_twist(0.0, ((0.1, 0, 0), ZERO), ROBOT), ROBOT)
    assert np.allclose(t.update_twist(1.0, ((0.1, 0, 0), ZERO), ROBOT), ROBOT)


def test_twist_base_frame():
    t = make()
    t.engage()
    t.update_twist(0.0, (ZERO, ZERO), ROBOT)
    out = t.update_twist(0.5, ((0.1, 0, 0.2), ZERO))
    assert np.allclose(out[:3, 3], ROBOT[:3, 3] + [0.05, 0, 0.1])
    assert np.allclose(out[:3, :3], ROBOT[:3, :3])


def test_twist_tool_frame():
    # ROBOT tool is flipped about x: tool +z points along base -z
    t = make(twist_frame='tool')
    t.engage()
    t.update_twist(0.0, (ZERO, ZERO), ROBOT)
    out = t.update_twist(0.5, ((0, 0, 0.1), ZERO))
    assert np.allclose(out[:3, 3], ROBOT[:3, 3] + [0, 0, -0.05])


def test_twist_rotation():
    t = make()
    t.engage()
    t.update_twist(0.0, (ZERO, ZERO), ROBOT)
    out = t.update_twist(1.0, (ZERO, (0, 0, 0.2)))
    w = matrix_to_rotvec(out[:3, :3] @ ROBOT[:3, :3].T)
    assert np.allclose(w, [0, 0, 0.2])


def test_twist_stale_stops():
    t = make(input_timeout=0.2)
    t.engage()
    t.update_twist(0.0, (ZERO, ZERO), ROBOT, twist_stamp=0.0)
    last = t.update_twist(0.1, ((0.1, 0, 0), ZERO), twist_stamp=0.1)
    assert np.allclose(t.update_twist(1.0, ((0.1, 0, 0), ZERO), twist_stamp=0.1), last)
    assert np.allclose(t.update_twist(1.1, None), last)


def test_twist_dof_mask_and_scale():
    t = make(dof_mask=[1, 0, 1, 0, 0, 1], translation_scale=2.0)
    t.engage()
    t.update_twist(0.0, (ZERO, ZERO), ROBOT)
    out = t.update_twist(0.5, ((0.1, 0.1, 0), (0.3, 0, 0)))
    assert np.allclose(out[:3, 3], ROBOT[:3, 3] + [0.1, 0, 0])
    assert np.allclose(out[:3, :3], ROBOT[:3, :3])


def test_twist_disengage_holds():
    t = make()
    t.engage()
    t.update_twist(0.0, (ZERO, ZERO), ROBOT)
    last = t.update_twist(0.5, ((0.1, 0, 0), ZERO))
    t.disengage()
    assert np.allclose(t.update_twist(1.0, ((0.1, 0, 0), ZERO), ROBOT), last)


def test_invalid_twist_frame():
    import pytest
    with pytest.raises(ValueError):
        Teleop(twist_frame='world')
