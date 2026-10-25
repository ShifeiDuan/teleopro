import numpy as np
import pytest

from teleopro_core.mapping import TeleopMapper
from teleopro_core.transforms import (
    make_transform, matrix_to_quat, matrix_to_rotvec, quat_to_matrix,
    rotvec_to_matrix, rpy_to_matrix)


def pose(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return make_transform(rpy_to_matrix(*rpy), xyz)


ROBOT_REF = pose((0.4, 0.1, 0.3), (np.pi, 0, 0.3))
INPUT_REF = pose((1.0, 2.0, 1.5), (0.2, -0.1, 1.0))


def test_quat_roundtrip():
    for rpy in [(0, 0, 0), (np.pi, 0, 0), (0.3, -1.2, 2.9), (0, np.pi, 0)]:
        R = rpy_to_matrix(*rpy)
        assert np.allclose(quat_to_matrix(matrix_to_quat(R)), R)


def test_rotvec_roundtrip():
    for v in [(0, 0, 0), (0.1, 0.2, -0.3), (0, 0, 3.0)]:
        assert np.allclose(matrix_to_rotvec(rotvec_to_matrix(np.array(v))), v)


def test_engage_does_not_jump():
    m = TeleopMapper()
    m.engage(INPUT_REF, ROBOT_REF)
    assert np.allclose(m.update(INPUT_REF, 0.01), ROBOT_REF)


def test_relative_translation_identity_alignment():
    m = TeleopMapper()
    m.engage(INPUT_REF, ROBOT_REF)
    moved = INPUT_REF.copy()
    moved[:3, 3] += [0.05, -0.02, 0.01]
    out = m.update(moved, 0.01)
    assert np.allclose(out[:3, 3], ROBOT_REF[:3, 3] + [0.05, -0.02, 0.01])
    assert np.allclose(out[:3, :3], ROBOT_REF[:3, :3])


def test_relative_alignment_rotates_motion():
    # Input +x maps to robot +y
    m = TeleopMapper(align=rpy_to_matrix(0, 0, np.pi / 2))
    m.engage(INPUT_REF, ROBOT_REF)
    moved = INPUT_REF.copy()
    moved[:3, 3] += [0.1, 0, 0]
    out = m.update(moved, 0.0)
    assert np.allclose(out[:3, 3] - ROBOT_REF[:3, 3], [0, 0.1, 0])


def test_relative_rotation_about_input_axis():
    m = TeleopMapper(align=rpy_to_matrix(0, 0, np.pi / 2))
    m.engage(INPUT_REF, ROBOT_REF)
    # Rotation about input x maps to robot y
    moved = make_transform(rpy_to_matrix(0.3, 0, 0) @ INPUT_REF[:3, :3], INPUT_REF[:3, 3])
    out = m.update(moved, 0.0)
    delta = matrix_to_rotvec(out[:3, :3] @ ROBOT_REF[:3, :3].T)
    assert np.allclose(delta, [0, 0.3, 0])


def test_scales():
    m = TeleopMapper(translation_scale=0.5, rotation_scale=2.0)
    m.engage(INPUT_REF, ROBOT_REF)
    moved = make_transform(rpy_to_matrix(0, 0, 0.1) @ INPUT_REF[:3, :3],
                           INPUT_REF[:3, 3] + [0.2, 0, 0])
    out = m.update(moved, 0.0)
    assert np.allclose(out[:3, 3] - ROBOT_REF[:3, 3], [0.1, 0, 0])
    assert np.allclose(matrix_to_rotvec(out[:3, :3] @ ROBOT_REF[:3, :3].T), [0, 0, 0.2])


def test_dof_mask_planar():
    m = TeleopMapper(dof_mask=[1, 1, 0, 0, 0, 1])
    m.engage(INPUT_REF, ROBOT_REF)
    moved = make_transform(rpy_to_matrix(0.2, 0, 0.1) @ INPUT_REF[:3, :3],
                           INPUT_REF[:3, 3] + [0.1, 0.1, 0.1])
    out = m.update(moved, 0.0)
    assert np.allclose(out[:3, 3] - ROBOT_REF[:3, 3], [0.1, 0.1, 0])
    w = matrix_to_rotvec(out[:3, :3] @ ROBOT_REF[:3, :3].T)
    assert abs(w[0]) < 1e-9 and abs(w[1]) < 1e-9 and abs(w[2]) > 0.05


def test_velocity_limit():
    m = TeleopMapper(max_lin_vel=1.0, max_ang_vel=1.0)
    m.engage(INPUT_REF, ROBOT_REF)
    moved = make_transform(rpy_to_matrix(0, 0, 1.0) @ INPUT_REF[:3, :3],
                           INPUT_REF[:3, 3] + [1.0, 0, 0])
    out = m.update(moved, 0.01)
    assert np.isclose(np.linalg.norm(out[:3, 3] - ROBOT_REF[:3, 3]), 0.01)
    assert np.isclose(np.linalg.norm(matrix_to_rotvec(out[:3, :3] @ ROBOT_REF[:3, :3].T)), 0.01)


def test_workspace_clamp():
    m = TeleopMapper(workspace_min=[0, -0.2, 0.2], workspace_max=[0.5, 0.2, 0.6])
    m.engage(INPUT_REF, ROBOT_REF)
    moved = INPUT_REF.copy()
    moved[:3, 3] += [1.0, -1.0, 1.0]
    out = m.update(moved, 0.0)
    assert np.allclose(out[:3, 3], [0.5, -0.2, 0.6])


def test_invalid_args():
    with pytest.raises(ValueError):
        TeleopMapper(dof_mask=[1, 1, 1])
    with pytest.raises(RuntimeError):
        TeleopMapper().update(INPUT_REF, 0.01)
