"""Teleoperation session with engage, input loss and re-anchoring."""

import logging

import numpy as np

from teleopro_core.mapping import TeleopMapper, TWIST_FRAMES
from teleopro_core.transforms import make_transform, matrix_to_quat, quat_to_matrix, rpy_to_matrix


def as_matrix(pose):
    """Convert a pose to a 4x4 matrix.

    Args:
        pose: 4x4 matrix or ((x, y, z), (qx, qy, qz, qw)).
    """
    if isinstance(pose, np.ndarray) and pose.shape == (4, 4):
        return pose.astype(float)
    xyz, quat = pose
    return make_transform(quat_to_matrix(quat), xyz)


def as_pos_quat(T):
    """Convert a 4x4 matrix to ((x, y, z), (qx, qy, qz, qw))."""
    return tuple(float(v) for v in T[:3, 3]), tuple(float(v) for v in matrix_to_quat(T[:3, :3]))


class Teleop:
    """Teleoperation session.

    Args:
        align_rpy: Rotation of the input axes in the robot base (roll, pitch, yaw).
        translation_scale: Translation scale factor.
        rotation_scale: Rotation scale factor.
        max_lin_vel: Linear velocity limit in m/s, 0 for none.
        max_ang_vel: Angular velocity limit in rad/s, 0 for none.
        dof_mask: [x, y, z, roll, pitch, yaw], 0 locks the axis.
        workspace_min: Lower position bound [x, y, z], or None.
        workspace_max: Upper position bound [x, y, z], or None.
        input_timeout: Input older than this many seconds is treated as lost.
        twist_frame: 'base' or 'tool', the frame twist input is expressed in.
        logger: Logger with debug(), info() and warning(). Defaults to logging.
    """

    def __init__(self, align_rpy=(0.0, 0.0, 0.0), translation_scale=1.0,
                 rotation_scale=1.0, max_lin_vel=0.5, max_ang_vel=1.0,
                 dof_mask=(1, 1, 1, 1, 1, 1), workspace_min=None, workspace_max=None,
                 input_timeout=0.2, twist_frame='base', logger=None):
        if twist_frame not in TWIST_FRAMES:
            raise ValueError(f'twist_frame must be one of {TWIST_FRAMES}, got {twist_frame!r}')
        self.mapper = TeleopMapper(
            align=rpy_to_matrix(*align_rpy),
            translation_scale=translation_scale,
            rotation_scale=rotation_scale,
            dof_mask=dof_mask,
            max_lin_vel=max_lin_vel,
            max_ang_vel=max_ang_vel,
            workspace_min=workspace_min,
            workspace_max=workspace_max,
        )
        self.input_timeout = float(input_timeout)
        self.twist_frame = twist_frame
        self.log = logger if logger is not None else logging.getLogger('teleopro_core')

        self._want_engaged = False
        self._input_stale = False
        self._needs_reanchor = False
        self._waiting_logged = False
        self._last_lost_log = None
        self._hold_pose = None
        self._last_now = None

    @property
    def engaged(self):
        return self.mapper.engaged

    @property
    def needs_robot_pose(self):
        """Whether the next update() uses robot_pose."""
        if self.mapper.engaged:
            return False
        return self._want_engaged or self._hold_pose is None

    def engage(self):
        """Start following the input."""
        self._want_engaged = True

    def disengage(self):
        """Stop following the input and hold the target."""
        self._want_engaged = False
        if self.mapper.engaged:
            self._hold_pose = self.mapper.last_target
            self.mapper.disengage()
            self.log.info('Teleop disengaged')

    def update(self, now, input_pose=None, robot_pose=None, input_stamp=None):
        """Advance one step with pose input.

        Args:
            now: Current time in seconds.
            input_pose: Input pose in the input base, or None if not tracked.
            robot_pose: Current robot pose in the robot base.
            input_stamp: Time of input_pose in seconds, or None.

        Returns:
            4x4 target pose in the robot base, or None if no robot pose yet.
        """
        dt = self._step_time(now)
        T_in = None if input_pose is None else as_matrix(input_pose)
        T_robot = None if robot_pose is None else as_matrix(robot_pose)

        stale = T_in is None or self._is_stale(now, input_stamp)
        if stale != self._input_stale:
            self._input_stale = stale
            if stale and self.mapper.engaged:
                self._needs_reanchor = True
                # Throttled, tracking can flicker
                if self._last_lost_log is None or now - self._last_lost_log >= 5.0:
                    self.log.warning('Input lost, holding target')
                    self._last_lost_log = now

        if self._want_engaged and not stale:
            if not self.mapper.engaged:
                self._try_engage(T_in, T_robot)
            elif self._needs_reanchor:
                # Reset references after input loss
                self.mapper.engage(T_in, self.mapper.last_target)
                self._needs_reanchor = False
                self.log.debug('Input back, re-anchored')
            else:
                return self.mapper.update(T_in, dt)

        if self.mapper.engaged:
            return self.mapper.last_target
        return self._hold(T_robot)

    def update_twist(self, now, twist=None, robot_pose=None, twist_stamp=None):
        """Advance one step with twist input.

        Args:
            now: Current time in seconds.
            twist: ((vx, vy, vz), (wx, wy, wz)) in m/s and rad/s, or None.
            robot_pose: Current robot pose in the robot base.
            twist_stamp: Time of twist in seconds, or None.

        Returns:
            4x4 target pose in the robot base, or None if no robot pose yet.
        """
        dt = self._step_time(now)
        T_robot = None if robot_pose is None else as_matrix(robot_pose)

        if self._want_engaged and not self.mapper.engaged:
            self._try_engage(None, T_robot)

        if self.mapper.engaged:
            # Stop when the twist is missing or stale
            if twist is None or self._is_stale(now, twist_stamp):
                twist = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            linear, angular = twist
            return self.mapper.integrate(linear, angular, dt, self.twist_frame)
        return self._hold(T_robot)

    def _step_time(self, now):
        dt = 0.0 if self._last_now is None else now - self._last_now
        self._last_now = now
        return dt

    def _is_stale(self, now, stamp):
        return (stamp is not None and self.input_timeout > 0.0
                and now - stamp > self.input_timeout)

    def _try_engage(self, T_in, T_robot):
        if T_robot is None:
            if not self._waiting_logged:
                self.log.warning('Waiting for robot pose to engage')
                self._waiting_logged = True
            return
        self.mapper.engage(T_in, T_robot)
        self._waiting_logged = False
        self.log.info('Teleop engaged')

    def _hold(self, T_robot):
        # Hold the first robot pose until engaged
        if self._hold_pose is None and T_robot is not None:
            self._hold_pose = T_robot.copy()
        return None if self._hold_pose is None else self._hold_pose.copy()
