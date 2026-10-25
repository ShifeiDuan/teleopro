"""Mapping from input pose or velocity to a robot target pose."""

import numpy as np

from teleopro_core.transforms import make_transform, matrix_to_rotvec, rotvec_to_matrix

TWIST_FRAMES = ('base', 'tool')


class TeleopMapper:
    """Maps input motion onto a robot target pose.

    Pose input applies the input motion since engage to the reference pose:

        p = p_ref + s_t * R_align * (p_in - p_in0)
        R = R_align * dR * R_align^T * R_ref,  dR = R_in * R_in0^T

    Twist input integrates the velocity, expressed in the robot base or tool frame.

    Args:
        align: 3x3 rotation of the input axes in the robot base.
        translation_scale: Translation scale factor.
        rotation_scale: Rotation scale factor.
        dof_mask: [x, y, z, roll, pitch, yaw], 0 locks the axis.
        max_lin_vel: Linear velocity limit in m/s, 0 for none.
        max_ang_vel: Angular velocity limit in rad/s, 0 for none.
        workspace_min: Lower position bound [x, y, z], or None.
        workspace_max: Upper position bound [x, y, z], or None.
    """

    def __init__(self, align=None, translation_scale=1.0, rotation_scale=1.0,
                 dof_mask=(1, 1, 1, 1, 1, 1), max_lin_vel=0.0, max_ang_vel=0.0,
                 workspace_min=None, workspace_max=None):
        if len(dof_mask) != 6:
            raise ValueError('dof_mask must have 6 entries [x, y, z, roll, pitch, yaw]')
        self.align = np.eye(3) if align is None else np.asarray(align, dtype=float)
        self.translation_scale = float(translation_scale)
        self.rotation_scale = float(rotation_scale)
        self.dof_mask = np.asarray(dof_mask, dtype=float)
        self.max_lin_vel = float(max_lin_vel)
        self.max_ang_vel = float(max_ang_vel)
        self.workspace_min = None if workspace_min is None else np.asarray(workspace_min, float)
        self.workspace_max = None if workspace_max is None else np.asarray(workspace_max, float)

        self._input_ref = None
        self._target_ref = None
        self._last_target = None

    @property
    def engaged(self):
        return self._target_ref is not None

    @property
    def last_target(self):
        return None if self._last_target is None else self._last_target.copy()

    def engage(self, T_input, T_target_ref):
        """Set the reference poses.

        Args:
            T_input: 4x4 input pose in the input base, or None for twist input.
            T_target_ref: 4x4 starting target pose in the robot base.
        """
        self._input_ref = None if T_input is None else np.array(T_input, dtype=float)
        self._target_ref = np.array(T_target_ref, dtype=float)
        self._last_target = self._target_ref.copy()

    def disengage(self):
        self._input_ref = None
        self._target_ref = None

    def update(self, T_input, dt):
        """Compute the target pose from an input pose.

        Args:
            T_input: 4x4 input pose in the input base.
            dt: Time since the last update in seconds.

        Returns:
            4x4 target pose in the robot base.
        """
        if self._input_ref is None:
            raise RuntimeError('update() called without an input reference')
        T_in = np.asarray(T_input, dtype=float)
        T0 = self._input_ref
        R_a = self.align
        dp = self.translation_scale * (R_a @ (T_in[:3, 3] - T0[:3, 3]))
        # Rotation vector in robot base
        w = self.rotation_scale * (R_a @ matrix_to_rotvec(T_in[:3, :3] @ T0[:3, :3].T))
        dp *= self.dof_mask[:3]
        w *= self.dof_mask[3:]
        T_desired = make_transform(rotvec_to_matrix(w) @ self._target_ref[:3, :3],
                                   self._target_ref[:3, 3] + dp)
        return self._finish(T_desired, dt)

    def integrate(self, linear, angular, dt, frame='base'):
        """Compute the target pose from an input velocity.

        Args:
            linear: Linear velocity [x, y, z] in m/s.
            angular: Angular velocity [x, y, z] in rad/s.
            dt: Time since the last update in seconds.
            frame: 'base' or 'tool', the frame the velocity is expressed in.

        Returns:
            4x4 target pose in the robot base.
        """
        if not self.engaged:
            raise RuntimeError('integrate() called while not engaged')
        if frame not in TWIST_FRAMES:
            raise ValueError(f'frame must be one of {TWIST_FRAMES}, got {frame!r}')
        T_prev = self._last_target
        v = self.translation_scale * (self.align @ np.asarray(linear, dtype=float))
        w = self.rotation_scale * (self.align @ np.asarray(angular, dtype=float))
        if frame == 'tool':
            v = T_prev[:3, :3] @ v
            w = T_prev[:3, :3] @ w
        v *= self.dof_mask[:3]
        w *= self.dof_mask[3:]
        T_desired = make_transform(rotvec_to_matrix(w * dt) @ T_prev[:3, :3],
                                   T_prev[:3, 3] + v * dt)
        return self._finish(T_desired, dt)

    def _finish(self, T_desired, dt):
        if self.workspace_min is not None:
            T_desired[:3, 3] = np.maximum(T_desired[:3, 3], self.workspace_min)
        if self.workspace_max is not None:
            T_desired[:3, 3] = np.minimum(T_desired[:3, 3], self.workspace_max)
        T_out = self._limit_rate(self._last_target, T_desired, dt)
        self._last_target = T_out
        return T_out.copy()

    def _limit_rate(self, T_prev, T_next, dt):
        if dt <= 0.0:
            return T_next
        p_prev, p_next = T_prev[:3, 3], T_next[:3, 3]
        dp = p_next - p_prev
        if self.max_lin_vel > 0.0:
            max_step = self.max_lin_vel * dt
            n = np.linalg.norm(dp)
            if n > max_step:
                dp *= max_step / n
        R = T_next[:3, :3]
        if self.max_ang_vel > 0.0:
            w = matrix_to_rotvec(T_next[:3, :3] @ T_prev[:3, :3].T)
            max_step = self.max_ang_vel * dt
            n = np.linalg.norm(w)
            if n > max_step:
                R = rotvec_to_matrix(w * max_step / n) @ T_prev[:3, :3]
        return make_transform(R, p_prev + dp)
