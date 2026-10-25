# teleopro_core

English | [简体中文](https://github.com/sdua078/teleopro/blob/main/teleopro_core/README.zh-CN.md)

Teleoperate any robot with any input device, in pure Python (numpy only, no
ROS). It turns the motion of a device (XR headset, tracker, mocap, ...) into a
target pose for the robot, with clutch, speed limits and input-loss handling.

ROS users: use the `teleopro_ros` package, which wraps this library with TF.

## Install

```bash
pip install teleopro-core
```

## Usage

```python
import time
from teleopro_core import Teleop, as_pos_quat

teleop = Teleop(translation_scale=0.5, max_lin_vel=0.1)

teleop.engage()                       # e.g. when a button is pressed
while True:
    device_pose = my_device.get_pose()       # ((x, y, z), (qx, qy, qz, qw)) or 4x4, None if not tracked
    robot_pose = my_robot.get_tcp_pose()     # same formats, in the robot base frame
    target = teleop.update(time.monotonic(), device_pose, robot_pose)
    if target is not None:
        my_robot.move_to(*as_pos_quat(target))
    time.sleep(0.01)
```

- On `engage()` the target starts at the current robot pose, then follows the
  device's motion since that moment. `disengage()` holds it still (clutch).
- If the device is not tracked (`None`) or its `input_stamp` is older than
  `input_timeout`, the target holds; when tracking returns it continues from
  where it stopped, without jumping.
- Before the first engage, `update()` returns the robot pose latched once.

Velocity devices (keyboard, SpaceMouse) use `update_twist()` instead:

```python
teleop = Teleop(twist_frame='tool')      # or 'base'
target = teleop.update_twist(time.monotonic(), ((vx, vy, vz), (wx, wy, wz)), robot_pose)
```

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `align_rpy` | `(0, 0, 0)` | Rotation from device axes to robot axes |
| `translation_scale`, `rotation_scale` | `1.0` | Motion scaling |
| `max_lin_vel`, `max_ang_vel` | `0.5`, `1.0` | Target speed limits (m/s, rad/s), 0 = off |
| `dof_mask` | `(1, 1, 1, 1, 1, 1)` | `[x, y, z, roll, pitch, yaw]`; 0 freezes a DoF |
| `workspace_min`, `workspace_max` | `None` | `[x, y, z]` box the target stays in |
| `input_timeout` | `0.2` | Seconds before a stale `input_stamp` counts as lost |
| `twist_frame` | `'base'` | `'base'` or `'tool'`, frame of the twist in `update_twist()` |
| `logger` | `logging` | Anything with `.debug()` / `.info()` / `.warning()` |

## Citation

See the [teleopro README](https://github.com/sdua078/teleopro#citation).

## License

Apache-2.0
