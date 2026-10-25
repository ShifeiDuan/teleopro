# teleopro_core

[English](README.md) | 简体中文

用任意输入设备遥操作任意机器人，纯 Python 实现（只依赖 numpy，不需要 ROS）。它把设备（XR 头显、追踪器、动捕等）的运动转换成机器人的目标位姿，并提供离合、速度限制和输入丢失处理。

ROS 用户：请使用 `teleopro_ros` 软件包，它用 TF 封装了这个库。

## 安装

```bash
pip install teleopro-core
```

## 使用

```python
import time
from teleopro_core import Teleop, as_pos_quat

teleop = Teleop(translation_scale=0.5, max_lin_vel=0.1)

teleop.engage()                       # 例如按下按键时
while True:
    device_pose = my_device.get_pose()       # ((x, y, z), (qx, qy, qz, qw)) 或 4x4 矩阵，未追踪到时为 None
    robot_pose = my_robot.get_tcp_pose()     # 格式同上，在机器人基坐标系下
    target = teleop.update(time.monotonic(), device_pose, robot_pose)
    if target is not None:
        my_robot.move_to(*as_pos_quat(target))
    time.sleep(0.01)
```

- 调用 `engage()` 后，目标位姿从机器人当前位姿出发，然后跟随设备从那一刻起的运动。`disengage()` 后保持不动（离合）。
- 设备未被追踪（`None`）或 `input_stamp` 超过 `input_timeout` 时，目标位姿保持不动；追踪恢复后从停下的位置继续，不会跳变。
- 第一次 engage 之前，`update()` 返回启动时锁定的机器人位姿。

速度型设备（键盘、SpaceMouse）改用 `update_twist()`：

```python
teleop = Teleop(twist_frame='tool')      # 或 'base'
target = teleop.update_twist(time.monotonic(), ((vx, vy, vz), (wx, wy, wz)), robot_pose)
```

## 参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `align_rpy` | `(0, 0, 0)` | 从设备坐标轴到机器人坐标轴的旋转 |
| `translation_scale`, `rotation_scale` | `1.0` | 运动缩放 |
| `max_lin_vel`, `max_ang_vel` | `0.5`, `1.0` | 目标位姿的速度限制（m/s、rad/s），0 表示不限制 |
| `dof_mask` | `(1, 1, 1, 1, 1, 1)` | `[x, y, z, roll, pitch, yaw]`；0 锁定该自由度 |
| `workspace_min`, `workspace_max` | `None` | 目标位姿所在的 `[x, y, z]` 范围 |
| `input_timeout` | `0.2` | `input_stamp` 超过多少秒视为丢失 |
| `twist_frame` | `'base'` | `'base'` 或 `'tool'`，`update_twist()` 中速度所在的坐标系 |
| `logger` | `logging` | 任何带有 `.debug()` / `.info()` / `.warning()` 的对象 |

## 引用

见 [teleopro README](https://github.com/sdua078/teleopro/blob/main/README.zh-CN.md#引用)。

## 许可证

Apache-2.0
