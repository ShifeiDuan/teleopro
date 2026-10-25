# teleopro_ros

[English](README.md) | 简体中文

[teleopro](../README.zh-CN.md) 的 ROS 2 节点，已在 ROS 2 Jazzy 和 UR5e 上经过测试。

## 编译

```bash
cd ~/ros2_ws/src
git clone https://github.com/sdua078/teleopro.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select teleopro_core teleopro_ros
```

## 使用

1. 根据设备选择输入类型：

   | `input_type` | 设备举例 |
   |---|---|
   | `pose` | XR 头显和手柄（Vision Pro、Quest、VIVE）、动捕（OptiTrack、Vicon） |
   | `twist` | SpaceMouse、摇杆或手柄、键盘 |

2. 在 [`config/teleop.yaml`](config/teleop.yaml) 中设置参数并启动。
3. 通过 `/teleopro/engage` engage。目标位姿从 `robot_source_frame` 出发，disengage 时保持不动。
4. 把目标位姿交给控制器：查询 TF `teleop_target`，或设置 `pose_topic` 得到 `PoseStamped`。

## 位姿输入

```bash
ros2 launch teleopro_ros teleop.launch.py \
    input_base_frame:=world input_source_frame:=right_wrist \
    robot_base_frame:=base_link robot_source_frame:=tool0 \
    pose_topic:=/my_controller/target_frame
ros2 service call /teleopro/engage std_srvs/srv/SetBool "{data: true}"
```

| 订阅 | 类型 | 说明 |
|---|---|---|
| TF `input_base_frame → input_source_frame` | TF | 设备位姿 |

| 参数 | 默认值 | 含义 |
|---|---|---|
| `input_type` | `pose` | |
| `input_base_frame` | 必填 | 设备的固定坐标系 |
| `input_source_frame` | 必填 | 驱动机器人的被追踪坐标系 |

## 速度输入

```bash
ros2 launch teleopro_ros teleop.launch.py input_type:=twist twist_topic:=/spacenav/twist \
    twist_frame:=tool robot_base_frame:=base_link robot_source_frame:=tool0 \
    pose_topic:=/my_controller/target_frame
ros2 service call /teleopro/engage std_srvs/srv/SetBool "{data: true}"
```

| 设备 | 驱动 | `twist_topic` |
|---|---|---|
| SpaceMouse | `spacenav` | `/spacenav/twist` |
| 摇杆 | `teleop_twist_joy` | `/cmd_vel` |
| 键盘 | `teleop_twist_keyboard` | `/cmd_vel` |

| 订阅 | 类型 | 说明 |
|---|---|---|
| `twist_topic` | `geometry_msgs/Twist` | 设备速度；设置 `twist_stamped` 时为 `TwistStamped` |

| 参数 | 默认值 | 含义 |
|---|---|---|
| `input_type` | `twist` | 必须设置，默认值是 `pose` |
| `twist_topic` | `/teleopro/twist` | 输入话题 |
| `twist_frame` | `base` | `base`：沿 `robot_base_frame` 的坐标轴；`tool`：沿末端执行器自身的坐标轴 |
| `twist_stamped` | `false` | 改为订阅 `TwistStamped` |

超过 `input_timeout` 没有收到 twist 时，目标位姿停止移动。

## 公共部分

| | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | TF `robot_base_frame → robot_source_frame` | TF | 末端执行器位姿，在启动时和 engage 时读取，作为目标位姿的起点 |
| 发布 | TF `robot_base_frame → target_frame` | TF | 目标位姿 |
| | `pose_topic` | `geometry_msgs/PoseStamped` | 目标位姿，设置了 `pose_topic` 时发布 |
| | `~/engaged` | `std_msgs/Bool` | engage 状态，latched |
| 服务 | `~/engage` | `std_srvs/SetBool` | `true` 开始，`false` 停止 |

| 参数 | 默认值 | 含义 |
|---|---|---|
| `robot_base_frame` | 必填 | 目标位姿所在的坐标系 |
| `robot_source_frame` | 必填 | 被控制的末端执行器 |
| `pose_topic` | `""` | 输出 `PoseStamped` 的话题；`""` 表示只发布 TF |
| `align_rpy` | `[0, 0, 0]` | 从设备坐标轴到机器人坐标轴的旋转 |
| `translation_scale`, `rotation_scale` | `1.0` | 运动缩放 |
| `max_lin_vel`, `max_ang_vel` | `0.5`, `1.0` | 目标位姿的速度限制（m/s、rad/s） |
| `rate` | `100.0` | 更新频率（Hz） |

上面的参数也可以作为 launch 参数传入。`twist_stamped`、`target_frame`、`dof_mask`、`workspace_*` 和 `input_timeout` 只能在 yaml 中设置，见 [`config/teleop.yaml`](config/teleop.yaml)。

## 注意事项

- 目前只支持笛卡尔控制：需要把目标位姿交给笛卡尔控制器或 IK。
- 在真机上测试时，刚开始要把手放在示教器的停止按钮上，并调低机械臂速度和 `max_lin_vel` / `max_ang_vel`。
- `robot_source_frame` 必须是控制器的末端执行器 link，否则 engage 时机器人会偏移这两个坐标系之间的距离。
- `align_rpy` 没有平移分量：只使用相对运动，位置偏移会相互抵消。操作者面对机器人时用 `[0, 0, 3.14159]`。
