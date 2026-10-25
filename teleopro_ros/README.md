# teleopro_ros

English | [简体中文](README.zh-CN.md)

ROS 2 node for [teleopro](../README.md). Tested on ROS 2 Jazzy with a UR5e.

## Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/sdua078/teleopro.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select teleopro_core teleopro_ros
```

## Usage

1. Choose the input type for your device:

   | `input_type` | Examples |
   |---|---|
   | `pose` | XR headsets and controllers (Vision Pro, Quest, VIVE), motion capture (OptiTrack, Vicon) |
   | `twist` | SpaceMouse, joystick or gamepad, keyboard |

2. Set the parameters in [`config/teleop.yaml`](config/teleop.yaml) and launch.
3. Engage with `/teleopro/engage`. The target starts at `robot_source_frame` and
   holds still while disengaged.
4. Feed the target to your controller: look up TF `teleop_target`, or set
   `pose_topic` to get a `PoseStamped`.

## Pose input

```bash
ros2 launch teleopro_ros teleop.launch.py \
    input_base_frame:=world input_source_frame:=right_wrist \
    robot_base_frame:=base_link robot_source_frame:=tool0 \
    pose_topic:=/my_controller/target_frame
ros2 service call /teleopro/engage std_srvs/srv/SetBool "{data: true}"
```

| Subscribes | Type | Notes |
|---|---|---|
| TF `input_base_frame → input_source_frame` | TF | Device pose |

| Parameter | Default | Meaning |
|---|---|---|
| `input_type` | `pose` | |
| `input_base_frame` | required | Fixed frame of the device |
| `input_source_frame` | required | Tracked frame that drives the robot |

## Twist input

```bash
ros2 launch teleopro_ros teleop.launch.py input_type:=twist twist_topic:=/spacenav/twist \
    twist_frame:=tool robot_base_frame:=base_link robot_source_frame:=tool0 \
    pose_topic:=/my_controller/target_frame
ros2 service call /teleopro/engage std_srvs/srv/SetBool "{data: true}"
```

| Device | Driver | `twist_topic` |
|---|---|---|
| SpaceMouse | `spacenav` | `/spacenav/twist` |
| Joystick | `teleop_twist_joy` | `/cmd_vel` |
| Keyboard | `teleop_twist_keyboard` | `/cmd_vel` |

| Subscribes | Type | Notes |
|---|---|---|
| `twist_topic` | `geometry_msgs/Twist` | Device velocity; `TwistStamped` if `twist_stamped` |

| Parameter | Default | Meaning |
|---|---|---|
| `input_type` | `twist` | Must be set, the default is `pose` |
| `twist_topic` | `/teleopro/twist` | Input topic |
| `twist_frame` | `base` | `base`: along the `robot_base_frame` axes; `tool`: along the end effector's own axes |
| `twist_stamped` | `false` | Subscribe to `TwistStamped` instead |

The target stops if no twist arrives for `input_timeout`.

## Common

| | Name | Type | Notes |
|---|---|---|---|
| Subscribes | TF `robot_base_frame → robot_source_frame` | TF | End effector pose, read at startup and on engage as the target's start pose |
| Publishes | TF `robot_base_frame → target_frame` | TF | Target pose |
| | `pose_topic` | `geometry_msgs/PoseStamped` | Target pose, if `pose_topic` is set |
| | `~/engaged` | `std_msgs/Bool` | Engage state, latched |
| Services | `~/engage` | `std_srvs/SetBool` | `true` engages, `false` disengages |

| Parameter | Default | Meaning |
|---|---|---|
| `robot_base_frame` | required | Frame the target is expressed in |
| `robot_source_frame` | required | Controlled end effector |
| `pose_topic` | `""` | Output `PoseStamped` topic; `""` = TF only |
| `align_rpy` | `[0, 0, 0]` | Rotation from device axes to robot axes |
| `translation_scale`, `rotation_scale` | `1.0` | Motion scaling |
| `max_lin_vel`, `max_ang_vel` | `0.5`, `1.0` | Target speed limits (m/s, rad/s) |
| `rate` | `100.0` | Update rate (Hz) |

The parameters above can also be launch arguments. `twist_stamped`, `target_frame`,
`dof_mask`, `workspace_*` and `input_timeout` are yaml only, see
[`config/teleop.yaml`](config/teleop.yaml).

## Notes

- Cartesian control only: feed the target to a Cartesian controller or IK.
- When testing on a real robot, keep a hand on the teach pendant's stop button
  at the beginning, and lower the robot speed and `max_lin_vel` / `max_ang_vel`.
- `robot_source_frame` must be your controller's end effector link, otherwise the
  robot shifts by the offset between the two frames when engaging.
- `align_rpy` has no translation: only relative motion is used, so position
  offsets cancel. Operator facing the robot: `[0, 0, 3.14159]`.
