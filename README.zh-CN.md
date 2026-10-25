<div align="center">

# Teleopro

**遥操大师：用任意输入设备操控任意机械臂。**

[![PyPI](https://img.shields.io/pypi/v/teleopro-core?color=blue)](https://pypi.org/project/teleopro-core/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/python-3.8%2B-3776AB?logo=python&logoColor=white)](https://pypi.org/project/teleopro-core/)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)
[![arXiv](https://img.shields.io/badge/arXiv-2610.25006-b31b1b)](https://arxiv.org/abs/2610.25006)

[English](README.md) | 简体中文

</div>

给出设备上的两个坐标系和机械臂上的两个坐标系，teleopro 就会输出一个目标位姿，跟随设备的运动而移动：

<p align="center">
  <img src="docs/architecture.svg" alt="输入设备 → teleopro → 你的控制器 → 机械臂" width="900">
</p>

teleopro 不直接控制机械臂。把目标位姿交给你自己的控制方式即可（IK、笛卡尔控制器、伺服控制等）。

- 支持位姿型设备（XR 头显、动捕）和速度型设备（SpaceMouse、摇杆、键盘）
- 离合：engage 后跟随，disengage 后保持；目标位姿不会跳变
- 方向映射、运动缩放、速度限制、工作空间限制、自由度锁定
- 已在 Universal Robots UR5e（ROS 2 Jazzy）上测试

## 软件包

| 软件包 | 适用于 | 安装 |
|---|---|---|
| [`teleopro_ros`](teleopro_ros/README.zh-CN.md) | ROS 2：在 yaml 中配置坐标系，得到一个 TF | 从源码编译，请参考 [README](teleopro_ros/README.zh-CN.md#编译) |
| [`teleopro_core`](teleopro_core/README.zh-CN.md) | 任意 Python 程序，不需要 ROS | `pip install teleopro-core` |

`teleopro_ros` 是 `teleopro_core` 的一层轻量 ROS 2 封装，两者行为完全一致。

## 限制与安全

- **目前只支持笛卡尔控制。** teleopro 输出的是末端执行器的笛卡尔目标位姿，需要笛卡尔控制器或 IK 来执行，不支持关节空间控制。
- **在真机上测试时：**
  - 开始测试时，请把手放在示教器的停止按钮上。
  - 从慢速开始：请调低机械臂速度，并从较小的 `max_lin_vel` / `max_ang_vel` 开始（例如 `0.05` m/s、`0.2` rad/s）。

## 引用

如果你在研究中使用了 teleopro，请引用：

```bibtex
@article{duan2026scalable,
  title={Scalable, Intuitive, Mixed Reality-Based Semi-Autonomous Robotic Telemanipulation for the Execution of a Fine-Grained Bimanual Task},
  author={Duan, Shifei and Wang, Zhe and De Pace, Francesco and Liarokapis, Minas},
  journal={arXiv preprint arXiv:2610.25006},
  year={2026}
}
```

## 许可证

Apache-2.0
