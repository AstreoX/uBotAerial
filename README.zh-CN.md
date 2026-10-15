<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
    <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
    <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="88">
  </picture>
</p>
<h1 align="center">uBot Aerial</h1>
<p align="center"><b>面向 Unreal Engine 5.5 的多旋翼无人机：从简化的导航飞行到刚体动力学。</b></p>
<p align="center">
  <a href="https://github.com/AstreoX/uBotAerial/releases"><img alt="release" src="https://img.shields.io/github/v/release/AstreoX/uBotAerial?label=release"></a>
  <a href="LICENSE"><img alt="license" src="https://img.shields.io/github/license/AstreoX/uBotAerial"></a>
  <img alt="Unreal Engine 5.5" src="https://img.shields.io/badge/Unreal%20Engine-5.5-313131">
</p>
<p align="center">
  <a href="https://astreox.github.io/uBotDocs/aerial/">文档</a> ·
  <a href="https://github.com/AstreoX/uBotAerial/releases">发布版本</a> ·
  <a href="https://github.com/users/AstreoX/projects/2">路线图</a> ·
  <a href="https://github.com/AstreoX/uBotAerial/issues">问题反馈</a>
</p>

[English](README.md) | **简体中文**

## 简介

uBot Aerial 是 uBot 家族的多旋翼无人机插件，适用于 Unreal Engine 5.5。插件提供三个示例无人机蓝图，涵盖面向导航研究的简化飞行，以及带内置飞控的刚体动力学，并附演示地图。插件不依赖 ROS，通过 [uBotROS](https://github.com/AstreoX/uBotROS) 可从 ROS 2 控制无人机。

- **简化飞行：** 速度指令按一阶响应跟踪，带加速度上限。
- **刚体动力学：** 转子推力、电机滞后、阻力和风；接收速度、姿态、角速度和单电机指令。
- **自定义飞控：** 在蓝图或 C++ 中继承 `uBot Flight Controller`，替换内置飞控。
- **手动飞行：** 键盘或手柄，Velocity、Angle、Rate 三种模式。
- **风与碰撞：** 恒定风加带种子的阵风；无人机沿墙和地面滑动。

| 示例无人机 | 用途 | 接收的指令 |
| --- | --- | --- |
| `BP_UBotNavigationDrone` | 导航、规划、感知 | 速度、偏航角速度 |
| `BP_UBotPhysicsDrone` | 真实的动力学，使用内置飞控 | 速度、姿态、角速度、单电机 |
| `BP_UBotBareAirframeDrone` | 飞控研究，使用自己编写的控制器 | 角速度、单电机 |

## 安装

**uBot Manager（推荐）。** [uBot Manager](https://github.com/AstreoX/uBotManager) 是 Windows 桌面程序，安装插件时自动处理依赖。

1. 从 [uBot Manager 的 Releases](https://github.com/AstreoX/uBotManager/releases) 下载并运行安装包。
2. 在 uBot Manager 中打开工程的 `.uproject`，在 Package 视图选择 **UBotAerial**，依次点击「安装」「应用…」。
3. 用 UE 5.5 打开工程，`Window > uBot` 的「已加载的 Package」中应列出 UBotAerial。

**手动安装。** 先安装 [uBotCore](https://github.com/AstreoX/uBotCore)，再将本仓库克隆到其同级目录，在 `Edit > Plugins` 中启用 **uBot Aerial** 并重启编辑器：

```powershell
git clone --branch v0.1.0 https://github.com/AstreoX/uBotAerial.git <Project>/Plugins/uBot/uBotAerial
```

要求：Unreal Engine 5.5，uBotCore `^0.1.2`，Enhanced Input（同时作为工程的 `Default Player Input Class`），源码构建需要 Visual Studio 2022。

## 快速开始

在演示地图中操控物理无人机：

1. 在 Content Browser 中启用 `Settings > Show Plugin Content`。
2. 打开 `/UBotAerial/Maps/L_UBotAerialDemo` 并点击「Play」。地图中的物理无人机由 Player 0 控制。
3. 键盘：Space 上升，Left Ctrl 下降，W / A / S / D 平移，E / Q 偏航。手柄：右摇杆平移，左摇杆升降和偏航。
4. 切换无人机：将另一架的 `Auto Possess Player` 设为 Player 0，原来那架设回 `Disabled`。

## 文档

手册位于[文档站](https://astreox.github.io/uBotDocs/aerial/)，同时包含碰撞、常见问题和 API 参考。

| 目的 | 文档 |
| --- | --- |
| 在演示地图中操控示例无人机 | [示例无人机与试飞关卡](https://astreox.github.io/uBotDocs/aerial/examples/) |
| 向导航无人机发送速度指令 | [用速度指令飞导航无人机](https://astreox.github.io/uBotDocs/aerial/simple-flight/) |
| 发送姿态、角速度或电机指令 | [物理无人机的指令与控制模式](https://astreox.github.io/uBotDocs/aerial/dynamics/) |
| 编写自己的飞控 | [自己写飞控](https://astreox.github.io/uBotDocs/aerial/flight-controller/) |
| 用键盘或手柄飞行 | [键盘和手柄手动飞行](https://astreox.github.io/uBotDocs/aerial/manual-flight/) |
| 添加风 | [风](https://astreox.github.io/uBotDocs/aerial/wind/) |
| 用组件搭建无人机 | [自己拼一架无人机](https://astreox.github.io/uBotDocs/aerial/build-a-drone/) |
| 从 ROS 2 控制无人机 | [从 ROS 2 发飞行指令](https://astreox.github.io/uBotDocs/ros/flight-commands/) |
| 查看 C++ 示例 | [docs/CPP.md](docs/CPP.md) |
| 查看飞行模型细节与验收指标 | [docs/FLIGHT-MODELS.md](docs/FLIGHT-MODELS.md) |
| 构建插件并运行测试 | [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) |

## uBot 家族

| 仓库 | 说明 |
| --- | --- |
| [uBotCore](https://github.com/AstreoX/uBotCore) | 仿真时钟、帧头、共享接口、插件管理 |
| [uBotSensor](https://github.com/AstreoX/uBotSensor) | 位姿、IMU、里程计、2D/3D 激光雷达、RGB 与深度相机 |
| [uBotROS](https://github.com/AstreoX/uBotROS) | 通过 TCP 桥接入 ROS 2：传感器、TF、/clock、cmd_vel |
| **[uBotAerial](https://github.com/AstreoX/uBotAerial)** | **多旋翼无人机：简化飞行、刚体动力学、风** |
| [uBotManager](https://github.com/AstreoX/uBotManager) | 在 UE 工程里安装、更新、构建 uBot 包的桌面程序和命令行 |
| [uBotDocs](https://github.com/AstreoX/uBotDocs) | 文档站：https://astreox.github.io/uBotDocs/ |

## 开发

- 构建：将仓库放到 UE 5.5 C++ 工程的 `Plugins/uBot/uBotAerial`，与 uBotCore 并列，然后构建工程。
- 测试：Unreal 自动化测试，名称以 `UBotAerial.` 开头，在 Session Frontend（Automation 标签页）或命令行中运行，详见 [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)。
- 反馈：问题与建议请提交到 [Issues](https://github.com/AstreoX/uBotAerial/issues)。

## 许可

Copyright 2026 AstreoX。代码以 [Apache-2.0](LICENSE) 发布，另见 [NOTICE](NOTICE)。

`Content/Models` 里的无人机模型（`SM_UBotDrone_*`、`M_UBotDrone`、`T_UBotDrone_*`）基于 Fab 上 HARTWEL JUNIOR KISAKA 的 [Animated Drone](https://www.fab.com/listings/c1f5f392-330a-418c-b0b0-e3c89639347f)，授权为 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。Apache-2.0 不适用于该模型。

改动如下：将带蒙皮的 glTF 模型转为静态网格（一个机身网格，以及以桨毂为中心的顺时针和逆时针桨叶网格），整体放大 2 倍，机头朝向 +X，材质合并为一个，贴图缩至 1024x1024 并由 specular-glossiness 转为金属度-粗糙度流程，法线贴图翻转绿色通道。不含动画。
