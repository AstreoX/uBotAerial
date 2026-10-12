<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
  <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="96">
</picture>

# uBot Aerial

## 简介

uBotAerial 是 uBot 家族的多旋翼无人机插件，面向 UE 5.5，建立在 [uBotCore](https://github.com/AstreoX/uBotCore) 之上。

> 开发中：0.1.0 还没有发布，类名、属性和默认值都还会变化。

插件提供三类现成的四旋翼，按研究需要选用：

| 无人机 | 适合 | 接收的指令 |
| --- | --- | --- |
| `uBot Navigation Drone` | 研究导航、规划，不关心飞控 | 速度、偏航角速度 |
| `uBot Physics Drone` | 需要真实的动力学，用内置飞控 | 速度、姿态 |
| `uBot Bare Airframe Drone` | 研究飞控，控制器自己写 | 角速度加推力、单电机 |

另外还有风（恒定风加阵风）和键盘、手柄手动飞行。

- **没有 ROS 依赖**：插件不含 ROS 代码。要用 ROS 2 控制无人机，配合 [uBotROS](https://github.com/AstreoX/uBotROS) 使用。
- **可复现**：动力学由插件自己按仿真时钟以固定步长计算，与帧率无关。

## 版本与许可

当前开发版本 0.1.0，依赖 `UBotCore ^0.1.0` 和引擎插件 Enhanced Input。

Apache License 2.0，见 [LICENSE](LICENSE)。
