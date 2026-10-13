<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
  <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="96">
</picture>

# uBot Aerial

## 简介

uBotAerial 是 uBot 家族的多旋翼无人机插件，为 UE 5.5 提供三个现成的四旋翼 X 无人机示例蓝图、一张演示地图、风，以及键盘和手柄手动飞行。它建立在 [uBotCore](https://github.com/AstreoX/uBotCore) 之上。

> 0.1.0 尚未发布，发布前类名、属性和默认值还可能变化。

三个示例无人机对应三类研究需要：

| 无人机 | 适合 | 接收的指令 |
| --- | --- | --- |
| `BP_UBotNavigationDrone` | 导航、规划、感知，不关心飞控 | 速度、偏航角速度 |
| `BP_UBotPhysicsDrone` | 需要真实的动力学，直接用内置飞控 | 速度、姿态、角速度、单电机 |
| `BP_UBotBareAirframeDrone` | 研究飞控，控制器自己写 | 角速度、单电机 |

- **蓝图优先**：三架无人机是插件 Content 里的示例蓝图，由插件的 C++ 组件拼成，可以直接放进关卡，也可以复制出来修改或照着自己拼；飞行模型、风和手动输入都有蓝图接口。
- **没有 ROS 依赖**：插件不含 ROS 代码。用 ROS 2 控制无人机时配合 [uBotROS](https://github.com/AstreoX/uBotROS)，它通过 UBotCore 的指令接口驱动这里的无人机。
- **可复现**：飞行模型按 uBot 仿真时间以固定步长推进，同样的帧间隔给出逐位相同的结果。
- **示例内容**：插件带三个示例蓝图、演示地图 `L_UBotAerialDemo` 和一套无人机模型（来源和授权见“模型来源与授权”）。示例蓝图不含节点逻辑，行为都在 C++ 组件里。

## 在 uBot 家族中的位置

uBotAerial 是 **Capability**（能力）层，依赖 `UBotCore ^0.1.2`（姿态和电机指令接口从 0.1.2 起提供）和引擎插件 Enhanced Input。它不依赖 uBotSensor 和 uBotROS。

| Package | 关系 |
| --- | --- |
| UBotCore | 依赖。提供速度、姿态、电机指令接口，仿真时钟，身份组件和扩展注册表 |
| UBotSensor | 无依赖。传感器组件可以直接加到无人机蓝图上，见“传感器” |
| UBotROS | 无依赖。UBotROS 的指令组件通过 UBotCore 接口控制无人机，见“从 ROS 2 发送指令” |

| 模块 | 类型 | 加载阶段 | 说明 |
| --- | --- | --- | --- |
| `UBotAerial` | Runtime | Default | 飞行模型、飞控、旋翼、风、手动输入 |

`"UBot"` 块声明的能力键（`Provides`）：`Actuation.Velocity`、`Actuation.Attitude`、`Actuation.Motor`、`Body.Multirotor`、`Aerial.Wind`；标签 `aerial`。模块依赖 `Core`、`CoreUObject`、`Engine`、`EnhancedInput`、`InputCore` 和 `UBotCore`，日志类别只有 `LogUBotAerial`。

## 安装

面向 UE 5.5。插件以源码形式分发，工程需要能编译 C++。先安装 uBotCore。

1. 把仓库克隆到 uBotCore 旁边，即 `<Project>/Plugins/uBot/uBotAerial`：

   ```powershell
   git clone https://github.com/AstreoX/uBotAerial.git <Project>/Plugins/uBot/uBotAerial
   ```

   也可以用 [uBot Manager](https://github.com/AstreoX/uBotManager) 安装（在 uBotCore 的 **uBot** 面板，`Window > uBot`，里点 *Open uBot Manager* 打开），它会先装好缺失的依赖；命令行等价于 `UnrealEditor-Cmd.exe <Project>.uproject -run=UBotPackage -Install=UBotAerial`。

2. 启用插件：在 `Edit > Plugins` 中搜索 uBot Aerial（分类 uBot）并勾选，或在 `.uproject` 中加入

   ```json
   "Plugins": [
       { "Name": "UBotCore", "Enabled": true },
       { "Name": "UBotAerial", "Enabled": true }
   ]
   ```

3. 重启编辑器。示例无人机在插件的 Content 里，见“快速开始”；组件在 Add Component 菜单的 **uBot** 分类下，显示名形如 `uBot Simple Flight`。

C++ 工程使用时，在自己模块的 `Build.cs` 中加入 `"UBotCore"` 和 `"UBotAerial"`。

手动飞行用 Enhanced Input：工程的 `Project Settings > Input > Default Player Input Class` 要是 Enhanced Input 的玩家输入类，否则日志里会报一次错，无人机不响应按键。

## 快速开始

先看演示地图：

1. 在 Content Browser 里打开 `Settings > Show Plugin Content`，才看得到插件的内容。
2. 打开 `/UBotAerial/Maps/L_UBotAerialDemo`，点「Play」。地图里的物理无人机已设为由 Player 0 控制：Space 上升，W / A / S / D 平移，E / Q 偏航，Left Ctrl 下降。要飞另一架，把 `Auto Possess Player` 移到它身上，见“演示地图”。

放进自己的关卡：

1. 把 `/UBotAerial/Examples/BP_UBotNavigationDrone` 拖进关卡。无人机在放置的位置悬停（放在地面上就停在地面上），任何指令都会让它立即动起来。
2. 让玩家控制它：在 Details 里把 `Auto Possess Player` 设为 Player 0，或在 GameMode 里把 `Default Pawn Class` 设为它。
3. 点「Play」，按键同上。
4. 不用玩家控制时，在蓝图里用 `Find Multirotor Movement`（或对无人机用 `Get Component by Class`）取得 `Movement` 组件，对它调用 `Apply Velocity Command`：`Linear Velocity Meters Per Second` 填 (2, 0, 0)，`Angular Velocity Radians Per Second` 填 (0, 0, 0.5)。无人机以 2 m/s 前进并向右偏航，直到新指令替换它或调用 `Stop Motion`。

要修改示例，先把它复制到自己的工程里，见“示例内容”。

把无人机放在地面上时，根组件中心离地 6.7 cm（碰撞盒的半高），碰撞盒刚好贴地。

## 三种无人机

三个示例蓝图的机体、碰撞、相机、手动输入和指令入口都一样，区别只在 `Movement` 组件用哪一种飞行模型。

| | `BP_UBotNavigationDrone` | `BP_UBotPhysicsDrone` | `BP_UBotBareAirframeDrone` |
| --- | --- | --- | --- |
| 飞行模型 | `uBot Simple Flight` | `uBot Multirotor Dynamics` | `uBot Multirotor Dynamics` |
| 适合 | 导航、规划、感知 | 需要真实动力学，用内置飞控 | 研究飞控，控制器自己写 |
| 接受的指令 | 速度 + 偏航角速度 | 速度、姿态、角速度、电机 | 角速度、电机 |
| 拒绝的指令 | 姿态、角速度、电机（没有对应接口） | 无 | 速度、姿态 |
| 物理步频 `PhysicsRateHz` | 500 Hz | 1000 Hz | 1000 Hz |
| 示例里的手动飞行模式 | `Velocity` | `Velocity` | `Rate` |
| 旋翼转速 | 只用于显示 | 真实的电机转速 | 真实的电机转速 |
| 风 | 简化响应 | 相对空气的阻力 | 相对空气的阻力 |

### 示例内容

插件的 Content 在 Content Browser 里默认不显示，打开 `Settings > Show Plugin Content` 后可见：

| 路径 | 内容 |
| --- | --- |
| `/UBotAerial/Examples/` | `BP_UBotNavigationDrone`、`BP_UBotPhysicsDrone`、`BP_UBotBareAirframeDrone`，父类是 Pawn，不含节点逻辑 |
| `/UBotAerial/Maps/L_UBotAerialDemo` | 演示地图，见“演示地图” |
| `/UBotAerial/Models/` | 无人机模型：`SM_UBotDrone_Body`、`SM_UBotDrone_Propeller_CW`、`SM_UBotDrone_Propeller_CCW`、材质 `M_UBotDrone`、贴图 `T_UBotDrone_BaseColor`、`T_UBotDrone_ORM`、`T_UBotDrone_Normal` |

示例可以直接放进关卡使用。要修改，先复制到自己的工程再改：在示例上右键 `Duplicate` 后移到 `/Game` 下，或用 `Asset Actions > Migrate`，改复制出来的那一份。直接改插件里的原资产，更新插件时改动会丢失。

### 演示地图

`/UBotAerial/Maps/L_UBotAerialDemo` 里有：

- 一块大的网格地面、一堵墙、一堵 45° 的斜墙（用来沿墙滑动）、一个 1 m 高的平台和一根立柱；
- 三个示例无人机并排，各有名称标签；墙上有操作提示；
- 一个默认关闭的 `uBot Wind`：恒定风 (0, 5, 0) m/s，阵风标准差 (1, 1, 0.3) m/s。把它打开，可以看到物理无人机迎风倾斜；
- World Settings 的 `GameMode Override` 是 `GameModeBase`；物理无人机的 `Auto Possess Player` 是 Player 0。

打开地图点「Play」即可飞。要飞另一架，把 `Auto Possess Player` 设到它身上，原来那架改回 `Disabled`。

### 机体与组件

三个示例蓝图的父类是 Pawn，组件完全相同。下面的变量名就是组件名，加传感器、改属性、用 Get Component 找组件时按它们找。

| 组件（变量名） | 类型 | 说明 |
| --- | --- | --- |
| `CollisionBox`（根） | Box | 半尺寸 (19.5, 25.4, 6.7) cm（整体 39 × 50.8 × 13.4 cm），碰撞预设 Pawn，不模拟物理。飞行模型扫掠这个形状 |
| `VisualRoot` | Scene | 网格的父级。导航无人机的 `Visual` 倾斜作用于它 |
| `Body` | Static Mesh | `SM_UBotDrone_Body`，没有碰撞 |
| `Rotor0`..`Rotor3` | `uBot Rotor` | 桨叶网格放在电机轴上，`Rotor Index` 为 0..3，按 PX4 四旋翼 X 的电机顺序；0、1 用 `SM_UBotDrone_Propeller_CCW`，2、3 用 `SM_UBotDrone_Propeller_CW`。随对应电机的转速旋转 |
| `CameraBoom` / `Camera` | Spring Arm / Camera | 跟随相机，见“手动飞行” |
| `Identity` | `uBot Identity` | 机器人 ID、ROS 命名空间、基坐标系 |
| `ManualInput` | `uBot Multirotor Manual Input` | 键盘和手柄，见“手动飞行” |
| `Movement` | `uBot Simple Flight` 或 `uBot Multirotor Dynamics` | 飞行模型，导航示例用前者，物理和裸机架示例用后者 |

蓝图本身的默认值：`Auto Possess AI` 是 `Disabled`，`Start with Tick Enabled` 关闭（行为都在组件里）。

三个示例在 `Movement` 上的设置：

| 示例 | `Movement` 的设置 |
| --- | --- |
| `BP_UBotNavigationDrone` | `uBot Simple Flight`，`Visual Component` 指向 `VisualRoot` |
| `BP_UBotPhysicsDrone` | `uBot Multirotor Dynamics`；`Airframe` 的转子位置设为模型的电机轴（下表），质量、转动惯量和转子系数保持默认（1.5 kg，450 mm 级） |
| `BP_UBotBareAirframeDrone` | 同物理示例，另外 `Accept Velocity Commands` 和 `Accept Attitude Commands` 关闭，`ManualInput` 的 `Flight Mode` 为 `Rate` |

要改飞行模型的属性或加组件，在复制出来的蓝图的 Components 面板里选 `Movement`。

`uBot Rotor` 的 `RotorIndex` 是电机序号。`Spin Direction` 由 `Movement` 在 BeginPlay 时按序号从飞行模型的转子布局设置，在 Details 里只读；布局可以用 `Get Rotor Layout` 读出，返回按电机序号排列的转子位置（m，机体系，UE 轴）和旋向。显示的转速是真实转速乘 `VisualSpinScale`（0.04），避免在帧率下混叠。

示例里 4 个旋翼放在模型的电机轴上，相对根组件为 (±14.53, ±17.08, 4.75) cm。电机序号和位置（机体系，X 前、Y 右），最后一列是物理示例机架里的转子位置：

| 电机 | 位置 | 旋向（俯视） | 转子位置（m） |
| --- | --- | --- | --- |
| 0 | 右前 | 逆时针 | (0.1453, 0.1708) |
| 1 | 左后 | 逆时针 | (−0.1453, −0.1708) |
| 2 | 左前 | 顺时针 | (0.1453, −0.1708) |
| 3 | 右后 | 顺时针 | (−0.1453, 0.1708) |

默认机架的臂长是 0.225 m，转子位于 (±D, ±D)，D = 0.225 / √2 ≈ 0.159 m；物理示例的机架只把转子位置换成上表（模型的臂长 0.224 m，与默认几乎相同）。机体受到与转子旋向相反的反扭矩。

### 自己拼一架无人机

最简单的办法是复制一个示例再改。从零拼：

1. 新建一个 Pawn（或 Actor）蓝图。根组件用一个碰撞形状（Box、Sphere 或 Capsule）：飞行模型扫掠的是根组件的形状。
2. 加 `uBot Simple Flight`（导航）或 `uBot Multirotor Dynamics`（物理）。它以根组件为 `Updated Component`，不需要另外指定。用 `Visual` 倾斜时，把 `Visual Component` 指向要倾斜的 Scene 组件。
3. 可选：加 `uBot Rotor` 组件，`Rotor Index` 设为电机序号，放在 `Get Rotor Layout` 返回的位置（m）乘 100 cm 处，相对根组件。它们的旋向随机架自动设置。
4. 可选：加 `uBot Multirotor Manual Input` 用键盘和手柄飞，加 `uBot Identity` 设机器人 ID 和 ROS 命名空间，加弹簧臂相机（把 `Inherit Pitch` 和 `Inherit Roll` 关掉，视角才不会随机体倾斜）。

### 共有属性

下面的属性在两种飞行模型的基类 `uBot Multirotor Movement`（`UUBotMultirotorMovementComponent`，抽象）上。

| 属性 | 默认 | 说明 |
| --- | --- | --- |
| `PhysicsRateHz` | 导航 500，物理 1000 | 每秒物理步数，限制在 [100, 4000] |
| `MaxStepsPerFrame` | 250 | 一帧最多跑的物理步数。需要更多时多出的时间被跳过，日志只警告一次 |
| `VelocityLimits` | 水平 10 m/s，爬升 3 m/s，下降 2 m/s，偏航 π/2 rad/s | 速度指令超过时被截断，水平方向保持不变 |
| `ImpactEventMinSpeedMetersPerSecond` | 0.3 m/s | 慢于它的撞击不触发 `OnImpact` |
| `ContactFriction` | 0.3 | 接触时滑动速度减少撞击速度的这个比例 |
| `bEnableWind` | 开 | 这架无人机是否受 `uBot Wind` 影响 |
| `ActuatorId` | 空 | 空则取 `uBot Identity` 的机器人 ID（没有身份组件时取 Actor 名）。执行器类型是 `Multirotor` |

### 单位与坐标

- 内部全部用 SI 单位和 UE 世界轴（X 前、Y 右、Z 上）：米、秒、弧度；名字带 `Degrees` 的属性除外。
- 厘米只出现在与引擎交互的边界：Actor 位置、`Teleport Multirotor` 和 `Get Wind Velocity At` 的位置参数。
- 姿态是 `FQuat`（机体到世界）。角速度在机体系，UE 旋向：+Z 向右偏航。
- 速度指令在**航向系**：X 沿机头在水平面上的投影，Y 向右，Z 向上，与机体倾斜无关。指令里只用角速度的 Z 分量，即偏航角速度。
- ROS 坐标换算由 UBotROS 完成，这里不做。

## 导航无人机的简化飞行模型

`uBot Simple Flight` 不计算力和姿态动力学：速度按一阶响应逼近指令，有加速度和加加速度（jerk）上限，姿态由加速度推出。水平和垂直分开跟踪：

```
目标加速度 = 限幅((目标速度 − 当前速度) / 时间常数, 加速度上限)
加速度    += 限幅(目标加速度 − 加速度, JerkLimit × 步长)
速度      += 加速度 × 步长
```

偏航角速度同样是一阶响应加角加速度上限。物理步频 500 Hz。

| 属性 | 默认 | 说明 |
| --- | --- | --- |
| `HorizontalTimeConstantSeconds` | 0.3 s | 水平速度响应的时间常数 |
| `VerticalTimeConstantSeconds` | 0.25 s | 垂直速度响应的时间常数 |
| `MaxHorizontalAccelerationMetersPerSecondSquared` | 4 m/s² | 水平加速度上限 |
| `MaxVerticalAccelerationMetersPerSecondSquared` | 3 m/s² | 垂直加速度上限 |
| `JerkLimitMetersPerSecondCubed` | 30 m/s³ | 加速度变化率上限；0 关闭 |
| `YawTimeConstantSeconds` | 0.15 s | 偏航角速度响应的时间常数 |
| `MaxYawAccelerationRadiansPerSecondSquared` | 6 rad/s² | 偏航角加速度上限 |
| `BrakeTimeConstantSeconds` | 0.1 s | `Stop Motion` 之后刹车的时间常数 |
| `BrakeHorizontalAccelerationMetersPerSecondSquared` | 6 m/s² | 刹车时水平减速度上限 |
| `BrakeVerticalAccelerationMetersPerSecondSquared` | 4 m/s² | 刹车时垂直减速度上限 |
| `WindResponse` | 0.5 | 风变化时无人机先随风漂移的份额；0 忽略风 |
| `WindRejectionTimeConstantSeconds` | 1 s | 补偿风变化的速度 |
| `TiltMode` | `Visual` | 什么随加速度倾斜，见下 |
| `MaxTiltDegrees` | 25° | 最大倾角，限制在 [0, 60] |
| `HoverRotorSpeedRadiansPerSecond` | 794 rad/s | 悬停时显示的旋翼转速（仅显示，高级属性） |

这些属性在细节面板的 `Simple Flight` 类别（及其 `Tracking`、`Braking`、`Wind` 子类别）下。

### 刹车

`Stop Motion` 把目标速度和偏航角速度置零，并换用刹车参数，直到下一条速度指令。速度低于 0.001 m/s 且加速度低于 0.01 m/s² 时，速度和加速度直接置为 0，不会像纯一阶响应那样无限接近。`Is Braking` 在这段时间返回真。

测试验收的刹车距离：

| 起始速度 | 结果 |
| --- | --- |
| 2 m/s | 刹车距离不超过 0.6 m，1 s 内速度恰好为 0，之后位置不再变化 |
| 0.1 m/s | 0.65 s 内位移不超过 2 cm，并停住 |

### 倾斜

姿态不是动力学的结果：机体 Z 轴指向（加速度 − 重力）的方向，倾角不超过 `MaxTiltDegrees`，航向取模型的偏航角。`TiltMode` 决定哪一部分跟着倾斜：

| `TiltMode` | 倾斜的部分 | 对挂在根组件上的传感器 |
| --- | --- | --- |
| `None` | 都不倾斜，只偏航 | 始终水平 |
| `Visual`（默认） | 只有 `VisualComponent`（导航示例里指向 `VisualRoot`）倾斜，根组件和挂在它下面的东西保持水平 | 始终水平 |
| `Body` | 根组件倾斜 | 看到真实多旋翼会有的姿态；IMU 只读到沿机体 Z 轴的比力 |

测试用逐帧位姿差分的探针（算法同 IMU）验收：悬停时根组件上的比力是 +g（误差 0.02 m/s² 以内）。`Body` 模式下以 4 m/s² 的上限加速时，沿机体 X、Y 轴的读数不超过 0.15 m/s²，机头低于水平 5° 以上；`Visual` 模式下根组件保持水平，`VisualRoot` 低头超过 5°。

`Get Tilted Attitude` 在任何模式下都返回倾斜后的姿态。

### 响应的验收指标

| 项目 | 验收 |
| --- | --- |
| 2 m/s 阶跃 | 1.5 s 时至少 1.9 m/s，超调不超过 2%，高度保持不变 |
| 偏航角速度 0.5 rad/s，4 s | 偏航角 1.925 rad，误差 0.03 rad 以内（一阶滞后 0.15 s） |
| 指令 (30, 0, 20) m/s，偏航角速度 5 rad/s | 截断为 (10, 0, 3) m/s 和 π/2 rad/s；横滚、俯仰角速度被忽略；含 NaN 的指令被拒绝 |
| 6 个轴向各 2.5 s 的 1 m/s 指令 | 每个轴向至少移动 12 cm；偏航角速度 ±0.5 rad/s 的 2.5 s 内至少转 2° |

## 物理无人机的动力学模型

`uBot Multirotor Dynamics` 是用于飞控研究的刚体多旋翼。它按固定步长（`PhysicsRateHz`，默认 1000 Hz，即 1 ms）用半隐式欧拉积分推进，姿态用指数映射更新。

| 建模 | 不建模 |
| --- | --- |
| 转子推力和反扭矩，都与转速的平方成正比 | 转子的陀螺效应 |
| 电机一阶滞后（按步长精确求解） | 入流 |
| 相对空气的线性和平方阻力，旋转阻力 | 桨叶挥舞 |
| 重力 | 地面效应 |
| 刚体转动（欧拉方程，含 ω × Iω 项） | 电池 |
| 风（通过相对空气的速度） | |
| 与场景的碰撞（扫掠） | |

推力只由转速决定，与来流无关；地面只通过碰撞起作用。

```
推力     T_i = kT · ω_i²           反扭矩   Q_i = kQ · ω_i²
阻力     F = −k1 · v_air − k2 · |v_air| · v_air      （机体系逐轴，v_air = (世界系速度 − 风速) 转到机体系）
转动     I · dω/dt = Σ(r_i × T_i) + 反扭矩 − k_ang · ω − ω × (I · ω)
```

kT、kQ 是转子的 `ThrustCoefficient` 和 `TorqueCoefficient`，k1、k2、k_ang 是机架的线性、平方和旋转阻力系数，r_i 是转子位置，I 是转动惯量。

### 默认机架

`FUBotMultirotorAirframe` 的默认值是一架 1.5 kg、轴距 450 mm 的四旋翼 X。机架属性在 `Movement` 组件的 `Airframe` 上，改动在下一次 `Reset Flight` 生效。

| 属性 | 默认 |
| --- | --- |
| `MassKilograms` | 1.5 kg |
| `InertiaKilogramSquareMeters` | (0.029, 0.029, 0.055) kg·m² |
| `LinearDragCoefficients` | (0.25, 0.25, 0.30) N 每 m/s |
| `QuadraticDragCoefficients` | (0.03, 0.03, 0.05) N 每 (m/s)² |
| `AngularDragCoefficients` | (0.003, 0.003, 0.005) N·m 每 rad/s |
| `MotorCommandMapping` | `Thrust`：转速 = 最大转速 × √指令；`Speed`：转速 = 最大转速 × 指令 |
| `Rotors`（每个转子） | `ThrustCoefficient` 5.84e-6 N/(rad/s)²，`TorqueCoefficient` 9.344e-8 N·m/(rad/s)²，`MaxSpeedRadiansPerSecond` 1100 rad/s，`MotorTimeConstantSeconds` 0.02 s |

由这些值得到：最大总推力 4 × 5.84e-6 × 1100² = 28.2656 N，约为重量的 1.92 倍。按 9.8 m/s² 重力，悬停需要约 0.52 的归一化指令（`Thrust` 映射），每个转子约 793 rad/s；`Get Hover Motor Command` 按工程的重力返回悬停指令。

`BP_UBotPhysicsDrone` 只把转子位置换成模型的电机轴，其余保持默认。机架无效（质量或转动惯量不为正、没有转子、转子不产生推力）时，日志报错，并改用默认机架。`Make Quad X Airframe` 返回换了臂长和质量的默认机架：转动惯量按质量和臂长平方缩放，其余参数保持默认。

### 控制模式

最近一次被接受的指令决定当前控制模式，`Get Active Control Mode` 读取它。开始时无人机在 `Velocity` 模式下悬停。

| 模式 | 指令 | 内容和处理 |
| --- | --- | --- |
| `Velocity` | `FUBotVelocityCommand` | 航向系线速度加偏航角速度，按 `VelocityLimits` 截断；含 NaN 拒绝 |
| `Attitude` | `FUBotAttitudeCommand` | 目标姿态（机体到世界）；`bUseYawRate` 时航向按 `YawRateRadiansPerSecond` 转动（按偏航上限截断）；`bUseThrust` 时用 `NormalizedThrust`（0 到 1，最大总推力的比例），否则按 `VerticalVelocityMetersPerSecond` 飞爬升率（按爬升和下降上限截断，0 保持高度）。四元数非有限值或长度接近 0 时拒绝 |
| `BodyRate` | `FUBotBodyRateCommand` | 机体系目标角速度（不截断），推力或爬升率同上 |
| `Motor` | `FUBotMotorCommand` | 每个电机一个值：`Normalized`（0 到 1）或 `AngularVelocity`（rad/s）。值的个数必须等于电机数（`Get Motor Count`，默认机架 4 个），含 NaN 拒绝；不经过飞控 |

每种指令的接受都可以用 `bAcceptVelocityCommands`、`bAcceptAttitudeCommands`、`bAcceptBodyRateCommands`、`bAcceptMotorCommands` 单独关闭，关闭后接口返回 `false`。

### 内置飞控

`uBot Cascaded Flight Controller` 是级联结构，每种模式从自己那一级进入：

| 模式 | 经过的环节 | 推力 |
| --- | --- | --- |
| `Velocity` | 速度环 → 姿态环 → 角速度环 → 推力分配 | 速度环给出 |
| `Attitude` | 姿态环 → 角速度环 → 推力分配 | 指令推力，或垂直速度环 |
| `BodyRate` | 角速度环 → 推力分配 | 指令推力，或垂直速度环 |
| `Motor` | 不经过飞控 | 不适用 |

- 速度环：世界系 PID，输出推力向量和目标姿态；输出饱和时积分器停止累积。
- 位置保持：目标位置的水平分量，以及垂直速度指令为 0 时的高度，用增益 `PositionHoldGain` 拉回，修正速度不超过 `PositionHoldMaxSpeed`。
- 姿态环：四元数误差的 P 控制，输出目标角速度。
- 角速度环：PI 加惯量前馈。
- 推力分配：用转子几何的伪逆。请求超出能力时，先削减偏航力矩，再平移总推力以保住横滚和俯仰，最后把指令限幅到 0 到 1。
- 速度模式下航向目标按偏航角速度推进；航向误差超过 0.5 rad 时重置为当前航向。

默认增益按默认机架和 1 kHz 调整：

| 属性 | 默认 | 说明 |
| --- | --- | --- |
| `VelocityP`、`VelocityI`、`VelocityD` | (1.8, 1.8, 4.0)、(0.4, 0.4, 2.0)、(0.2, 0.2, 0.0) | 速度环增益（世界系 X、Y、Z）；D 作用在测得的加速度上 |
| `VelocityIntegratorLimit` | (3, 3, 5) m/s² | 积分器能加的最大加速度 |
| `PositionHoldGain` | 1.5 1/s | 位置保持增益 |
| `PositionHoldMaxSpeed` | 1 m/s | 位置保持的最大修正速度 |
| `MaxTiltDegrees` | 35° | 速度环要求的最大倾角 |
| `AttitudeP` | (6.5, 6.5, 2.8) 1/s | 姿态环增益（机体系 X、Y、Z） |
| `MaxBodyRateDegreesPerSecond` | (220, 220, 200) | 姿态环要求的最大角速度 |
| `RateP`、`RateI` | (20, 20, 8) 1/s、(5, 5, 2) 1/s² | 角速度环增益 |
| `RateIntegratorLimit` | (10, 10, 2) rad/s² | 角速度环积分器能加的最大角加速度 |
| `MinThrustShare`、`MaxThrustShare` | 0.08、0.95 | 速度环要求的总推力占最大总推力的范围 |

### 指令丢失

指令丢失有三个来源：发送方释放（`Release Attitude Command` 在姿态或角速度模式下，`Release Motor Command` 在电机模式下），`CommandTimeoutSeconds` 超时，或速度指令被 `Stop Motion` 释放。前两个由 `CommandLossAction` 决定怎么做：

| `CommandLossAction` | 效果 |
| --- | --- |
| `Hover`（默认） | 刹停并锁定位置，与 `Stop Motion` 相同 |
| `Land` | 原地以 `LandingSpeedMetersPerSecond`（默认 0.7 m/s）下降，被地面挡住后停在那里（电机保持悬停） |
| `CutMotors` | 所有电机指令置 0，无人机下落 |

`CommandTimeoutSeconds` 默认 0，即不超时，由发送方负责（uBot ROS 的指令组件自带超时，默认 0.5 s）。大于 0 时，最后一条被接受的指令（任何类型）距今超过这么多仿真秒就算丢失，每条指令后只触发一次。

`Stop Motion` 总是刹停并锁定位置，不看 `CommandLossAction`：速度降到 0.05 m/s 以下后，锁定当前的水平位置和高度。发零速度的速度指令与此不同，它只跟踪零速度，不锁定位置，风或扰动造成的位移不会被拉回。

测试验收：

| 项目 | 验收 |
| --- | --- |
| 悬停 10 s | 高度偏差 1 cm 以内，水平位置偏差 0.5 cm 以内，倾角 0.1° 以内，转子转速与悬停转速相差不超过 0.5% |
| 2 m/s 速度阶跃 | 2 s 时至少 1.8 m/s，峰值不超过 2.2 m/s，6 s 后误差 0.03 m/s 以内，高度偏差 0.15 m 以内，侧向速度不超过 0.05 m/s |
| 偏航角速度 0.5 rad/s，4 s | 转过 2.0 rad，误差 0.15 rad 以内 |
| 从 2 m/s 的巡航 `Stop Motion` | 刹车位移不超过 1.6 m，3 s 后速度低于 0.05 m/s，之后 7 s 内位置变化不超过 5 cm，高度偏差 0.05 m 以内 |
| 姿态跟踪 | 横滚 10°：0.5 s 内误差 1° 以内，超调不超过 3°，高度偏差 0.1 m 以内；偏航 45°：2 s 内误差 2° 以内 |
| 角速度跟踪 | 1 rad/s 的横滚角速度：0.15 s 内误差 10% 以内，0.5 s 内 2% 以内；指令置零后 0.2 s 内转动低于 0.05 rad/s |
| 电机全为 0（关闭阻力） | 以 g 自由下落，加速度误差 0.005 m/s² 以内，加速度计读数低于 0.05 m/s² |
| 指令丢失，`CommandLossAction` 为 `Land` | 下降到原位置下方的地面上（水平漂移小于 0.3 m，中心高低于 10 cm），静止（速度低于 0.05 m/s），电机转速与悬停转速相差不超过 5% |

## 裸机架无人机

`BP_UBotBareAirframeDrone` 与 `BP_UBotPhysicsDrone` 用同一个动力学，只是 `Movement` 上的 `bAcceptVelocityCommands` 和 `bAcceptAttitudeCommands` 关闭：只接受角速度和电机指令。速度和姿态指令返回 `false`，`Stop Motion` 仍然有效。`ManualInput` 的 `FlightMode` 在这个示例里是 `Rate`。

第一条指令之前，以及指令丢失之后，它用内置飞控悬停。控制器有两种接法：

- 从外部程序发角速度或电机指令（例如通过 uBot ROS 指令组件）。没有自定义飞控接管时，角速度指令经过内置的角速度环和垂直速度环；电机指令直接驱动转子。
- 写一个 `uBot Flight Controller` 子类，放到 `Movement` 的 `FlightController` 上，见下一节。

## 编写自己的飞控

`uBot Flight Controller`（`UUBotFlightController`，抽象，可蓝图化）在每个物理步（或每 `ControllerRateDivider` 个物理步）被调用一次，输入无人机的真值状态和当前模式的设定值，输出归一化的电机指令：

| 函数 | 说明 |
| --- | --- |
| `Compute Motor Commands(State, Setpoint, DeltaSeconds, OutMotorCommands)` | 返回每个转子一个 0 到 1 的指令，按电机序号。`DeltaSeconds` 是到下一次调用的时间 |
| `Reset Controller(State, Airframe)` | 清空记忆（积分器、设定值）并取得机架 |
| `Supports Control Mode(Mode)` | 这个控制器是否飞 `Mode` 的设定值；返回 `false` 的模式由内置飞控代飞 |

`Reset Controller` 在首次使用、更换控制器、控制模式切换、`Reset Flight` 之后，在下一次计算前被调用。电机模式不经过控制器。

`FUBotFlightSetpoint` 里按模式生效的字段：

| 模式 | 字段 |
| --- | --- |
| `Velocity` | `VelocityMetersPerSecond`（世界系）、`YawRateRadiansPerSecond`、`bHoldPosition`、`HoldPositionMeters` |
| `Attitude` | `Orientation`、`bUseYawRate`、`YawRateRadiansPerSecond`、`bUseThrust`、`NormalizedThrust`、`VerticalVelocityMetersPerSecond` |
| `BodyRate` | `BodyRateRadiansPerSecond`、`bUseThrust`、`NormalizedThrust`、`VerticalVelocityMetersPerSecond` |

控制器返回的指令个数不等于电机数时，电机停转，日志报一次错。非有限值按 0 处理，其余限幅到 0 到 1。

在 C++ 里重写 `_Implementation` 函数。下面是一个只飞角速度指令的比例控制器（不含高度环；推力用指令值，没给时取悬停推力）：

```cpp
// MyRateController.h
#pragma once

#include "CoreMinimal.h"
#include "UBotFlightController.h"
#include "UBotMultirotorDynamics.h"
#include "MyRateController.generated.h"

UCLASS(meta = (DisplayName = "My Rate Controller"))
class UMyRateController : public UUBotFlightController
{
    GENERATED_BODY()

public:
    /** 角速度环 P 增益（机体 X、Y、Z 轴），1/s。 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rate")
    FVector RateP = FVector(20.0, 20.0, 8.0);

protected:
    virtual void ResetController_Implementation(const FUBotMultirotorState& State, const FUBotMultirotorAirframe& InAirframe) override;
    virtual void ComputeMotorCommands_Implementation(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint,
        double DeltaSeconds, TArray<double>& OutMotorCommands) override;
    virtual bool SupportsControlMode_Implementation(EUBotFlightControlMode Mode) const override;

private:
    FUBotMultirotorAirframe Airframe;
    UBot::Multirotor::FThrustAllocator Allocator;
};

// MyRateController.cpp
#include "MyRateController.h"

#include "Engine/World.h"

void UMyRateController::ResetController_Implementation(const FUBotMultirotorState& State, const FUBotMultirotorAirframe& InAirframe)
{
    Airframe = InAirframe;
    Allocator.Build(Airframe);
}

void UMyRateController::ComputeMotorCommands_Implementation(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint,
    double DeltaSeconds, TArray<double>& OutMotorCommands)
{
    if (!Allocator.IsBuilt())
    {
        OutMotorCommands.Init(0.0, Airframe.Rotors.Num());
        return;
    }

    // 角速度误差 -> 角加速度 -> 力矩（含 w x (I w) 项）
    const FVector& Rates = State.AngularVelocityRadiansPerSecond;
    const FVector& Inertia = Airframe.InertiaKilogramSquareMeters;
    const FVector Torque = Inertia * (RateP * (Setpoint.BodyRateRadiansPerSecond - Rates)) + FVector::CrossProduct(Rates, Inertia * Rates);

    const double HoverThrust = Airframe.MassKilograms * FMath::Abs(GetWorld()->GetGravityZ()) / 100.0;
    const double Thrust = Setpoint.bUseThrust ? Setpoint.NormalizedThrust * Allocator.GetMaxTotalThrust() : HoverThrust;

    Allocator.Allocate(Thrust, Torque, OutMotorCommands);
}

bool UMyRateController::SupportsControlMode_Implementation(EUBotFlightControlMode Mode) const
{
    return Mode == EUBotFlightControlMode::BodyRate;
}
```

在物理无人机的 `Movement` 组件里，从 `FlightController` 的下拉列表选这个类；C++ 里用 `Dynamics->SetFlightController(NewObject<UMyRateController>(Dynamics))`，蓝图里用 `Set Flight Controller`。这个例子只接管 `BodyRate`，其余模式由内置飞控代飞。让 `Supports Control Mode` 对所有模式返回 `true`，它就接管悬停和速度模式，此时 `Setpoint.Mode` 是 `Velocity`，速度设定值要自己处理。

蓝图子类也能用：重写 `Compute Motor Commands`，把结果写进输出数组 `Out Motor Commands`。蓝图控制器在每个物理步（默认 1 kHz）经过蓝图虚拟机运行，开销大，应调大 `ControllerRateDivider`。

`ControllerRateDivider`（默认 1）让控制器每隔这么多个物理步运行一次，其间保持上一次的指令；此时 `DeltaSeconds` 是这么多个步长（除数 4 时是 0.004 s）。内置飞控的默认增益按 1 kHz 调整。

## 发送指令

### 从 C++ 和蓝图

指令都经过 UBotCore 的接口发送，接收方是无人机的 `Movement` 组件。蓝图里用 `Find Multirotor Movement`（`uBot Multirotor Library`）或 `Get Component by Class` 取得它：

| 接口 | 指令 | 导航 | 物理 | 裸机架 |
| --- | --- | --- | --- | --- |
| `IUBotVelocityCommandable` | `Apply Velocity Command`、`Stop Motion` | 是 | 是 | 速度指令被拒绝，`Stop Motion` 有效 |
| `IUBotAttitudeCommandable` | `Apply Attitude Command`、`Apply Body Rate Command`、`Release Attitude Command` | 没有这个接口 | 是 | 只接角速度 |
| `IUBotMotorCommandable` | `Apply Motor Command`、`Get Motor Count`、`Release Motor Command` | 没有这个接口 | 是 | 是 |

指令一直有效，直到被新指令替换或被释放；物理无人机另有 `CommandTimeoutSeconds`。`UBot::Actuation::FindVelocityCommandable`、`FindAttitudeCommandable`、`FindMotorCommandable` 在 Actor 上找到实现接口的对象，对无人机返回的是 `Movement` 组件。

```cpp
#include "UBotActuatorInterface.h"

// Drone 是无人机的 AActor*

// 速度：航向系 X 前、Y 右、Z 上（m/s）；角速度只用 Z（rad/s，+ 向右偏航）
if (UObject* Target = UBot::Actuation::FindVelocityCommandable(Drone))
{
    FUBotVelocityCommand Command;
    Command.LinearVelocityMetersPerSecond = FVector(2.0, 0.0, 0.0);
    Command.AngularVelocityRadiansPerSecond = FVector(0.0, 0.0, 0.5);
    Command.Source = TEXT("MyCode");
    if (!IUBotVelocityCommandable::Execute_ApplyVelocityCommand(Target, Command))
    {
        // 被拒绝：这种指令被关闭、含 NaN
    }
}

if (UObject* Target = UBot::Actuation::FindAttitudeCommandable(Drone))
{
    // 姿态：偏航 45°，不给推力，高度由无人机保持
    FUBotAttitudeCommand Attitude;
    Attitude.Orientation = FQuat(FVector::UpVector, FMath::DegreesToRadians(45.0));
    IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Target, Attitude);

    // 角速度：机体系 rad/s，推力为最大总推力的比例
    FUBotBodyRateCommand Rates;
    Rates.BodyRateRadiansPerSecond = FVector(1.0, 0.0, 0.0);
    Rates.bUseThrust = true;
    Rates.NormalizedThrust = 0.55;
    IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Target, Rates);

    // 不再发送时释放，无人机按 CommandLossAction 处理
    IUBotAttitudeCommandable::Execute_ReleaseAttitudeCommand(Target);
}

if (UObject* Target = UBot::Actuation::FindMotorCommandable(Drone))
{
    // 电机：每个电机一个值，个数必须等于 Get Motor Count
    FUBotMotorCommand Motors;
    Motors.Type = EUBotMotorCommandType::Normalized;
    Motors.Values.Init(0.55, IUBotMotorCommandable::Execute_GetMotorCount(Target));
    IUBotMotorCommandable::Execute_ApplyMotorCommand(Target, Motors);
}
```

蓝图里对 `Find Multirotor Movement` 的返回值（或 `Movement` 组件）调用同名的接口函数（类别 `uBot|Actuation`）。

### 从 ROS 2 发送指令

UBotROS 的三个指令组件把 ROS 2 消息换算后交给上面的接口，不含控制逻辑。把它们加到（复制出来的）无人机蓝图上即可，它们用 `UBot::Actuation::Find*` 找到 `Movement` 组件，命名空间取自无人机上的 `uBot Identity`。

| 组件 | 默认话题 | 消息 | 导航 | 物理 | 裸机架 |
| --- | --- | --- | --- | --- | --- |
| `uBot ROS Velocity Command` | `/<namespace>/cmd_vel` | `geometry_msgs/msg/Twist` | 是 | 是 | 被拒绝 |
| `uBot ROS Attitude Command` | `/<namespace>/cmd_attitude` | `mavros_msgs/msg/AttitudeTarget` | 没有目标 | 姿态和角速度 | 只接角速度 |
| `uBot ROS Motor Command` | `/<namespace>/cmd_motors` | `actuator_msgs/msg/Actuators` | 没有目标 | 是 | 是 |

超时由 UBotROS 的组件处理（`CommandTimeoutSeconds`，默认 0.5 s）：速度指令超时后无人机收到一次 `Stop Motion`，姿态和电机指令超时后收到 `Release Attitude Command` 或 `Release Motor Command`，由物理无人机的 `CommandLossAction` 决定（默认悬停）。

```bash
# 以 1 m/s 前进，向左偏航 0.2 rad/s（ROS 的 angular.z 向左为正）
ros2 topic pub -r 10 /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.2}}"
# 角速度：绕 z 轴 1 rad/s（向左偏航），不给推力（无人机自己保持高度）
ros2 topic pub -r 50 /drone/cmd_attitude mavros_msgs/msg/AttitudeTarget "{type_mask: 192, body_rate: {z: 1.0}}"
```

话题规则、QoS、坐标和单位换算、`type_mask` 的含义见 UBotROS 的 README 里“速度指令”和“飞行指令”两节。

## 手动飞行

`uBot Multirotor Manual Input` 是示例蓝图里的 `ManualInput` 组件，自己拼的无人机加上它即可。它只在本地玩家控制（Possess）这架无人机时工作，AI 控制器或远程控制器不触发它；`bTakeControlWhenPossessed` 关闭则不接管。

控制期间它每帧发一条指令；控制结束时，`Velocity` 模式发 `Stop Motion`，其余模式发 `Release Attitude Command`，无人机刹停并悬停（物理无人机在后一种情况下按 `CommandLossAction` 处理）。同时有其他发送方（例如 ROS）时，最后到达的指令生效。

| 轴 | 键盘 | 手柄（模式 2） |
| --- | --- | --- |
| 前 / 后 | W / S | 右摇杆上 / 下 |
| 右 / 左 | D / A | 右摇杆右 / 左 |
| 上 / 下 | Space / Left Ctrl | 左摇杆上 / 下 |
| 偏航右 / 左 | E / Q | 左摇杆右 / 左 |

`FlightMode` 决定摇杆的含义，要和无人机能接受的指令匹配：

| `FlightMode` | 摇杆含义 | 发出的指令 | 示例里用它的无人机 |
| --- | --- | --- | --- |
| `Velocity` | 前后左右：航向系水平速度，满偏 `MaxHorizontalSpeedMetersPerSecond`（5 m/s）；上下：升降速度，满偏 `MaxVerticalSpeedMetersPerSecond`（2 m/s）；偏航：满偏 `MaxYawRateDegreesPerSecond`（90°/s） | 速度指令 | 导航、物理 |
| `Angle` | 前后左右：倾斜，满偏 `MaxTiltDegrees`（30°）；偏航轴转动航向；上下：爬升率 | 姿态指令，不带推力，高度由无人机保持 | 无（物理无人机可选） |
| `Rate` | 前后左右：横滚和俯仰角速度，满偏 `MaxBodyRateDegreesPerSecond`（220°/s）；偏航轴：偏航角速度；上下：爬升率 | 角速度指令 | 裸机架 |

`Angle` 模式下，右摇杆向右是向右倾，向前是机头下俯。`Rate` 模式下符号相同：右摇杆向右是向右横滚，向前是机头下俯，偏航轴向右是向右偏航。导航无人机没有姿态接口，选 `Angle` 或 `Rate` 时日志警告一次。

摇杆死区 `DeadZone` 默认 0.15，死区边缘重映射为 0，满偏为 1。水平两个轴合起来的长度限制为 1，所以斜向不会更快。

自定义键位：设置 `CustomMappingContext`、`CustomMoveAction`（Axis3D：前、右、上）和 `CustomYawAction`（Axis1D，+ 向右偏航），映射上下文要映射这两个动作；`MappingPriority`（默认 1）是映射上下文的优先级。留空则用内置键位，它们在开始控制时按 `UBot::ManualFlight::GetDefaultBindings()` 在运行时创建，不需要资产。

示例蓝图的 `Camera` 是挂在弹簧臂 `CameraBoom` 末端的普通相机：玩家控制无人机时，视图使用它。弹簧臂长 120 cm，俯仰 −15°，只继承机体的偏航（不继承俯仰和横滚，所以物理无人机倾斜、翻滚时视角不会跟着晃），相机延迟速度 5，相机视场 80°。

测试验收：W 满偏给出 5 m/s 前进，W + D 的速度模长仍是 5 m/s；Left Ctrl 给出 −2 m/s，E 给出 π/2 rad/s 的偏航角速度。`Angle` 模式右摇杆满偏横滚 30°，偏航轴满偏 1 s 把航向转 90°。

## 风

### `uBot Wind` 与脚本风

`uBot Wind`（`AUBotWind`）放进关卡后，它的风在整个关卡里均匀吹（与位置无关），多个 Actor 的风叠加。编辑器里的箭头显示恒定风的方向，游戏中隐藏。

| 属性 | 默认 | 说明 |
| --- | --- | --- |
| `bEnabled` | 开 | 这个 Actor 的风是否吹 |
| `Settings.ConstantVelocityMetersPerSecond` | (0, 0, 0) | 恒定风速，世界系，即空气移动的方向，m/s |
| `Settings.GustStandardDeviationMetersPerSecond` | (0, 0, 0) | 每个世界轴阵风的标准差，m/s；0 关闭阵风 |
| `Settings.GustCorrelationTimeSeconds` | 2 s | 阵风的典型持续时间，越长越平缓，不小于 0.05 s |
| `Settings.Seed` | 0 | 阵风的种子 |

`Set Wind Settings` 和 `Set Wind Enabled` 在运行时改变风。风子系统 `uBot Wind Subsystem`（用 `Get World Subsystem` 取得）有两个节点：

- `Set Scripted Wind(Settings, bEnabled)`：设置（`bEnabled` 关闭时清除）一份与关卡里 `uBot Wind` 叠加的风。
- `Get Wind Velocity At(Location Cm)`：当前仿真时间的风速，m/s。

风子系统只在 Game 和 PIE 世界里存在，`uBot Wind` 在 `BeginPlay` 时登记。

### 阵风

阵风是每个世界轴 8 个正弦分量的和：频率从 1/(20T) 到 2/T Hz 在对数轴上均匀分布（T 是 `GustCorrelationTimeSeconds`），幅值按一阶（类 Dryden）谱的形状分配并缩放到所设的标准差，相位由种子生成。结果只取决于设置和仿真时间，同样的设置在同样的时刻给出同样的风。

测试验收：同一设置得到逐位相同的阵风，换种子得到不同的阵风；600 s 内每个轴的均方根在所设标准差的 15% 以内，均值在标准差的 10% 以内；没有设标准差则没有阵风。

### 无人机的响应

每个物理步，飞行模型在无人机位置和该步的物理时间取一次风速，`bEnableWind` 关闭则取 0。

| 无人机 | 响应 | 测试验收 |
| --- | --- | --- |
| 物理 / 裸机架 | 风通过相对空气的速度进入阻力公式。悬停时内置飞控的位置保持把无人机拉回，机体向来风方向倾斜 | 悬停时遇到 5 m/s 恒定风：被推开不超过 1 m（且超过 0.1 m），10 s 后速度低于 0.05 m/s，倾角 7.7°（误差 1° 以内），推力指向来风方向；关闭 `bEnableWind` 后倾角回到 0.5° 以内 |
| 导航 | 恒定风被补偿，不产生位移。风速变化时无人机先随风漂移，再按 `WindRejectionTimeConstantSeconds` 补偿；漂移速度是风速变化的 `WindResponse` 倍 | 3 m/s 的风速阶跃：最大漂移超过 1 m 且不超过 1.52 m（上限 0.5 × 3 m/s × 1 s = 1.5 m），8 s 后速度低于 0.01 m/s |

## 碰撞

飞行模型用根组件（`CollisionBox`）的碰撞形状、对象通道和响应沿运动方向扫掠，忽略无人机自己，遇到几何体后沿接触面滑动。旋转不参与扫掠；起点就嵌在几何体里时被推出。

### 地面也是表面

地面、墙和其他物体都只是表面，碰到地面和碰到一块普通的板子没有区别：

- 无人机在表面前停住（朝向表面的那部分运动被去掉），沿表面滑动（`ContactFriction` 决定接触时滑动速度的损失），接触期间继续执行每一条指令。在地面上按住 W 就贴着地面前进，按 Space 立刻上升。
- 没有落地状态，没有起飞条件，没有坠毁，也没有冻结的状态；旋翼在地面上不会停转。指令不会因为碰撞被拒绝。
- 撞得再重（例如以 6 m/s 撞墙）也只触发 `OnImpact`；无人机继续飞行，继续接受指令。

### 开始和重置

开始时和 `Reset Flight` 之后，无人机都在当前位置悬停：放在地面上也一样，不会下沉贴地，就停在放置的位置，旋翼以悬停转速旋转。`Reset Flight` 停止所有运动、清除当前指令，并在当前位姿重新悬停。

**传送**：`Teleport Multirotor(NewLocation, NewRotation, bStopMotion = true)`，位置单位厘米，不做扫掠。`bStopMotion` 为真时速度清零并在新位置悬停，否则保持速度继续飞。

**外部移动**：用 `SetActorLocation` 等从外部移动 Actor（编辑器、重置脚本），飞行模型在下一帧接管新位姿，速度保持。

### 物理无人机不往表面里推

物理无人机的速度指令（姿态或角速度模式下是垂直速度指令）不会往最近 50 ms 内接触过的表面里推：指令里朝向该表面的分量被丢掉，控制器不会因为推不动而积分饱和。所以在地面上按住 Left Ctrl（下降），电机保持悬停推力，紧接着给上升指令，无人机立刻升起。

### 事件

飞行模型的事件在每帧的 tick 末尾按发生顺序触发，处理函数里可以再发指令。

| 事件 | 参数 | 触发 |
| --- | --- | --- |
| `OnImpact` | `Hit`、`ImpactSpeedMetersPerSecond` | 开始接触，且沿表面法线的撞击速度不低于 `ImpactEventMinSpeedMetersPerSecond`（0.3 m/s） |

测试验收：

| 项目 | 验收 |
| --- | --- |
| 导航无人机以 1 m/s 撞墙 | 停在墙前 1 cm 以内，朝墙的速度不超过 0.06 m/s，触发一次 `OnImpact`，撞击速度与接近速度相差不超过 0.05 m/s |
| 以 2 m/s 撞向 45° 的墙 | 沿墙的滑动速度至少 0.7 m/s，向 −Y 移动超过 1 m |
| `Collision.FloorIsASurface`（导航） | 从 1 m 高处以 1 m/s 下降：停在地面上，中心高 9 cm（测试用 Pawn 的碰撞盒），触发一次 `OnImpact`，旋翼继续旋转。然后以 1 m/s 前进、同时仍发 −0.5 m/s 的下降指令：2 s 内沿地面移动至少 1 m，仍在地面上。再发上升指令：0.5 s 后升高至少 15 cm，2.5 s 后中心离地至少 1 m |
| `Collision.StartOnFloor`（导航） | 放在地面上，1 s 内停在放置的位置；发 1 m/s 的前进指令，1 s 内移动至少 0.5 m。`Reset Flight` 让运动中的无人机停下，并清除它的指令 |
| `Collision.PhysicsFloor`（物理） | 以 −1 m/s 下压地面 5 s：静止在中心高 9 cm（误差 0.5 cm），速度低于 0.05 m/s，倾角小于 1°，电机接近悬停（高于 600 rad/s）。然后以 1 m/s 前进、同时仍在下压：3 s 内至少前进 1.5 m，速度约 1 m/s（误差 0.15 m/s），仍在地面上。再发上升指令：0.5 s 后中心离地至少 0.25 m，2.5 s 后至少 1 m |
| `Collision.HardImpact`（物理） | 以 6 m/s 撞墙：触发 `OnImpact`，撞击速度至少 3 m/s，停在墙前；仍然可以飞行并接受指令；以 −1 m/s 飞回，3 s 内至少飞回 1 m，并留在空中 |

示例蓝图放在地面上时，根组件中心离地 6.7 cm（碰撞盒的半高）；上表的 9 cm 是测试用 Pawn 的碰撞盒。

## 确定性与时间

- **仿真时间，固定步长。** 物理时间 = 起点 + 步序号 × 步长，步序号是整数，不累加帧间隔。没有墙上时间，没有随机数（阵风由种子决定）。
- **暂停。** 暂停时不跑物理步，恢复后不补跑：500 Hz 的导航无人机在恢复后第一个 1/60 s 的帧里只跑 8 或 9 步。
- **单步。** `UUBotClockSubsystem::StepSimulation` 恰好推进指定的帧数。
- **帧间隔相同，结果逐位相同。** 测试里同一场景（含碰撞）跑两次，位置、速度、姿态完全一致；恒定指令下 60 Hz 和 24 Hz 的帧间隔都跑同样的步数（导航 6 s 共 3000 步，物理 3 s 共 3000 步），最终状态完全一致。指令在帧边界送达，测试覆盖的是恒定指令和固定的指令时间表。
- **限制。** 一帧需要的步数超过 `MaxStepsPerFrame` 时，多出的时间被跳过，不会追赶，结果就不再对应连续的步序。
- **呈现的位姿。** 飞行模型在 `TG_PrePhysics` 更新，物理最多落后仿真时间一个步长，Actor 的位姿外推到当前仿真时间；`Get Multirotor State` 是最近一个物理步的真值，两者最多相差一个步长。

uBotSensor 的传感器在 `TG_PostPhysics` 采样并对位姿做差分，所以外推让帧内采样到的速度和加速度是平滑的。测试用逐帧位姿差分的探针复现 IMU 的算法：导航无人机以 3 m/s 匀速飞行时，加速度读数不超过 0.05 m/s²，速度波动不超过 0.002 m/s；物理无人机悬停 10 s，读数与 +g 相差不超过 0.05 m/s²。

## 传感器

uBotSensor 的传感器组件可以直接加到复制出来的示例蓝图（或自己拼的无人机）上，插件之间没有代码依赖。

- 挂在 `CollisionBox`（根）下的传感器随根组件运动；导航无人机的 `Visual` 模式下它保持水平。
- 挂在 `VisualRoot` 下的传感器随机体模型一起倾斜；这只在导航无人机的 `Visual` 模式下与根组件不同。
- 物理无人机的根组件就是机体，传感器自然随机体倾斜。
- ROS 2 发布用 UBotROS 的 `uBot ROS Sensor Bridge`，加在同一个无人机蓝图上。

## 状态与蓝图接口

`Get Multirotor State` 返回最近一个物理步的真值（`FUBotMultirotorState`）：

| 字段 | 说明 |
| --- | --- |
| `PositionMeters` | 根的位置，m，世界轴 |
| `Orientation` | 机体到世界的姿态 |
| `LinearVelocityMetersPerSecond` | 世界系线速度，m/s |
| `AngularVelocityRadiansPerSecond` | 机体系角速度，rad/s，UE 旋向 |
| `LinearAccelerationMetersPerSecondSquared` | 该步的世界系线加速度 |
| `SpecificForceMetersPerSecondSquared` | 理想加速度计的读数（加速度减重力，机体系）；悬停时 Z 约 +9.81 |
| `RotorSpeedsRadiansPerSecond` | 按电机序号的转速，导航无人机为空 |
| `SimTimeSeconds` | 这个物理步的 uBot 仿真时间 |

蓝图函数的类别是 `uBot|Aerial|Multirotor`、`uBot|Aerial|Wind` 和 `uBot|Aerial|Input`：

| 对象 | 节点 |
| --- | --- |
| 飞行模型（`uBot Simple Flight`、`uBot Multirotor Dynamics`） | `Get Multirotor State`、`Get Last Velocity Command`、`Get Physics Step Count`、`Get Rotor Layout`、`Reset Flight`、`Teleport Multirotor` |
| `uBot Simple Flight` | `Is Braking`、`Get Tilted Attitude` |
| `uBot Multirotor Dynamics` | `Get Active Control Mode`、`Get Flight Setpoint`、`Set Flight Controller`、`Get Last Motor Commands`、`Get Hover Motor Command` |
| `uBot Rotor` | `Set Rotor Speed`、`Get Rotor Speed` |
| `uBot Wind` | `Set Wind Settings`、`Set Wind Enabled` |
| `uBot Wind Subsystem` | `Set Scripted Wind`、`Get Wind Velocity At` |
| `uBot Multirotor Manual Input` | `Is Controlling`、`Get Manual Flight Axes` |
| `uBot Flight Controller` | `Reset Controller`、`Compute Motor Commands`、`Supports Control Mode` |
| `uBot Multirotor Library` | `Make Quad X Airframe`、`Find Multirotor Movement` |

## 动力学后端（实验性）

物理无人机的积分由一个动力学后端完成。后端通过扩展点 `UBotAerial.DynamicsBackend` 注册（`IUBotMultirotorBackendFactory`），`DynamicsBackendName` 按名字选择。

这个扩展点是实验性的，接口在小版本之间可能变化。随插件提供的后端只有 `BuiltIn`（上面描述的积分器），也是默认值；名字找不到时回退到 `BuiltIn` 并记一条 Warning。

## 常见问题

| 现象 | 原因和处理 |
| --- | --- |
| Content Browser 里看不到示例蓝图和地图 | 插件的 Content 默认不显示：打开 `Settings > Show Plugin Content` |
| 更新插件后，对示例的修改不见了 | 直接改的是插件里的原资产。把示例复制到自己的工程再改，见“示例内容” |
| 按键没有反应 | 玩家要控制（Possess）这架无人机，`bTakeControlWhenPossessed` 要开；默认玩家输入类要是 Enhanced Input；`FlightMode` 要是无人机能接受的（导航无人机只能用 `Velocity`），否则日志警告一次 |
| 指令返回 `false` | 这种指令被 `bAccept...Commands` 关闭（裸机架拒绝速度和姿态）；指令含 NaN；电机指令的个数不等于电机数；姿态四元数长度接近 0 |
| 物理无人机在风里越飘越远 | 零速度指令不锁定位置。要刹停并锁定位置，用 `Stop Motion` |
| 导航无人机加速时挂在根上的传感器不倾斜 | 默认 `TiltMode` 是 `Visual`，根组件始终水平；要让根倾斜用 `Body` |
| 日志提示一帧需要的物理步数超过上限 | 帧太长。调大 `MaxStepsPerFrame` 或降低 `PhysicsRateHz` |
| 运行中改了 `Airframe` 没有效果 | 改动在下一次 `Reset Flight` 生效 |
| 日志提示找不到动力学后端 | `DynamicsBackendName` 拼写错误，或没有注册这个后端；已回退到 `BuiltIn` |
| 日志提示机架不能飞 | 质量或转动惯量不为正，没有转子，或转子不产生推力；已改用默认机架 |

## 测试

测试名都以 `UBotAerial.` 开头，按类别分组：

- `SimpleFlight.*`、`Math.SimpleFlightModel`：阶跃响应、偏航角速度、指令截断、有界刹车、逐轴的最小位移、比力、逐帧呈现无抖动，验收指标见“导航无人机的简化飞行模型”。
- `Dynamics.*`、`Controller.*`、`Math.ThrustAllocation`：10 s 悬停、速度阶跃、偏航角速度、有界刹车、电机全为 0 的自由落体、指令丢失（`Hover`、`Land`、`CutMotors`、超时）、各无人机接受的指令类型、姿态和角速度跟踪、自定义飞控（每步一次、`ControllerRateDivider`、个数错误）。推力分配：最大总推力，可行请求的往返误差，横滚、俯仰、偏航的符号，偏航饱和时先削减偏航。
- `Collision.*`：撞墙停住、45° 墙面滑动、地面只是一个表面（导航无人机下降到地面、贴着地面前进、随即爬升；物理无人机在地面上下压、前进、爬升）、放在地面上的起步和 `Reset Flight`、高速撞墙后继续飞行并接受指令、外部移动与 `Teleport Multirotor`。
- `Wind.*`：阵风的确定性和统计量，`uBot Wind` 与脚本风叠加，物理无人机的稳态风响应，导航无人机的风速阶跃。
- `Input.*`：轴到指令的映射（`Velocity`、`Angle`、`Rate` 的符号和幅值，死区），默认键位不重复、每个轴有正负键和摇杆，映射上下文与键位一致。
- `Base.PauseNoMotion`、`Determinism.*`：暂停期间没有物理步、恢复后不补跑，两次运行逐位相同，不同帧率下恒定指令的结果相同。
- `Math.*`：固定步长时钟（每秒恰好 500 步，一帧跑 8 或 9 步，超限的步数被丢弃并计数）、姿态与推力方向、指令截断、四旋翼 X 布局。
- `Examples.*`：示例蓝图的组件构成（旋翼在机架的位置上，`Movement` 的类型和设置），物理示例的悬停和速度阶跃，示例资产的内容依赖（只引用 `/UBotAerial`、`/Engine` 和 `/Script`）。
- `Module.Loads`、`Backend.Selection`：包注册表里的层、能力键和依赖，后端的选择和回退。

测试在手动 tick 的临时 Game 世界里运行，不需要 GPU，需要在启用了 UBotCore 和 UBotAerial 的工程里用带编辑器上下文的 `UnrealEditor` 运行。编辑器内：`Tools` 菜单打开 Session Frontend 的 Automation 标签页，过滤 `UBotAerial`。命令行：

```powershell
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Project>\<Project>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotAerial; Quit' '-TestExit=Automation Test Queue Empty'
```

把 `UBotAerial` 换成更具体的前缀（如 `UBotAerial.SimpleFlight`）可以只跑一类。UBotCore 自己的测试用 `UBotCore` 前缀。

## 版本与许可

当前版本 **0.1.0**（Beta，`IsBetaVersion`），尚未发布；仓库 <https://github.com/AstreoX/uBotAerial>。

Copyright 2026 AstreoX。本项目的代码以 [Apache License 2.0](LICENSE) 发布，可以自由使用、修改和再分发（包括商业用途），再分发时需保留 `LICENSE` 与 `NOTICE` 文件，并注明你修改过的文件。无人机模型不在其内，见下一节。

## 模型来源与授权

`/UBotAerial/Models/` 里的无人机模型来自 Fab 上 HARTWEL JUNIOR KISAKA 的 [Animated Drone](https://www.fab.com/listings/c1f5f392-330a-418c-b0b0-e3c89639347f)，授权为 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。我们做了这些改动：

- 转成静态网格：机身，加上居中在电机轴上的顺时针和逆时针桨叶；
- 整体放大 2 倍，对应 450 mm 的机架；机头转向 +X；
- 材质合并为一个，贴图缩到 1024，并转成金属度-粗糙度流程。

Apache-2.0 只覆盖插件的代码，不覆盖这套模型。署名写在仓库的 `NOTICE` 文件里；再分发包含这些模型的内容时，需按 CC BY 4.0 保留署名并注明修改。
