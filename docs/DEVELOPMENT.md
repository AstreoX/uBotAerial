
# 开发与测试

> 本文取自原 README，正文未改动。正文中“见……”的引用对应[文档站](https://astreox.github.io/uBotDocs/aerial/)上的页面。

## 构建

将仓库放到 UE 5.5 工程的 `Plugins/uBot/uBotAerial`，与 uBotCore 并列；工程需要能编译 C++。首次打开工程时，编辑器会提示编译模块，见[安装](https://astreox.github.io/uBotDocs/start/install/)。

## 在 uBot 系列中的位置

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

## 插件内容

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

### 示例蓝图的默认值

蓝图本身的默认值：`Auto Possess AI` 是 `Disabled`，`Start with Tick Enabled` 关闭（行为都在组件里）。

## 测试

测试名都以 `UBotAerial.` 开头，按类别分组：

- `SimpleFlight.*`、`Math.SimpleFlightModel`：阶跃响应、偏航角速度、指令截断、有界刹车、逐轴的最小位移、比力、逐帧呈现无抖动，验收指标见“导航无人机的简化飞行模型”。
- `Dynamics.*`、`Controller.*`、`Math.ThrustAllocation`：10 s 悬停、速度阶跃、偏航角速度、有界刹车、电机全为 0 的自由落体、指令丢失（`Hover`、`Land`、`CutMotors`、超时）、各无人机接受的指令类型、姿态和角速度跟踪、自定义飞控（每步一次、`ControllerRateDivider`、个数错误）。推力分配：最大总推力，可行请求的往返误差，横滚、俯仰、偏航的符号，偏航饱和时先削减偏航。
- `Collision.*`：撞墙停住、45° 墙面滑动、地面只是一个表面（导航无人机下降到地面、贴着地面前进、随即爬升；物理无人机在地面上下压、前进、爬升）、放在地面上的起步和 `Reset Flight`、高速撞墙后继续飞行并接受指令、外部移动与 `Teleport Multirotor`。
- `Wind.*`：阵风的确定性和统计量，`uBot Wind` 与脚本风叠加，物理无人机的稳态风响应，导航无人机的风速阶跃。
- `Input.*`：轴到指令的映射（`Velocity`、`Angle`、`Rate` 的符号和幅值，死区），默认键位不重复、每个轴有正负键和摇杆，映射上下文与键位一致。
- `Base.PauseNoMotion`、`Determinism.*`：暂停期间没有物理步、恢复后不补跑，两次运行逐位相同，不同帧率下恒定指令的结果相同。
- `BodyState.*`：刚体状态源只提供根组件（BeginPlay 之前就成立，BeginPlay 之后才有状态）；状态就是积分器的状态，而不是外推后的呈现位姿，角速度在世界系且与姿态的转动率一致；角加速度等于单个物理步里世界角速度的变化，`Reset Flight` 和传送后为零。
- `Math.*`：固定步长时钟（每秒恰好 500 步，一帧跑 8 或 9 步，超限的步数被丢弃并计数）、姿态与推力方向、指令截断、四旋翼 X 布局；`Math.BodyRateBetweenSmallAngles`：两个姿态之间的机体系角速度，转角小到每步不足 2e-4 rad（500 Hz 下低于 0.1 rad/s）时转轴也是对的。
- `Examples.*`：示例蓝图的组件构成（旋翼在机架的位置上，`Movement` 的类型和设置），物理示例的悬停和速度阶跃，示例资产的内容依赖（只引用 `/UBotAerial`、`/Engine` 和 `/Script`）。
- `Module.Loads`、`Backend.Selection`：包注册表里的层、能力键和依赖，后端的选择和回退。

测试在手动 tick 的临时 Game 世界里运行，不需要 GPU，需要在启用了 UBotCore 和 UBotAerial 的工程里用带编辑器上下文的 `UnrealEditor` 运行。编辑器内：`Tools` 菜单打开 Session Frontend 的 Automation 标签页，过滤 `UBotAerial`。命令行：

```powershell
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Project>\<Project>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotAerial; Quit' '-TestExit=Automation Test Queue Empty'
```

把 `UBotAerial` 换成更具体的前缀（如 `UBotAerial.SimpleFlight`）可以只跑一类。UBotCore 自己的测试用 `UBotCore` 前缀。
