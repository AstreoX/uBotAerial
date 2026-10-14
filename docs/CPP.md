# C++ 例子

> 本文取自改版前的 README，正文原样保留。正文里“见……”这类引用指向当时 README 的章节，现在对应[文档站](https://astreox.github.io/uBotDocs/aerial/)上的页面。

## 编写自己的飞控

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
