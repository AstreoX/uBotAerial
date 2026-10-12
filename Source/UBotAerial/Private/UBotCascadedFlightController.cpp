#include "UBotCascadedFlightController.h"

#include "UBotMultirotorMath.h"

namespace
{
    // Above this heading error the velocity loop's yaw setpoint snaps to the current heading, rad.
    constexpr double YawResyncRadians = 0.5;
    // The vertical loop never divides by less than this tilt cosine (about 72 degrees).
    constexpr double MinTiltCosine = 0.3;

    FVector ClampIntegral(const FVector& Integral, const FVector& Gain, const FVector& Limit)
    {
        FVector Result = Integral;
        for (int32 Axis = 0; Axis < 3; ++Axis)
        {
            if (Gain[Axis] > 0.0)
            {
                const double Max = FMath::Max(Limit[Axis], 0.0) / Gain[Axis];
                Result[Axis] = FMath::Clamp(Result[Axis], -Max, Max);
            }
            else
            {
                Result[Axis] = 0.0;
            }
        }
        return Result;
    }

    // World gravity as the state implies it: acceleration minus what the accelerometer reads.
    double GravityFromState(const FUBotMultirotorState& State)
    {
        const FVector Gravity = State.LinearAccelerationMetersPerSecondSquared - State.Orientation.RotateVector(State.SpecificForceMetersPerSecondSquared);
        const double Magnitude = Gravity.Size();
        return Magnitude > 0.1 ? Magnitude : 9.80665;
    }
}

bool UUBotCascadedFlightController::SupportsControlMode_Implementation(EUBotFlightControlMode Mode) const
{
    return Mode != EUBotFlightControlMode::Motor;
}

void UUBotCascadedFlightController::ResetController_Implementation(const FUBotMultirotorState& State, const FUBotMultirotorAirframe& InAirframe)
{
    Airframe = InAirframe;
    Allocator.Build(Airframe);
    VelocityIntegral = FVector::ZeroVector;
    RateIntegral = FVector::ZeroVector;
    YawSetpoint = UBot::Multirotor::GetYaw(State.Orientation);
    bHasYawSetpoint = true;
    GravityMagnitude = GravityFromState(State);
}

void UUBotCascadedFlightController::ComputeMotorCommands_Implementation(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint,
    double DeltaSeconds, TArray<double>& OutMotorCommands)
{
    if (!Allocator.IsBuilt())
    {
        OutMotorCommands.SetNumZeroed(Airframe.Rotors.Num());
        return;
    }
    GravityMagnitude = GravityFromState(State);
    const double MaxThrust = Allocator.GetMaxTotalThrust();

    switch (Setpoint.Mode)
    {
    case EUBotFlightControlMode::Velocity:
    {
        double Thrust = 0.0;
        FQuat Attitude = FQuat::Identity;
        RunVelocityLoop(State, Setpoint, DeltaSeconds, Thrust, Attitude);
        const FVector YawFeedForward = State.Orientation.UnrotateVector(FVector(0.0, 0.0, Setpoint.YawRateRadiansPerSecond));
        RunRateLoop(State, RunAttitudeLoop(State, Attitude, YawFeedForward), Thrust, DeltaSeconds, OutMotorCommands);
        break;
    }
    case EUBotFlightControlMode::Attitude:
    {
        FQuat Target = Setpoint.Orientation.GetNormalized();
        FVector FeedForward = FVector::ZeroVector;
        if (Setpoint.bUseYawRate)
        {
            if (!bHasYawSetpoint)
            {
                YawSetpoint = UBot::Multirotor::GetYaw(State.Orientation);
                bHasYawSetpoint = true;
            }
            YawSetpoint = UBot::Multirotor::WrapAngle(YawSetpoint + Setpoint.YawRateRadiansPerSecond * DeltaSeconds);
            Target = UBot::Multirotor::AttitudeFromThrustDirectionAndYaw(Target.GetAxisZ(), YawSetpoint);
            FeedForward = State.Orientation.UnrotateVector(FVector(0.0, 0.0, Setpoint.YawRateRadiansPerSecond));
        }
        else
        {
            YawSetpoint = UBot::Multirotor::GetYaw(Target);
            bHasYawSetpoint = true;
        }
        const double Thrust = Setpoint.bUseThrust ? FMath::Clamp(Setpoint.NormalizedThrust, 0.0, 1.0) * MaxThrust
                                                  : RunVerticalLoop(State, Setpoint.VerticalVelocityMetersPerSecond, DeltaSeconds);
        RunRateLoop(State, RunAttitudeLoop(State, Target, FeedForward), Thrust, DeltaSeconds, OutMotorCommands);
        break;
    }
    case EUBotFlightControlMode::BodyRate:
    {
        // The heading is not controlled here; leaving body-rate mode starts from the heading reached.
        bHasYawSetpoint = false;
        const double Thrust = Setpoint.bUseThrust ? FMath::Clamp(Setpoint.NormalizedThrust, 0.0, 1.0) * MaxThrust
                                                  : RunVerticalLoop(State, Setpoint.VerticalVelocityMetersPerSecond, DeltaSeconds);
        RunRateLoop(State, Setpoint.BodyRateRadiansPerSecond, Thrust, DeltaSeconds, OutMotorCommands);
        break;
    }
    default:
        OutMotorCommands.SetNumZeroed(Allocator.GetRotorCount());
        break;
    }
}

void UUBotCascadedFlightController::RunVelocityLoop(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint, double DeltaSeconds,
    double& OutThrust, FQuat& OutAttitude)
{
    const double Yaw = UBot::Multirotor::GetYaw(State.Orientation);
    if (!bHasYawSetpoint)
    {
        YawSetpoint = Yaw;
        bHasYawSetpoint = true;
    }
    YawSetpoint = UBot::Multirotor::WrapAngle(YawSetpoint + Setpoint.YawRateRadiansPerSecond * DeltaSeconds);
    if (FMath::Abs(UBot::Multirotor::WrapAngle(YawSetpoint - Yaw)) > YawResyncRadians)
    {
        YawSetpoint = Yaw;
    }

    FVector Target = Setpoint.VelocityMetersPerSecond;
    if (Setpoint.bHoldPosition)
    {
        FVector Correction = (Setpoint.HoldPositionMeters - State.PositionMeters) * PositionHoldGain;
        if (Setpoint.VelocityMetersPerSecond.Z != 0.0)
        {
            Correction.Z = 0.0;
        }
        Target += UBot::Multirotor::ClampNorm(Correction, PositionHoldMaxSpeed);
    }

    const double Mass = Airframe.MassKilograms;
    const FVector Error = Target - State.LinearVelocityMetersPerSecond;
    const FVector Acceleration = VelocityP * Error + VelocityI * VelocityIntegral - VelocityD * State.LinearAccelerationMetersPerSecondSquared;
    FVector Force = (Acceleration + FVector(0.0, 0.0, GravityMagnitude)) * Mass;

    // Keep the vertical part and shorten the horizontal part when the tilt would be too large.
    bool bSaturated = false;
    Force.Z = FMath::Max(Force.Z, 0.1 * Mass * GravityMagnitude);
    const double MaxHorizontal = Force.Z * FMath::Tan(FMath::DegreesToRadians(MaxTiltDegrees));
    const double Horizontal = FVector2D(Force.X, Force.Y).Size();
    if (Horizontal > MaxHorizontal)
    {
        Force.X *= MaxHorizontal / Horizontal;
        Force.Y *= MaxHorizontal / Horizontal;
        bSaturated = true;
    }

    // Thrust along the body's current Z axis, as the body is tilted now.
    const double MaxThrust = Allocator.GetMaxTotalThrust();
    const double Low = MinThrustShare * MaxThrust;
    const double High = MaxThrustShare * MaxThrust;
    double Thrust = FVector::DotProduct(Force, State.Orientation.GetAxisZ());
    if (Thrust < Low || Thrust > High)
    {
        bSaturated = true;
        Thrust = FMath::Clamp(Thrust, Low, High);
    }
    if (!bSaturated)
    {
        // The integrator stops while the output saturates, so it does not wind up.
        VelocityIntegral = ClampIntegral(VelocityIntegral + Error * DeltaSeconds, VelocityI, VelocityIntegratorLimit);
    }

    OutThrust = Thrust;
    OutAttitude = UBot::Multirotor::AttitudeFromThrustDirectionAndYaw(Force, YawSetpoint);
}

double UUBotCascadedFlightController::RunVerticalLoop(const FUBotMultirotorState& State, double VerticalVelocity, double DeltaSeconds)
{
    const double Error = VerticalVelocity - State.LinearVelocityMetersPerSecond.Z;
    const double Acceleration = VelocityP.Z * Error + VelocityI.Z * VelocityIntegral.Z - VelocityD.Z * State.LinearAccelerationMetersPerSecondSquared.Z;
    const double Cosine = FMath::Max(State.Orientation.GetAxisZ().Z, MinTiltCosine);
    const double MaxThrust = Allocator.GetMaxTotalThrust();
    const double Wanted = Airframe.MassKilograms * (Acceleration + GravityMagnitude) / Cosine;
    const double Thrust = FMath::Clamp(Wanted, 0.0, MaxThrustShare * MaxThrust);
    if (Thrust == Wanted)
    {
        FVector Integral = VelocityIntegral;
        Integral.Z += Error * DeltaSeconds;
        VelocityIntegral = ClampIntegral(Integral, VelocityI, VelocityIntegratorLimit);
    }
    return Thrust;
}

FVector UUBotCascadedFlightController::RunAttitudeLoop(const FUBotMultirotorState& State, const FQuat& Target, const FVector& RateFeedForward) const
{
    FQuat ErrorQuat = State.Orientation.Inverse() * Target;
    if (ErrorQuat.W < 0.0)
    {
        ErrorQuat = FQuat(-ErrorQuat.X, -ErrorQuat.Y, -ErrorQuat.Z, -ErrorQuat.W);
    }
    const FVector Vector(ErrorQuat.X, ErrorQuat.Y, ErrorQuat.Z);
    const double Sine = Vector.Size();
    const FVector RotationVector = Sine > UE_DOUBLE_SMALL_NUMBER ? Vector / Sine * (2.0 * FMath::Atan2(Sine, ErrorQuat.W)) : FVector::ZeroVector;

    FVector Rates = AttitudeP * RotationVector + RateFeedForward;
    for (int32 Axis = 0; Axis < 3; ++Axis)
    {
        const double Max = FMath::DegreesToRadians(FMath::Max(MaxBodyRateDegreesPerSecond[Axis], 0.0));
        Rates[Axis] = FMath::Clamp(Rates[Axis], -Max, Max);
    }
    return Rates;
}

void UUBotCascadedFlightController::RunRateLoop(const FUBotMultirotorState& State, const FVector& RateSetpoint, double Thrust, double DeltaSeconds,
    TArray<double>& OutMotorCommands)
{
    const FVector& Rates = State.AngularVelocityRadiansPerSecond;
    const FVector Error = RateSetpoint - Rates;
    RateIntegral = ClampIntegral(RateIntegral + Error * DeltaSeconds, RateI, RateIntegratorLimit);
    const FVector AngularAcceleration = RateP * Error + RateI * RateIntegral;
    const FVector Inertia = Airframe.InertiaKilogramSquareMeters;
    const FVector Torque = Inertia * AngularAcceleration + FVector::CrossProduct(Rates, Inertia * Rates);
    Allocator.Allocate(Thrust, Torque, OutMotorCommands);
}
