#include "UBotSimpleFlightModel.h"

#include "UBotMultirotorMath.h"

namespace UBot::SimpleFlight
{
    namespace
    {
        // Velocity and acceleration below these count as stopped when braking.
        constexpr double StopSpeed = 1e-3;
        constexpr double StopAcceleration = 1e-2;
    }

    FVector Step(const FParams& Params, const FInput& Input, double StepSeconds, FState& State)
    {
        const double H = FMath::Max(StepSeconds, 0.0);
        const bool bBraking = Input.bBraking;
        const double TauXY = FMath::Max(bBraking ? Params.BrakeTimeConstantSeconds : Params.HorizontalTimeConstantSeconds, 1e-3);
        const double TauZ = FMath::Max(bBraking ? Params.BrakeTimeConstantSeconds : Params.VerticalTimeConstantSeconds, 1e-3);
        const double MaxXY = FMath::Max(bBraking ? Params.BrakeHorizontalAcceleration : Params.MaxHorizontalAcceleration, 0.0);
        const double MaxZ = FMath::Max(bBraking ? Params.BrakeVerticalAcceleration : Params.MaxVerticalAcceleration, 0.0);

        // First-order tracking with limited acceleration, horizontal and vertical separately.
        const FVector Error = Input.TargetVelocity - State.Velocity;
        const FVector TargetXY = UBot::Multirotor::ClampNorm(FVector(Error.X, Error.Y, 0.0) / TauXY, MaxXY);
        const FVector TargetAcceleration(TargetXY.X, TargetXY.Y, FMath::Clamp(Error.Z / TauZ, -MaxZ, MaxZ));

        // Limited jerk smooths the acceleration (and with it the attitude and an IMU reading).
        const FVector AccelerationChange = TargetAcceleration - State.Acceleration;
        State.Acceleration += Params.JerkLimit > 0.0 ? UBot::Multirotor::ClampNorm(AccelerationChange, Params.JerkLimit * H) : AccelerationChange;
        State.Velocity += State.Acceleration * H;

        if (bBraking && State.Velocity.Size() < StopSpeed && State.Acceleration.Size() < StopAcceleration)
        {
            // Without this latch a first-order stop creeps on forever.
            State.Velocity = FVector::ZeroVector;
            State.Acceleration = FVector::ZeroVector;
        }

        // Yaw rate: first order with limited angular acceleration.
        const double YawAcceleration = FMath::Clamp((Input.TargetYawRate - State.YawRate) / FMath::Max(Params.YawTimeConstantSeconds, 1e-3),
            -Params.MaxYawAcceleration, Params.MaxYawAcceleration);
        State.YawRate += YawAcceleration * H;
        if (bBraking && FMath::Abs(State.YawRate) < StopSpeed && FMath::Abs(Input.TargetYawRate) < StopSpeed)
        {
            State.YawRate = 0.0;
        }
        State.Yaw = UBot::Multirotor::WrapAngle(State.Yaw + State.YawRate * H);

        // A velocity-controlled drone compensates steady wind; a change pushes it until the controller catches up.
        if (!State.bWindStarted)
        {
            State.CompensatedWind = Input.Wind;
            State.bWindStarted = true;
        }
        const double Blend = 1.0 - FMath::Exp(-H / FMath::Max(Params.WindRejectionTimeConstantSeconds, 1e-3));
        State.CompensatedWind += (Input.Wind - State.CompensatedWind) * Blend;
        State.WindDrift = (Input.Wind - State.CompensatedWind) * FMath::Max(Params.WindResponse, 0.0);

        return (State.Velocity + State.WindDrift) * H;
    }

    FQuat ComputeAttitude(const FVector& Acceleration, const FVector& Gravity, double Yaw, double MaxTiltRadians)
    {
        const FVector Thrust = UBot::Multirotor::LimitTilt(Acceleration - Gravity, MaxTiltRadians);
        return UBot::Multirotor::AttitudeFromThrustDirectionAndYaw(Thrust, Yaw);
    }
}
