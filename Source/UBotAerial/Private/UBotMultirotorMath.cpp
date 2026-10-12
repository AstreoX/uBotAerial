#include "UBotMultirotorMath.h"

#include "UBotAerialTypes.h"

namespace UBot::Multirotor
{
    void FFixedStepClock::Reset(double NowSeconds, double RateHz)
    {
        AnchorSeconds = NowSeconds;
        StepSeconds = 1.0 / FMath::Clamp(RateHz, 1.0, 100000.0);
        StepIndex = 0;
        DroppedSteps = 0;
        bStarted = true;
    }

    void FFixedStepClock::SetRate(double RateHz)
    {
        const double NewStepSeconds = 1.0 / FMath::Clamp(RateHz, 1.0, 100000.0);
        if (NewStepSeconds == StepSeconds)
        {
            return;
        }
        AnchorSeconds = GetTimeSeconds();
        StepIndex = 0;
        StepSeconds = NewStepSeconds;
    }

    int32 FFixedStepClock::GetStepsDue(double NowSeconds, int32 MaxSteps)
    {
        if (!bStarted)
        {
            return 0;
        }
        // The epsilon keeps a sim time that lands on a step boundary from rounding one step short.
        const int64 Target = static_cast<int64>(FMath::FloorToDouble((NowSeconds - AnchorSeconds) / StepSeconds + 1e-6));
        int64 Due = Target - StepIndex;
        if (Due <= 0)
        {
            return 0;
        }
        const int64 Max = FMath::Max(MaxSteps, 1);
        if (Due > Max)
        {
            DroppedSteps += Due - Max;
            AnchorSeconds += static_cast<double>(Due - Max) * StepSeconds;
            Due = Max;
        }
        return static_cast<int32>(Due);
    }

    double GetYaw(const FQuat& Orientation)
    {
        const FVector Forward = Orientation.GetAxisX();
        if (FMath::Abs(Forward.X) < UE_DOUBLE_SMALL_NUMBER && FMath::Abs(Forward.Y) < UE_DOUBLE_SMALL_NUMBER)
        {
            // Nose straight up or down: take the heading from the body Z axis instead.
            const FVector Up = Orientation.GetAxisZ() * (Forward.Z > 0.0 ? -1.0 : 1.0);
            return FMath::Atan2(Up.Y, Up.X);
        }
        return FMath::Atan2(Forward.Y, Forward.X);
    }

    FQuat MakeYawQuat(double Yaw)
    {
        return FQuat(FVector::UpVector, Yaw);
    }

    double WrapAngle(double Angle)
    {
        Angle = FMath::Fmod(Angle + UE_DOUBLE_PI, UE_DOUBLE_TWO_PI);
        if (Angle <= 0.0)
        {
            Angle += UE_DOUBLE_TWO_PI;
        }
        return Angle - UE_DOUBLE_PI;
    }

    FQuat AttitudeFromThrustDirectionAndYaw(const FVector& ThrustDirection, double Yaw)
    {
        const FVector Z = ThrustDirection.GetSafeNormal(UE_DOUBLE_SMALL_NUMBER, FVector::UpVector);
        const FVector Heading(FMath::Cos(Yaw), FMath::Sin(Yaw), 0.0);
        return FRotationMatrix::MakeFromZX(Z, Heading).ToQuat().GetNormalized();
    }

    FVector LimitTilt(const FVector& ThrustDirection, double MaxTiltRadians)
    {
        const FVector Direction = ThrustDirection.GetSafeNormal(UE_DOUBLE_SMALL_NUMBER, FVector::UpVector);
        const double MaxTilt = FMath::Clamp(MaxTiltRadians, 0.0, FMath::DegreesToRadians(89.0));
        const double Tilt = FMath::Acos(FMath::Clamp(Direction.Z, -1.0, 1.0));
        if (Tilt <= MaxTilt)
        {
            return Direction;
        }
        const FVector Horizontal(Direction.X, Direction.Y, 0.0);
        const FVector Azimuth = Horizontal.GetSafeNormal();
        if (Azimuth.IsNearlyZero())
        {
            return FVector::UpVector;
        }
        return Azimuth * FMath::Sin(MaxTilt) + FVector::UpVector * FMath::Cos(MaxTilt);
    }

    FVector ClampNorm(const FVector& V, double MaxNorm)
    {
        const double Max = FMath::Max(MaxNorm, 0.0);
        const double SizeSquared = V.SizeSquared();
        if (SizeSquared <= Max * Max)
        {
            return V;
        }
        return V * (Max / FMath::Sqrt(SizeSquared));
    }

    void ClampVelocityCommand(const FUBotMultirotorVelocityLimits& Limits, FVector& InOutLinear, double& InOutYawRate)
    {
        const FVector Horizontal = ClampNorm(FVector(InOutLinear.X, InOutLinear.Y, 0.0), Limits.MaxHorizontalSpeedMetersPerSecond);
        const double Climb = FMath::Max(Limits.MaxClimbSpeedMetersPerSecond, 0.0f);
        const double Descent = FMath::Max(Limits.MaxDescentSpeedMetersPerSecond, 0.0f);
        InOutLinear = FVector(Horizontal.X, Horizontal.Y, FMath::Clamp(InOutLinear.Z, -Descent, Climb));
        const double YawLimit = FMath::Max(Limits.MaxYawRateRadiansPerSecond, 0.0f);
        InOutYawRate = FMath::Clamp(InOutYawRate, -YawLimit, YawLimit);
    }

    FQuat IntegrateBodyRate(const FQuat& Orientation, const FVector& BodyRate, double DeltaSeconds)
    {
        const double Rate = BodyRate.Size();
        const double Angle = Rate * DeltaSeconds;
        if (Angle <= UE_DOUBLE_SMALL_NUMBER)
        {
            return Orientation;
        }
        // Body-frame increment: the right-hand factor of UE's quaternion product is applied first.
        return (Orientation * FQuat(BodyRate / Rate, Angle)).GetNormalized();
    }

    FVector BodyRateBetween(const FQuat& From, const FQuat& To, double DeltaSeconds)
    {
        if (DeltaSeconds <= 0.0)
        {
            return FVector::ZeroVector;
        }
        FQuat Delta = From.Inverse() * To;
        if (Delta.W < 0.0)
        {
            Delta = FQuat(-Delta.X, -Delta.Y, -Delta.Z, -Delta.W);
        }
        FVector Axis;
        double Angle = 0.0;
        Delta.ToAxisAndAngle(Axis, Angle);
        return Axis * (Angle / DeltaSeconds);
    }

    FVector SpecificForce(const FQuat& Orientation, const FVector& AccelerationWorld, const FVector& Gravity)
    {
        return Orientation.UnrotateVector(AccelerationWorld - Gravity);
    }

    double GetTilt(const FQuat& Orientation)
    {
        return FMath::Acos(FMath::Clamp(Orientation.GetAxisZ().Z, -1.0, 1.0));
    }

    void GetQuadXRotorLayout(double ArmLengthMeters, TArray<FVector>& OutPositionsMeters, TArray<EUBotRotorSpinDirection>& OutSpinDirections)
    {
        const double D = FMath::Max(ArmLengthMeters, 0.0) / UE_DOUBLE_SQRT_2;
        OutPositionsMeters = { FVector(D, D, 0.0), FVector(-D, -D, 0.0), FVector(D, -D, 0.0), FVector(-D, D, 0.0) };
        OutSpinDirections = { EUBotRotorSpinDirection::CounterClockwise, EUBotRotorSpinDirection::CounterClockwise,
            EUBotRotorSpinDirection::Clockwise, EUBotRotorSpinDirection::Clockwise };
    }
}
