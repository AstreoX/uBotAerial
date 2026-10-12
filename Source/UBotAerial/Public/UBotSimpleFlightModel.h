#pragma once

#include "CoreMinimal.h"

/**
 * The simplified flight model of the navigation drone: it tracks a velocity and a yaw rate with a
 * first-order response, limited acceleration and limited jerk. No forces, no attitude dynamics; the
 * attitude is derived from the acceleration. SI units on UE world axes.
 */
namespace UBot::SimpleFlight
{
    struct FParams
    {
        double HorizontalTimeConstantSeconds = 0.3;
        double VerticalTimeConstantSeconds = 0.25;
        double MaxHorizontalAcceleration = 4.0;
        double MaxVerticalAcceleration = 3.0;
        /** m/s³; 0 turns the jerk limit off. */
        double JerkLimit = 30.0;
        double BrakeTimeConstantSeconds = 0.1;
        double BrakeHorizontalAcceleration = 6.0;
        double BrakeVerticalAcceleration = 4.0;
        double YawTimeConstantSeconds = 0.15;
        double MaxYawAcceleration = 6.0;
        /** Share of a wind change the drone drifts with before it compensates (0 ignores wind). */
        double WindResponse = 0.5;
        double WindRejectionTimeConstantSeconds = 1.0;
    };

    struct FState
    {
        FVector Velocity = FVector::ZeroVector;
        FVector Acceleration = FVector::ZeroVector;
        double Yaw = 0.0;
        double YawRate = 0.0;
        /** Low-passed wind the drone has compensated for. */
        FVector CompensatedWind = FVector::ZeroVector;
        bool bWindStarted = false;
        /** Drift velocity from wind changes during the latest step. */
        FVector WindDrift = FVector::ZeroVector;
    };

    struct FInput
    {
        /** Target velocity in the world frame, already clamped. */
        FVector TargetVelocity = FVector::ZeroVector;
        double TargetYawRate = 0.0;
        /** Brake parameters and the exact stop latch. */
        bool bBraking = false;
        /** Wind velocity at the drone, m/s. */
        FVector Wind = FVector::ZeroVector;
    };

    /** Advances State by StepSeconds and returns the displacement of this step, m. */
    UBOTAERIAL_API FVector Step(const FParams& Params, const FInput& Input, double StepSeconds, FState& State);

    /**
     * Attitude matching the acceleration: body Z along (Acceleration - Gravity), tilted at most
     * MaxTiltRadians, heading Yaw.
     */
    UBOTAERIAL_API FQuat ComputeAttitude(const FVector& Acceleration, const FVector& Gravity, double Yaw, double MaxTiltRadians);
}
