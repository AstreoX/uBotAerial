#pragma once

#include "CoreMinimal.h"
#include "UBotAerialTypes.h"

/**
 * Plain math shared by the multirotor flight models. SI units on UE world axes (X forward, Y right,
 * Z up); rotations are FQuat (body to world) and angular rates use the UE rotation sense, so the
 * usual rigid-body formulas apply component by component.
 */
namespace UBot::Multirotor
{
    /**
     * Fixed-step clock on sim time. Physics time is Anchor + StepIndex * StepSeconds, never a running
     * sum of frame deltas, so the same sim times always give the same steps.
     */
    struct UBOTAERIAL_API FFixedStepClock
    {
        /** Starts counting at NowSeconds with RateHz steps per second (clamped to 1..100000). */
        void Reset(double NowSeconds, double RateHz);

        /** Changes the rate from the current physics time on. */
        void SetRate(double RateHz);

        /**
         * Steps to run so physics time reaches NowSeconds, at most MaxSteps. When more are due, the
         * extra time is skipped (counted in GetDroppedSteps) instead of being caught up later.
         */
        int32 GetStepsDue(double NowSeconds, int32 MaxSteps);

        /** Counts one step as done. */
        void AdvanceStep() { ++StepIndex; }

        double GetStepSeconds() const { return StepSeconds; }
        double GetTimeSeconds() const { return AnchorSeconds + static_cast<double>(StepIndex) * StepSeconds; }
        int64 GetStepIndex() const { return StepIndex; }
        int64 GetDroppedSteps() const { return DroppedSteps; }
        bool IsStarted() const { return bStarted; }

    private:
        double AnchorSeconds = 0.0;
        double StepSeconds = 0.001;
        int64 StepIndex = 0;
        int64 DroppedSteps = 0;
        bool bStarted = false;
    };

    /** Heading of a body: the angle of its X axis projected on the horizontal plane, rad (+ turns right). */
    UBOTAERIAL_API double GetYaw(const FQuat& Orientation);

    /** Rotation about world Z by Yaw (rad, + turns right). */
    UBOTAERIAL_API FQuat MakeYawQuat(double Yaw);

    /** Wraps an angle to (-pi, pi]. */
    UBOTAERIAL_API double WrapAngle(double Angle);

    /**
     * Attitude whose body Z axis points along ThrustDirection and whose X axis is as close as possible
     * to the heading Yaw. A zero direction counts as straight up.
     */
    UBOTAERIAL_API FQuat AttitudeFromThrustDirectionAndYaw(const FVector& ThrustDirection, double Yaw);

    /** ThrustDirection tilted at most MaxTiltRadians away from world up, keeping its azimuth. */
    UBOTAERIAL_API FVector LimitTilt(const FVector& ThrustDirection, double MaxTiltRadians);

    /** Scales V down to at most MaxNorm. */
    UBOTAERIAL_API FVector ClampNorm(const FVector& V, double MaxNorm);

    /**
     * Clamps a velocity command: the horizontal norm (keeping its direction), climb and descent
     * separately, and the yaw rate. Negative limits count as zero.
     */
    UBOTAERIAL_API void ClampVelocityCommand(const FUBotMultirotorVelocityLimits& Limits, FVector& InOutLinear, double& InOutYawRate);

    /** Orientation after rotating at BodyRate (body frame, rad/s) for DeltaSeconds; normalized. */
    UBOTAERIAL_API FQuat IntegrateBodyRate(const FQuat& Orientation, const FVector& BodyRate, double DeltaSeconds);

    /** Body rate that turns From into To in DeltaSeconds (shortest way), rad/s in the body frame of From. */
    UBOTAERIAL_API FVector BodyRateBetween(const FQuat& From, const FQuat& To, double DeltaSeconds);

    /** What an ideal accelerometer reads: (Acceleration - Gravity) in the body frame, m/s². */
    UBOTAERIAL_API FVector SpecificForce(const FQuat& Orientation, const FVector& AccelerationWorld, const FVector& Gravity);

    /** Tilt of a body away from level (angle between its Z axis and world up), rad. */
    UBOTAERIAL_API double GetTilt(const FQuat& Orientation);

    /**
     * Rotor positions (body frame, m) and spin directions of a quadrotor X with arms of ArmLengthMeters,
     * in PX4 motor order: front right (CCW), rear left (CCW), front left (CW), rear right (CW).
     */
    UBOTAERIAL_API void GetQuadXRotorLayout(double ArmLengthMeters, TArray<FVector>& OutPositionsMeters, TArray<EUBotRotorSpinDirection>& OutSpinDirections);
}
