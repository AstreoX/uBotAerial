#pragma once

#include "CoreMinimal.h"
#include "UBotAerialTypes.generated.h"

/** Flight state of a multirotor. Append new values at the end. */
UENUM(BlueprintType)
enum class EUBotMultirotorFlightState : uint8
{
    /** Resting on the ground: frozen, level, rotors stopped. A climb command takes off. */
    Landed,
    /** In the air. */
    Flying,
    /** Hit something faster than the crash speed: rotors stopped, commands are rejected until Reset Flight. */
    Crashed
};

/** How a multirotor starts when play begins. Append new values at the end. */
UENUM(BlueprintType)
enum class EUBotMultirotorInitialState : uint8
{
    /** Landed when walkable ground is within a few centimeters below the drone, otherwise hovering. */
    Auto,
    /** Landed where it stands, until a climb command takes off. Use it for a drone placed on the ground. */
    Landed,
    /** Hovering where it stands. */
    Hovering
};

/** What the simplified flight model tilts. Append new values at the end. */
UENUM(BlueprintType)
enum class EUBotSimpleFlightTiltMode : uint8
{
    /** Nothing tilts: the drone only yaws. */
    None,
    /** Only the visual component tilts; the root and everything attached to it stay level. */
    Visual,
    /** The root tilts, so attached sensors see the attitude a real multirotor would have. */
    Body
};

/** Spin direction of a rotor seen from above. Append new values at the end. */
UENUM(BlueprintType)
enum class EUBotRotorSpinDirection : uint8
{
    Clockwise,
    CounterClockwise
};

/** Velocity limits of a multirotor. Commands above them are clamped, keeping the horizontal direction. */
USTRUCT(BlueprintType)
struct UBOTAERIAL_API FUBotMultirotorVelocityLimits
{
    GENERATED_BODY()

    /** Maximum horizontal speed (norm of the horizontal velocity), m/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor|Limits", meta = (ClampMin = "0.0", Units = "MetersPerSecond"))
    float MaxHorizontalSpeedMetersPerSecond = 10.0f;

    /** Maximum climb speed, m/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor|Limits", meta = (ClampMin = "0.0", Units = "MetersPerSecond"))
    float MaxClimbSpeedMetersPerSecond = 3.0f;

    /** Maximum descent speed (a positive number), m/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor|Limits", meta = (ClampMin = "0.0", Units = "MetersPerSecond"))
    float MaxDescentSpeedMetersPerSecond = 2.0f;

    /** Maximum yaw rate, rad/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor|Limits", meta = (ClampMin = "0.0"))
    float MaxYawRateRadiansPerSecond = UE_HALF_PI;
};

/** Ground-truth state of a multirotor at its latest physics step, in SI units on UE world axes. */
USTRUCT(BlueprintType)
struct UBOTAERIAL_API FUBotMultirotorState
{
    GENERATED_BODY()

    /** Position of the root in the world, m (UE axes: X forward, Y right, Z up). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    FVector PositionMeters = FVector::ZeroVector;

    /** Orientation of the body (body to world). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    FQuat Orientation = FQuat::Identity;

    /** Linear velocity in the world frame, m/s. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    FVector LinearVelocityMetersPerSecond = FVector::ZeroVector;

    /** Angular velocity in the body frame, rad/s, UE rotation sense (+Z yaws right). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    FVector AngularVelocityRadiansPerSecond = FVector::ZeroVector;

    /** Linear acceleration in the world frame over the latest step, m/s². */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    FVector LinearAccelerationMetersPerSecondSquared = FVector::ZeroVector;

    /** What an ideal accelerometer on the body reads (acceleration minus gravity, in the body frame), m/s². Hover reads about +9.81 on Z. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    FVector SpecificForceMetersPerSecondSquared = FVector::ZeroVector;

    /** Flight state at this step. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    EUBotMultirotorFlightState FlightState = EUBotMultirotorFlightState::Flying;

    /** uBot sim time of this step, s. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    double SimTimeSeconds = 0.0;
};
