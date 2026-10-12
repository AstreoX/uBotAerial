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

/** Which command a physics multirotor flies. Append new values at the end. */
UENUM(BlueprintType)
enum class EUBotFlightControlMode : uint8
{
    /** Velocity in the heading frame plus yaw rate (the full built-in cascade). */
    Velocity,
    /** Attitude plus thrust or climb rate. */
    Attitude,
    /** Body rates plus thrust or climb rate. */
    BodyRate,
    /** One command per motor; no flight controller. */
    Motor
};

/** What a physics multirotor does when its commands stop (released by the sender or timed out). Append new values at the end. */
UENUM(BlueprintType)
enum class EUBotCommandLossAction : uint8
{
    /** Brake and hold the position. */
    Hover,
    /** Descend in place until landed. */
    Land,
    /** Stop all motors (the drone falls). */
    CutMotors
};

/** How a normalized motor command (0..1) maps to rotor speed. Append new values at the end. */
UENUM(BlueprintType)
enum class EUBotMotorCommandMapping : uint8
{
    /** Thrust grows linearly with the command: speed = max speed * sqrt(command). */
    Thrust,
    /** Speed grows linearly with the command: speed = max speed * command. */
    Speed
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

    /** Rotor speeds in motor order, rad/s (empty for the simplified flight model). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    TArray<double> RotorSpeedsRadiansPerSecond;

    /** Flight state at this step. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    EUBotMultirotorFlightState FlightState = EUBotMultirotorFlightState::Flying;

    /** uBot sim time of this step, s. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|State")
    double SimTimeSeconds = 0.0;
};

/** One rotor of a multirotor: where it sits, how it spins, and its motor. */
USTRUCT(BlueprintType)
struct UBOTAERIAL_API FUBotRotorSpec
{
    GENERATED_BODY()

    /** Position of the rotor hub in the body frame, m (UE axes: X forward, Y right, Z up). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor")
    FVector PositionMeters = FVector::ZeroVector;

    /** Spin direction seen from above. The body feels the opposite reaction torque. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor")
    EUBotRotorSpinDirection SpinDirection = EUBotRotorSpinDirection::Clockwise;

    /** Thrust = this * speed², N/(rad/s)². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor", meta = (ClampMin = "0.0"))
    double ThrustCoefficient = 5.84e-6;

    /** Reaction torque = this * speed², N·m/(rad/s)². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor", meta = (ClampMin = "0.0"))
    double TorqueCoefficient = 9.344e-8;

    /** Largest rotor speed, rad/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor", meta = (ClampMin = "1.0"))
    double MaxSpeedRadiansPerSecond = 1100.0;

    /** First-order time constant of the motor speed, s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor", meta = (ClampMin = "0.0"))
    double MotorTimeConstantSeconds = 0.02;
};

/** Mass, inertia, rotors and drag of a multirotor. The default is a 1.5 kg, 450 mm quadrotor X. */
USTRUCT(BlueprintType)
struct UBOTAERIAL_API FUBotMultirotorAirframe
{
    GENERATED_BODY()

    FUBotMultirotorAirframe();

    /** Mass, kg. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Airframe", meta = (ClampMin = "0.01"))
    double MassKilograms = 1.5;

    /** Principal moments of inertia about the body X, Y and Z axes, kg·m². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Airframe")
    FVector InertiaKilogramSquareMeters = FVector(0.029, 0.029, 0.055);

    /** The rotors, in motor order (motor commands use this order). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Airframe")
    TArray<FUBotRotorSpec> Rotors;

    /** Linear drag along the body axes, N per m/s of air-relative speed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Airframe")
    FVector LinearDragCoefficients = FVector(0.25, 0.25, 0.30);

    /** Quadratic drag along the body axes, N per (m/s)² of air-relative speed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Airframe")
    FVector QuadraticDragCoefficients = FVector(0.03, 0.03, 0.05);

    /** Rotational drag about the body axes, N·m per rad/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Airframe")
    FVector AngularDragCoefficients = FVector(0.003, 0.003, 0.005);

    /** How a normalized motor command maps to rotor speed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Airframe")
    EUBotMotorCommandMapping MotorCommandMapping = EUBotMotorCommandMapping::Thrust;
};

/** What a flight controller is asked to fly during one physics step. SI units on UE world axes. */
USTRUCT(BlueprintType)
struct UBOTAERIAL_API FUBotFlightSetpoint
{
    GENERATED_BODY()

    /** Which fields apply. Motor mode never reaches a controller. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    EUBotFlightControlMode Mode = EUBotFlightControlMode::Velocity;

    /** Velocity mode: target velocity in the world frame, m/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    FVector VelocityMetersPerSecond = FVector::ZeroVector;

    /** Velocity mode: yaw rate, rad/s (+ turns right). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    double YawRateRadiansPerSecond = 0.0;

    /** Velocity mode: hold Hold Position horizontally, and its altitude while the vertical velocity is 0. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    bool bHoldPosition = false;

    /** Velocity mode: the position to hold, m (world). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    FVector HoldPositionMeters = FVector::ZeroVector;

    /** Attitude mode: target attitude, body to world. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    FQuat Orientation = FQuat::Identity;

    /** Attitude mode: turn the heading at Yaw Rate instead of following the yaw of Orientation. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    bool bUseYawRate = false;

    /** Body rate mode: target angular velocity in the body frame, rad/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    FVector BodyRateRadiansPerSecond = FVector::ZeroVector;

    /** Attitude and body rate modes: use Normalized Thrust; otherwise fly Vertical Velocity. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    bool bUseThrust = false;

    /** Collective thrust as a share of the maximum, 0..1. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    double NormalizedThrust = 0.0;

    /** Attitude and body rate modes without thrust: climb rate, m/s (+ up). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setpoint")
    double VerticalVelocityMetersPerSecond = 0.0;
};

/** Wind: a constant part plus gusts. The gusts are a sum of sines with seeded phases, so the same settings give the same wind at the same sim time. */
USTRUCT(BlueprintType)
struct UBOTAERIAL_API FUBotWindSettings
{
    GENERATED_BODY()

    /** Steady wind velocity in the world frame, m/s (the direction the air moves). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind")
    FVector ConstantVelocityMetersPerSecond = FVector::ZeroVector;

    /** Standard deviation of the gusts per world axis, m/s. Zero turns gusts off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind")
    FVector GustStandardDeviationMetersPerSecond = FVector::ZeroVector;

    /** Typical duration of a gust, s. Longer is smoother. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (ClampMin = "0.05", Units = "Seconds"))
    float GustCorrelationTimeSeconds = 2.0f;

    /** Seed of the gust pattern. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind")
    int32 Seed = 0;
};

/** How keyboard and gamepad input flies a multirotor. Append new values at the end. */
UENUM(BlueprintType)
enum class EUBotManualFlightMode : uint8
{
    /** Sticks command velocity in the heading frame and yaw rate (any drone that takes velocity commands). */
    Velocity,
    /** Sticks command tilt and yaw rate, the up axis commands climb rate (drones that take attitude commands). */
    Angle,
    /** Sticks command body rates, the up axis commands climb rate (drones that take body-rate commands). */
    Rate
};

/** Manual flight input, each axis -1..1. */
USTRUCT(BlueprintType)
struct UBOTAERIAL_API FUBotManualFlightAxes
{
    GENERATED_BODY()

    /** + forward. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input")
    float Forward = 0.0f;

    /** + right. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input")
    float Right = 0.0f;

    /** + up. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input")
    float Up = 0.0f;

    /** + yaw right. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input")
    float Yaw = 0.0f;
};
