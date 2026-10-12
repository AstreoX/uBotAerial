#pragma once

#include "CoreMinimal.h"
#include "UBotAerialTypes.h"

/**
 * Rigid-body dynamics of a multirotor and the thrust allocation shared by flight controllers. SI
 * units on UE world axes; torques and angular rates use the UE rotation sense, so the usual
 * right-handed formulas apply component by component (positions and forces mirror, axial vectors
 * flip sign, and r x F keeps its form).
 */
namespace UBot::Multirotor
{
    /** Kinematic state of a multirotor body plus its rotor speeds. */
    struct FRigidBodyState
    {
        FVector Position = FVector::ZeroVector;
        FVector Velocity = FVector::ZeroVector;
        FQuat Orientation = FQuat::Identity;
        /** Body frame, rad/s. */
        FVector AngularVelocity = FVector::ZeroVector;
        /** Motor order, rad/s. */
        TArray<double> RotorSpeeds;
        /** World acceleration over the latest step, m/s². */
        FVector Acceleration = FVector::ZeroVector;
    };

    /** Rotors of a quadrotor X with arms of ArmLengthMeters, in PX4 order, with the default motor. */
    UBOTAERIAL_API TArray<FUBotRotorSpec> MakeQuadXRotors(double ArmLengthMeters);

    /** Thrust of a rotor at full speed, N. */
    UBOTAERIAL_API double GetMaxRotorThrust(const FUBotRotorSpec& Rotor);

    /** Rotor speed a normalized command (0..1) asks for, rad/s. */
    UBOTAERIAL_API double CommandToRotorSpeed(const FUBotRotorSpec& Rotor, EUBotMotorCommandMapping Mapping, double Command);

    /** Normalized command (0..1) for a rotor speed, rad/s. */
    UBOTAERIAL_API double RotorSpeedToCommand(const FUBotRotorSpec& Rotor, EUBotMotorCommandMapping Mapping, double SpeedRadiansPerSecond);

    /** Normalized command that holds a level hover with every rotor equal (thrust mapping assumes equal rotors). */
    UBOTAERIAL_API double GetHoverCommand(const FUBotMultirotorAirframe& Airframe, double GravityMetersPerSecondSquared);

    /** True when the airframe can fly: mass and inertia positive, at least one rotor with thrust. OutError says why not. */
    UBOTAERIAL_API bool IsAirframeValid(const FUBotMultirotorAirframe& Airframe, FString* OutError = nullptr);

    /** Thrust (N, along body Z) and torque (N·m, body frame) the rotors produce at these speeds. */
    UBOTAERIAL_API void ComputeRotorWrench(const FUBotMultirotorAirframe& Airframe, TConstArrayView<double> RotorSpeeds, double& OutThrust, FVector& OutTorque);

    /**
     * One semi-implicit Euler step: motor lag (exact), rotor forces and torques, air-relative drag,
     * gravity, Euler's rotation equation and an exponential-map attitude update. Commands are normalized
     * (0..1) in motor order; missing ones count as 0.
     */
    UBOTAERIAL_API void StepRigidBody(const FUBotMultirotorAirframe& Airframe, TConstArrayView<double> Commands, const FVector& Gravity,
        const FVector& Wind, double StepSeconds, FRigidBodyState& InOutState);

    /**
     * Maps a collective thrust and a body torque to normalized motor commands through the pseudo-inverse
     * of the rotor geometry. When the request does not fit, the yaw torque is reduced first, then the
     * collective thrust is shifted, then the commands are clipped.
     */
    class UBOTAERIAL_API FThrustAllocator
    {
    public:
        /** Prepares the allocation for Airframe. Returns false when the rotors cannot produce thrust. */
        bool Build(const FUBotMultirotorAirframe& Airframe);

        bool IsBuilt() const { return RotorCount > 0; }
        int32 GetRotorCount() const { return RotorCount; }

        /** Largest total thrust, N. */
        double GetMaxTotalThrust() const { return MaxTotalThrust; }

        /** Commands for Thrust (N) and Torque (N·m, body frame); OutCommands gets one value per rotor. */
        void Allocate(double Thrust, const FVector& Torque, TArray<double>& OutCommands) const;

    private:
        int32 RotorCount = 0;
        /** Row-major RotorCount x 4: rotor thrust from (thrust, torque x, torque y, torque z). */
        TArray<double> Inverse;
        TArray<double> MaxThrust;
        double MaxTotalThrust = 0.0;
        EUBotMotorCommandMapping Mapping = EUBotMotorCommandMapping::Thrust;
    };
}
