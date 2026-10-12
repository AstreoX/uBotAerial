#pragma once

#include "CoreMinimal.h"
#include "UBotFlightController.h"
#include "UBotMultirotorDynamics.h"
#include "UBotCascadedFlightController.generated.h"

/**
 * The built-in flight controller: velocity (PID, world frame) -> thrust vector and attitude ->
 * attitude (P on the quaternion error) -> body rates (PI with inertia feed-forward) -> thrust
 * allocation over the rotors. Each mode enters the cascade at its own level. The default gains suit
 * the default airframe at 1 kHz.
 */
UCLASS(meta = (DisplayName = "uBot Cascaded Flight Controller"))
class UBOTAERIAL_API UUBotCascadedFlightController : public UUBotFlightController
{
    GENERATED_BODY()

public:
    /** Velocity loop P gain per world axis, 1/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity")
    FVector VelocityP = FVector(1.8, 1.8, 4.0);

    /** Velocity loop I gain per world axis, 1/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity")
    FVector VelocityI = FVector(0.4, 0.4, 2.0);

    /** Velocity loop D gain (on the measured acceleration) per world axis. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity")
    FVector VelocityD = FVector(0.2, 0.2, 0.0);

    /** Largest acceleration the velocity integrator may add, per world axis, m/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity")
    FVector VelocityIntegratorLimit = FVector(3.0, 3.0, 5.0);

    /** Position hold gain, 1/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity", meta = (ClampMin = "0.0"))
    double PositionHoldGain = 1.5;

    /** Largest correction speed of the position hold, m/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity", meta = (ClampMin = "0.0"))
    double PositionHoldMaxSpeed = 1.0;

    /** Largest tilt the velocity loop asks for, degrees. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity", meta = (ClampMin = "1.0", ClampMax = "80.0"))
    double MaxTiltDegrees = 35.0;

    /** Attitude loop P gain per body axis, 1/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Attitude")
    FVector AttitudeP = FVector(6.5, 6.5, 2.8);

    /** Largest body rate the attitude loop asks for, per body axis, deg/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Attitude")
    FVector MaxBodyRateDegreesPerSecond = FVector(220.0, 220.0, 200.0);

    /** Rate loop P gain per body axis, 1/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rate")
    FVector RateP = FVector(20.0, 20.0, 8.0);

    /** Rate loop I gain per body axis, 1/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rate")
    FVector RateI = FVector(5.0, 5.0, 2.0);

    /** Largest angular acceleration the rate integrator may add, per body axis, rad/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rate")
    FVector RateIntegratorLimit = FVector(10.0, 10.0, 2.0);

    /** Smallest collective thrust the velocity loop asks for, as a share of the maximum (keeps attitude control alive). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thrust", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double MinThrustShare = 0.08;

    /** Largest collective thrust the controller asks for, as a share of the maximum (leaves room for attitude control). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thrust", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double MaxThrustShare = 0.95;

protected:
    virtual void ResetController_Implementation(const FUBotMultirotorState& State, const FUBotMultirotorAirframe& Airframe) override;
    virtual void ComputeMotorCommands_Implementation(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint, double DeltaSeconds,
        TArray<double>& OutMotorCommands) override;
    virtual bool SupportsControlMode_Implementation(EUBotFlightControlMode Mode) const override;

private:
    /** Collective thrust (N) and target attitude for a world-frame velocity setpoint. */
    void RunVelocityLoop(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint, double DeltaSeconds, double& OutThrust, FQuat& OutAttitude);
    /** Collective thrust (N) for a climb rate with the body tilted as it is now. */
    double RunVerticalLoop(const FUBotMultirotorState& State, double VerticalVelocity, double DeltaSeconds);
    /** Body-rate setpoint for a target attitude. */
    FVector RunAttitudeLoop(const FUBotMultirotorState& State, const FQuat& Target, const FVector& RateFeedForward) const;
    /** Motor commands for a body-rate setpoint and a collective thrust. */
    void RunRateLoop(const FUBotMultirotorState& State, const FVector& RateSetpoint, double Thrust, double DeltaSeconds, TArray<double>& OutMotorCommands);

    FUBotMultirotorAirframe Airframe;
    UBot::Multirotor::FThrustAllocator Allocator;
    FVector VelocityIntegral = FVector::ZeroVector;
    FVector RateIntegral = FVector::ZeroVector;
    double YawSetpoint = 0.0;
    bool bHasYawSetpoint = false;
    double GravityMagnitude = 9.81;
};
