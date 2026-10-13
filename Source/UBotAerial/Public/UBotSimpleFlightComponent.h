#pragma once

#include "CoreMinimal.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotSimpleFlightModel.h"
#include "UBotSimpleFlightComponent.generated.h"

/**
 * Simplified flight control for navigation research: the drone tracks velocity commands with a
 * smooth, bounded response (first order, limited acceleration and jerk) and brakes to a stop within
 * a bounded distance. There are no forces or attitude dynamics; the tilt follows the acceleration.
 */
UCLASS(ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot Simple Flight"))
class UBOTAERIAL_API UUBotSimpleFlightComponent : public UUBotMultirotorMovementComponent
{
    GENERATED_BODY()

public:
    UUBotSimpleFlightComponent();

    /** What tilts with the acceleration. Visual keeps the root and attached sensors level and tilts only the Visual Component. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight")
    EUBotSimpleFlightTiltMode TiltMode = EUBotSimpleFlightTiltMode::Visual;

    /** Largest tilt, degrees. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight", meta = (ClampMin = "0.0", ClampMax = "60.0", Units = "Degrees"))
    float MaxTiltDegrees = 25.0f;

    /** The component that tilts in Visual mode (usually the parent of the drone's meshes). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight", meta = (UseComponentPicker, AllowedClasses = "/Script/Engine.SceneComponent"))
    FComponentReference VisualComponent;

    /** Time constant of the horizontal velocity response, s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Tracking", meta = (ClampMin = "0.01", Units = "Seconds"))
    float HorizontalTimeConstantSeconds = 0.3f;

    /** Time constant of the vertical velocity response, s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Tracking", meta = (ClampMin = "0.01", Units = "Seconds"))
    float VerticalTimeConstantSeconds = 0.25f;

    /** Largest horizontal acceleration, m/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Tracking", meta = (ClampMin = "0.0"))
    float MaxHorizontalAccelerationMetersPerSecondSquared = 4.0f;

    /** Largest vertical acceleration, m/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Tracking", meta = (ClampMin = "0.0"))
    float MaxVerticalAccelerationMetersPerSecondSquared = 3.0f;

    /** Largest change of acceleration, m/s³. 0 turns the limit off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Tracking", meta = (ClampMin = "0.0"))
    float JerkLimitMetersPerSecondCubed = 30.0f;

    /** Time constant of the yaw rate response, s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Tracking", meta = (ClampMin = "0.01", Units = "Seconds"))
    float YawTimeConstantSeconds = 0.15f;

    /** Largest yaw acceleration, rad/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Tracking", meta = (ClampMin = "0.0"))
    float MaxYawAccelerationRadiansPerSecondSquared = 6.0f;

    /** Time constant while braking after Stop Motion, s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Braking", meta = (ClampMin = "0.01", Units = "Seconds"))
    float BrakeTimeConstantSeconds = 0.1f;

    /** Largest horizontal deceleration while braking, m/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Braking", meta = (ClampMin = "0.0"))
    float BrakeHorizontalAccelerationMetersPerSecondSquared = 6.0f;

    /** Largest vertical deceleration while braking, m/s². */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Braking", meta = (ClampMin = "0.0"))
    float BrakeVerticalAccelerationMetersPerSecondSquared = 4.0f;

    /** Share of a wind change the drone drifts with before it compensates. 0 ignores wind. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Wind", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float WindResponse = 0.5f;

    /** How fast the drone compensates a wind change, s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight|Wind", meta = (ClampMin = "0.01", Units = "Seconds"))
    float WindRejectionTimeConstantSeconds = 1.0f;

    /** Rotor speed shown while hovering, rad/s (visual only). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simple Flight", AdvancedDisplay, meta = (ClampMin = "0.0"))
    float HoverRotorSpeedRadiansPerSecond = 794.0f;

    /** True while braking after Stop Motion, until the next velocity command. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Is Braking"))
    bool IsBraking() const { return bBraking; }

    /** The tilted attitude of the latest step (also in Visual mode, where the root stays level). */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Tilted Attitude"))
    FQuat GetTiltedAttitude() const { return TiltedAttitude; }

    virtual void BeginPlay() override;

protected:
    virtual void ResetModel() override;
    virtual void SimulateStep(double StepSeconds) override;
    virtual bool AcceptVelocityCommand(const FVector& LinearHeading, double YawRate) override;
    virtual void HandleStopMotion() override;
    virtual void HandleExternalTeleport() override;
    virtual void UpdateVisuals(double FrameSeconds) override;
    virtual double GetVisualRotorSpeed(int32 RotorIndex) const override;
    virtual double GetMaxAccelerationBound() const override;

private:
    UBot::SimpleFlight::FParams MakeParams() const;
    FQuat MakeBodyOrientation() const;

    UBot::SimpleFlight::FState Model;
    FVector TargetVelocityHeading = FVector::ZeroVector;
    double TargetYawRate = 0.0;
    bool bBraking = false;
    FQuat TiltedAttitude = FQuat::Identity;
    TWeakObjectPtr<USceneComponent> Visual;
    FQuat VisualRestRotation = FQuat::Identity;
};
