#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "InputCoreTypes.h"
#include "UBotActuatorInterface.h"
#include "UBotAerialTypes.h"
#include "UBotMultirotorManualInputComponent.generated.h"

class AController;
class APawn;
class APlayerController;
class UInputAction;
class UInputMappingContext;

/** Mapping from manual input axes to multirotor commands (plain functions, no input system). */
namespace UBot::ManualFlight
{
    struct FSettings
    {
        double MaxHorizontalSpeed = 5.0;
        double MaxVerticalSpeed = 2.0;
        double MaxYawRate = UE_DOUBLE_HALF_PI;
        double MaxTilt = FMath::DegreesToRadians(30.0);
        double MaxBodyRate = FMath::DegreesToRadians(220.0);
        double DeadZone = 0.15;
    };

    /** One default key or stick: the axis it drives (0 forward, 1 right, 2 up, 3 yaw) and its sign. */
    struct FKeyBinding
    {
        FKey Key;
        int32 Axis = 0;
        double Scale = 1.0;
    };

    /** Keyboard: W/S, D/A, Space/Left Ctrl, E/Q. Gamepad (mode 2): right stick moves, left stick climbs and yaws. */
    UBOTAERIAL_API TArray<FKeyBinding> GetDefaultBindings();

    /** 0 inside the dead zone, rescaled so the edge of the dead zone maps to 0 and full deflection to 1. */
    UBOTAERIAL_API double ApplyDeadZone(double Value, double DeadZone);

    /** Raw axes with dead zones applied, each clamped to -1..1, and the horizontal pair limited to length 1. */
    UBOTAERIAL_API FUBotManualFlightAxes ConditionAxes(const FUBotManualFlightAxes& Raw, double DeadZone);

    /** Velocity in the heading frame and yaw rate. */
    UBOTAERIAL_API FUBotVelocityCommand MakeVelocityCommand(const FUBotManualFlightAxes& Axes, const FSettings& Settings);

    /** Tilt toward the stick (in the heading frame), heading turned at the yaw axis' rate, climb rate from the up axis. */
    UBOTAERIAL_API FUBotAttitudeCommand MakeAttitudeCommand(const FUBotManualFlightAxes& Axes, const FSettings& Settings, double& InOutYaw, double DeltaSeconds);

    /** Body rates: right stick rolls right, forward pitches nose down, yaw axis yaws right; climb rate from the up axis. */
    UBOTAERIAL_API FUBotBodyRateCommand MakeBodyRateCommand(const FUBotManualFlightAxes& Axes, const FSettings& Settings);
}

/**
 * Flies the owning multirotor with keyboard or gamepad while a local player controls (possesses) it.
 * The input actions and the mapping context are created at run time (Enhanced Input), so no assets
 * are needed; set your own to change the keys. While it controls the drone it sends a command every
 * frame. When control ends, it depends on the Flight Mode: in Velocity mode the drone brakes and hovers;
 * in Angle and Rate mode the attitude command is released, and the drone's Command Loss Action decides
 * what happens next (Hover by default).
 */
UCLASS(ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot Multirotor Manual Input"))
class UBOTAERIAL_API UUBotMultirotorManualInputComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UUBotMultirotorManualInputComponent();

    /** Take over the drone when a local player possesses its pawn. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input")
    bool bTakeControlWhenPossessed = true;

    /** What the sticks command. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input")
    EUBotManualFlightMode FlightMode = EUBotManualFlightMode::Velocity;

    /** Horizontal speed at full deflection (Velocity mode), m/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input", meta = (ClampMin = "0.0", Units = "MetersPerSecond"))
    float MaxHorizontalSpeedMetersPerSecond = 5.0f;

    /** Climb or descent speed at full deflection, m/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input", meta = (ClampMin = "0.0", Units = "MetersPerSecond"))
    float MaxVerticalSpeedMetersPerSecond = 2.0f;

    /** Yaw rate at full deflection, deg/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input", meta = (ClampMin = "0.0"))
    float MaxYawRateDegreesPerSecond = 90.0f;

    /** Tilt at full deflection (Angle mode), degrees. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input", meta = (ClampMin = "0.0", ClampMax = "60.0", Units = "Degrees"))
    float MaxTiltDegrees = 30.0f;

    /** Roll and pitch rate at full deflection (Rate mode), deg/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input", meta = (ClampMin = "0.0"))
    float MaxBodyRateDegreesPerSecond = 220.0f;

    /** Stick dead zone, share of full deflection. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input", meta = (ClampMin = "0.0", ClampMax = "0.9"))
    float DeadZone = 0.15f;

    /** Your own mapping context; it must map Custom Move Action and Custom Yaw Action. Empty uses the built-in keys. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input|Custom")
    TObjectPtr<UInputMappingContext> CustomMappingContext;

    /** Your own move action (Axis3D: forward, right, up), used with Custom Mapping Context. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input|Custom")
    TObjectPtr<UInputAction> CustomMoveAction;

    /** Your own yaw action (Axis1D, + yaws right), used with Custom Mapping Context. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input|Custom")
    TObjectPtr<UInputAction> CustomYawAction;

    /** Priority of the mapping context. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Manual Input|Custom")
    int32 MappingPriority = 1;

    /** True while this component flies the drone. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Input", meta = (DisplayName = "Is Controlling"))
    bool IsControlling() const { return bControlling; }

    /** The axes of the latest frame, after dead zones. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Input", meta = (DisplayName = "Get Manual Flight Axes"))
    FUBotManualFlightAxes GetAxes() const { return Axes; }

    /** The built-in mapping context (null until a player took control). */
    const UInputMappingContext* GetDefaultMappingContext() const { return DefaultMappingContext; }

    /** Builds the built-in actions and mapping context (done when control starts). */
    void BuildDefaultInput();

    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    UFUNCTION()
    void HandleControllerChanged(APawn* Pawn, AController* OldController, AController* NewController);

    void TakeControl(APlayerController* PlayerController);
    void ReleaseControl();
    UBot::ManualFlight::FSettings MakeSettings() const;
    const UInputMappingContext* GetMappingContext() const;
    const UInputAction* GetMoveAction() const;
    const UInputAction* GetYawAction() const;

    UPROPERTY(Transient)
    TObjectPtr<UInputMappingContext> DefaultMappingContext;

    UPROPERTY(Transient)
    TObjectPtr<UInputAction> DefaultMoveAction;

    UPROPERTY(Transient)
    TObjectPtr<UInputAction> DefaultYawAction;

    TWeakObjectPtr<APlayerController> ControllingPlayer;
    FUBotManualFlightAxes Axes;
    double YawSetpoint = 0.0;
    bool bControlling = false;
    bool bYawSetpointValid = false;
    bool bWarnedNoEnhancedInput = false;
    bool bWarnedNoTarget = false;
};
