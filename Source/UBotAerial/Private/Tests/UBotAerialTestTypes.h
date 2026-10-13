#pragma once

#include "CoreMinimal.h"
#include "Engine/HitResult.h"
#include "GameFramework/Pawn.h"
#include "UBotAerialTypes.h"
#include "UBotFlightController.h"
#include "UObject/Object.h"
#include "UBotAerialTestTypes.generated.h"

class UBoxComponent;
class USceneComponent;
class UUBotIdentityComponent;
class UUBotMultirotorDynamicsComponent;
class UUBotMultirotorManualInputComponent;
class UUBotMultirotorMovementComponent;
class UUBotRotorComponent;
class UUBotSimpleFlightComponent;

// Test-only quadrotor with the structure the example Blueprints use: a collision box root, a visual root,
// four rotors without meshes, identity, manual input and a movement component chosen by the subclass.
UCLASS(Abstract, Transient, NotBlueprintable, NotPlaceable, HideDropdown)
class AUBotAerialTestDrone : public APawn
{
    GENERATED_BODY()

public:
    AUBotAerialTestDrone(const FObjectInitializer& ObjectInitializer);

    static const FName MovementComponentName;
    UUBotMultirotorMovementComponent* GetMultirotorMovement() const { return Movement; }

    UPROPERTY()
    TObjectPtr<UBoxComponent> CollisionBox;

    UPROPERTY()
    TObjectPtr<USceneComponent> VisualRoot;

    UPROPERTY()
    TArray<TObjectPtr<UUBotRotorComponent>> Rotors;

    UPROPERTY()
    TObjectPtr<UUBotIdentityComponent> Identity;

    UPROPERTY()
    TObjectPtr<UUBotMultirotorManualInputComponent> ManualInput;

    UPROPERTY()
    TObjectPtr<UUBotMultirotorMovementComponent> Movement;
};

// Test-only navigation drone using Simple Flight and visual tilt.
UCLASS(Transient, NotBlueprintable, NotPlaceable, HideDropdown)
class AUBotAerialTestNavigationDrone : public AUBotAerialTestDrone
{
    GENERATED_BODY()

public:
    AUBotAerialTestNavigationDrone(const FObjectInitializer& ObjectInitializer);

    UUBotSimpleFlightComponent* GetSimpleFlight() const;
};

// Test-only physics drone with the dynamics component's default command acceptance.
UCLASS(Transient, NotBlueprintable, NotPlaceable, HideDropdown)
class AUBotAerialTestPhysicsDrone : public AUBotAerialTestDrone
{
    GENERATED_BODY()

public:
    AUBotAerialTestPhysicsDrone(const FObjectInitializer& ObjectInitializer);

    UUBotMultirotorDynamicsComponent* GetDynamics() const;
};

// Test-only bare airframe accepting body-rate and motor commands, with manual rate input.
UCLASS(Transient, NotBlueprintable, NotPlaceable, HideDropdown)
class AUBotAerialTestBareDrone : public AUBotAerialTestDrone
{
    GENERATED_BODY()

public:
    AUBotAerialTestBareDrone(const FObjectInitializer& ObjectInitializer);

    UUBotMultirotorDynamicsComponent* GetDynamics() const;
};

// Test-only flight controller: counts calls and returns a constant command for every motor.
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotAerialTestController : public UUBotFlightController
{
    GENERATED_BODY()

public:
    int32 Calls = 0;
    int32 Resets = 0;
    double LastDeltaSeconds = 0.0;
    double Command = 0.0;
    int32 MotorCount = 4;
    EUBotFlightControlMode LastMode = EUBotFlightControlMode::Motor;

protected:
    virtual void ResetController_Implementation(const FUBotMultirotorState& State, const FUBotMultirotorAirframe& Airframe) override
    {
        ++Resets;
    }

    virtual void ComputeMotorCommands_Implementation(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint, double DeltaSeconds,
        TArray<double>& OutMotorCommands) override
    {
        ++Calls;
        LastDeltaSeconds = DeltaSeconds;
        LastMode = Setpoint.Mode;
        OutMotorCommands.Init(Command, MotorCount);
    }

    virtual bool SupportsControlMode_Implementation(EUBotFlightControlMode Mode) const override
    {
        return true;
    }
};

// Test-only listener for the multirotor events (dynamic delegates need a UObject with UFUNCTIONs).
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotAerialTestListener : public UObject
{
    GENERATED_BODY()

public:
    UFUNCTION()
    void HandleImpact(const FHitResult& Hit, float ImpactSpeedMetersPerSecond)
    {
        ImpactSpeeds.Add(ImpactSpeedMetersPerSecond);
        LastImpactNormal = Hit.Normal;
    }

    UFUNCTION()
    void HandleCrash(const FHitResult& Hit, float ImpactSpeedMetersPerSecond)
    {
        CrashSpeeds.Add(ImpactSpeedMetersPerSecond);
    }

    UFUNCTION()
    void HandleStateChanged(EUBotMultirotorFlightState NewState, EUBotMultirotorFlightState PreviousState)
    {
        States.Add(NewState);
    }

    TArray<float> ImpactSpeeds;
    TArray<float> CrashSpeeds;
    TArray<EUBotMultirotorFlightState> States;
    FVector LastImpactNormal = FVector::ZeroVector;
};
