#pragma once

#include "CoreMinimal.h"
#include "Engine/HitResult.h"
#include "UBotAerialTypes.h"
#include "UBotFlightController.h"
#include "UObject/Object.h"
#include "UBotAerialTestTypes.generated.h"

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
