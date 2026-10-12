#pragma once

#include "CoreMinimal.h"
#include "Engine/HitResult.h"
#include "UBotAerialTypes.h"
#include "UObject/Object.h"
#include "UBotAerialTestTypes.generated.h"

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
