#pragma once

#include "CoreMinimal.h"
#include "Engine/World.h"
#include "UBotWindProvider.h"
#include "UBotAerialWindTestTypes.generated.h"

// Aerial tests supply wind through the Core contract, without an environment plugin.
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotAerialTestWindProvider : public UObject, public IUBotWindProvider
{
    GENERATED_BODY()

public:
    FVector Velocity = FVector::ZeroVector;
    mutable FVector LastLocationCentimeters = FVector::ZeroVector;
    mutable double LastTimeSeconds = 0.0;
    mutable int32 Queries = 0;

    virtual UWorld* GetWorld() const override { return GetTypedOuter<UWorld>(); }

    virtual FVector GetWindVelocity(const FVector& LocationCentimeters, double TimeSeconds) const override
    {
        LastLocationCentimeters = LocationCentimeters;
        LastTimeSeconds = TimeSeconds;
        ++Queries;
        return Velocity;
    }
};
