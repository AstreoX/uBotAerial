#pragma once

#include "UBotAerialTestTypes.h"
#include "UBotSimpleFlightComponent.h"
#include "UBotAerialWindTestTypes.generated.h"

// Observe real fixed-step samples without an optional module in a reflected declaration.
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotAerialTestWindFlight : public UUBotSimpleFlightComponent
{
    GENERATED_BODY()

public:
    FVector SampledWind = FVector::ZeroVector;
    FVector SampledLocationCentimeters = FVector::ZeroVector;
    double SampledTimeSeconds = 0.0;
    int32 Samples = 0;

protected:
    virtual void SimulateStep(double StepSeconds) override
    {
        SampledLocationCentimeters = PositionMeters * 100.0;
        SampledTimeSeconds = GetPhysicsTimeSeconds();
        SampledWind = bEnableWind ? SampleWind() : FVector::ZeroVector;
        ++Samples;
        Super::SimulateStep(StepSeconds);
    }
};

UCLASS(Transient, NotBlueprintable, NotPlaceable, HideDropdown)
class AUBotAerialTestWindDrone : public AUBotAerialTestDrone
{
    GENERATED_BODY()

public:
    AUBotAerialTestWindDrone(const FObjectInitializer& ObjectInitializer)
        : Super(ObjectInitializer.SetDefaultSubobjectClass<UUBotAerialTestWindFlight>(MovementComponentName))
    {
    }

    UUBotAerialTestWindFlight* GetWindFlight() const { return Cast<UUBotAerialTestWindFlight>(Movement); }
};
