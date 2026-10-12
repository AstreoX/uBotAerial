#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "UBotAerialTypes.h"
#include "UBotMultirotorLibrary.generated.h"

class AActor;
class UUBotMultirotorMovementComponent;

/** Helpers for uBot multirotors. */
UCLASS(meta = (DisplayName = "uBot Multirotor Library"))
class UBOTAERIAL_API UUBotMultirotorLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    /** The default airframe with a different arm length and mass (inertia scales with mass and arm length squared). */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Make Quad X Airframe"))
    static FUBotMultirotorAirframe MakeQuadXAirframe(double ArmLengthMeters = 0.225, double MassKilograms = 1.5);

    /** The multirotor movement component of an actor (the actor's first one), or null. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Find Multirotor Movement", DefaultToSelf = "Actor"))
    static UUBotMultirotorMovementComponent* FindMultirotorMovement(const AActor* Actor);
};
