#pragma once

#include "CoreMinimal.h"
#include "UBotMultirotorPawn.h"
#include "UBotBareAirframeDrone.generated.h"

class UUBotMultirotorDynamicsComponent;

/**
 * Quadrotor for flight-control research: the same rigid-body dynamics as the physics drone, but it
 * only accepts body-rate and motor commands, so the controller is yours (in another program, or a
 * uBot Flight Controller subclass set on its dynamics). Until the first command and after commands
 * stop it hovers with the built-in controller.
 */
UCLASS(meta = (DisplayName = "uBot Bare Airframe Drone"))
class UBOTAERIAL_API AUBotBareAirframeDrone : public AUBotMultirotorPawn
{
    GENERATED_BODY()

public:
    AUBotBareAirframeDrone(const FObjectInitializer& ObjectInitializer);

    /** The dynamics of this drone. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Dynamics"))
    UUBotMultirotorDynamicsComponent* GetDynamics() const;
};
