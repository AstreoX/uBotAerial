#pragma once

#include "CoreMinimal.h"
#include "UBotMultirotorPawn.h"
#include "UBotPhysicsDrone.generated.h"

class UUBotMultirotorDynamicsComponent;

/**
 * Quadrotor with rigid-body dynamics and the built-in cascaded flight controller (uBot Multirotor
 * Dynamics). Fly it with velocity commands (for example uBot ROS Velocity Command) or attitude and
 * body-rate commands; motor commands drive the rotors directly.
 */
UCLASS(meta = (DisplayName = "uBot Physics Drone"))
class UBOTAERIAL_API AUBotPhysicsDrone : public AUBotMultirotorPawn
{
    GENERATED_BODY()

public:
    AUBotPhysicsDrone(const FObjectInitializer& ObjectInitializer);

    /** The dynamics of this drone. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Dynamics"))
    UUBotMultirotorDynamicsComponent* GetDynamics() const;
};
