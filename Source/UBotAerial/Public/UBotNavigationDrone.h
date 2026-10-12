#pragma once

#include "CoreMinimal.h"
#include "UBotMultirotorPawn.h"
#include "UBotNavigationDrone.generated.h"

class UUBotSimpleFlightComponent;

/**
 * Quadrotor for navigation research: simplified flight control (uBot Simple Flight). Send velocity
 * commands (for example through uBot ROS Velocity Command) and it flies them smoothly, stops within a
 * bounded distance and keeps attached sensors level.
 */
UCLASS(meta = (DisplayName = "uBot Navigation Drone"))
class UBOTAERIAL_API AUBotNavigationDrone : public AUBotMultirotorPawn
{
    GENERATED_BODY()

public:
    AUBotNavigationDrone(const FObjectInitializer& ObjectInitializer);

    /** The simplified flight model of this drone. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Simple Flight"))
    UUBotSimpleFlightComponent* GetSimpleFlight() const;
};
