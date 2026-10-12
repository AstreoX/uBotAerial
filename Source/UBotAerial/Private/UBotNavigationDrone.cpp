#include "UBotNavigationDrone.h"

#include "UBotSimpleFlightComponent.h"

AUBotNavigationDrone::AUBotNavigationDrone(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UUBotSimpleFlightComponent>(AUBotMultirotorPawn::MovementComponentName))
{
    if (UUBotSimpleFlightComponent* SimpleFlight = GetSimpleFlight())
    {
        SimpleFlight->VisualComponent.ComponentProperty = GET_MEMBER_NAME_CHECKED(AUBotMultirotorPawn, VisualRoot);
    }
}

UUBotSimpleFlightComponent* AUBotNavigationDrone::GetSimpleFlight() const
{
    return Cast<UUBotSimpleFlightComponent>(Movement);
}
