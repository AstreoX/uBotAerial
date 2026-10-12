#include "UBotPhysicsDrone.h"

#include "UBotMultirotorDynamicsComponent.h"

AUBotPhysicsDrone::AUBotPhysicsDrone(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UUBotMultirotorDynamicsComponent>(AUBotMultirotorPawn::MovementComponentName))
{
}

UUBotMultirotorDynamicsComponent* AUBotPhysicsDrone::GetDynamics() const
{
    return Cast<UUBotMultirotorDynamicsComponent>(Movement);
}
