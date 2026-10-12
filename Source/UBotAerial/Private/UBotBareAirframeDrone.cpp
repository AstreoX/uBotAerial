#include "UBotBareAirframeDrone.h"

#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorManualInputComponent.h"

AUBotBareAirframeDrone::AUBotBareAirframeDrone(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UUBotMultirotorDynamicsComponent>(AUBotMultirotorPawn::MovementComponentName))
{
    if (UUBotMultirotorDynamicsComponent* Dynamics = GetDynamics())
    {
        Dynamics->bAcceptVelocityCommands = false;
        Dynamics->bAcceptAttitudeCommands = false;
        Dynamics->bAcceptBodyRateCommands = true;
        Dynamics->bAcceptMotorCommands = true;
    }
    if (ManualInput)
    {
        ManualInput->FlightMode = EUBotManualFlightMode::Rate;
    }
}

UUBotMultirotorDynamicsComponent* AUBotBareAirframeDrone::GetDynamics() const
{
    return Cast<UUBotMultirotorDynamicsComponent>(Movement);
}
