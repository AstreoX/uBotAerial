#include "UBotFlightController.h"

#include "Engine/World.h"

UWorld* UUBotFlightController::GetWorld() const
{
    // The template in the class defaults has no world; an instance lives inside its multirotor component.
    if (HasAnyFlags(RF_ClassDefaultObject | RF_ArchetypeObject))
    {
        return nullptr;
    }
    const UObject* Outer = GetOuter();
    return Outer ? Outer->GetWorld() : nullptr;
}

void UUBotFlightController::Compute(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint, double DeltaSeconds,
    TArray<double>& OutMotorCommands)
{
    if (GetClass()->HasAnyClassFlags(CLASS_Native))
    {
        ComputeMotorCommands_Implementation(State, Setpoint, DeltaSeconds, OutMotorCommands);
    }
    else
    {
        ComputeMotorCommands(State, Setpoint, DeltaSeconds, OutMotorCommands);
    }
}

void UUBotFlightController::Reset(const FUBotMultirotorState& State, const FUBotMultirotorAirframe& Airframe)
{
    if (GetClass()->HasAnyClassFlags(CLASS_Native))
    {
        ResetController_Implementation(State, Airframe);
    }
    else
    {
        ResetController(State, Airframe);
    }
}

void UUBotFlightController::ComputeMotorCommands_Implementation(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint,
    double DeltaSeconds, TArray<double>& OutMotorCommands)
{
    OutMotorCommands.Reset();
}
