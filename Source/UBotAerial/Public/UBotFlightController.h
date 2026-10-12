#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "UBotAerialTypes.h"
#include "UBotFlightController.generated.h"

/**
 * Flight controller of a physics multirotor. The multirotor calls Compute Motor Commands once per
 * physics step (or every Controller Rate Divider steps) with the ground-truth state and the setpoint
 * of the active control mode, and applies the normalized motor commands it returns.
 *
 * Subclass it in C++ (override the _Implementation functions) to bring your own controller. A
 * Blueprint subclass works too, but runs at the physics rate (1 kHz by default) through the
 * Blueprint VM; raise Controller Rate Divider for it.
 */
UCLASS(Abstract, Blueprintable, EditInlineNew, DefaultToInstanced, CollapseCategories, meta = (DisplayName = "uBot Flight Controller"))
class UBOTAERIAL_API UUBotFlightController : public UObject
{
    GENERATED_BODY()

public:
    /** Clears the controller's memory (integrators, setpoints) and takes the airframe it flies. */
    UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Reset Controller"))
    void ResetController(const FUBotMultirotorState& State, const FUBotMultirotorAirframe& Airframe);

    /**
     * Normalized motor commands (0..1, one per rotor in motor order) for this step. Delta Seconds is the
     * time until the next call.
     */
    UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Compute Motor Commands"))
    void ComputeMotorCommands(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint, double DeltaSeconds, TArray<double>& OutMotorCommands);

    /** Whether this controller flies setpoints of Mode. Motor mode never reaches a controller. */
    UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Supports Control Mode"))
    bool SupportsControlMode(EUBotFlightControlMode Mode) const;

    virtual UWorld* GetWorld() const override;

    /** Calls Compute Motor Commands, directly for native classes and through the Blueprint VM otherwise. */
    void Compute(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint, double DeltaSeconds, TArray<double>& OutMotorCommands);

    /** Calls Reset Controller the same way. */
    void Reset(const FUBotMultirotorState& State, const FUBotMultirotorAirframe& Airframe);

protected:
    virtual void ResetController_Implementation(const FUBotMultirotorState& State, const FUBotMultirotorAirframe& Airframe) {}
    virtual void ComputeMotorCommands_Implementation(const FUBotMultirotorState& State, const FUBotFlightSetpoint& Setpoint, double DeltaSeconds,
        TArray<double>& OutMotorCommands);
    virtual bool SupportsControlMode_Implementation(EUBotFlightControlMode Mode) const { return false; }
};
