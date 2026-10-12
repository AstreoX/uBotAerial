#pragma once

#include "CoreMinimal.h"
#include "UBotMultirotorDynamicsBackend.h"

/** The plugin's own rigid-body integrator (UBot::Multirotor::StepRigidBody). */
class FUBotBuiltInMultirotorBackend final : public IUBotMultirotorDynamicsBackend
{
public:
    virtual void Reset(const FUBotMultirotorAirframe& InAirframe, const UBot::Multirotor::FRigidBodyState& State) override;
    virtual void Step(TConstArrayView<double> MotorCommands, const FUBotMultirotorEnvironment& Environment, double StepSeconds,
        UBot::Multirotor::FRigidBodyState& InOutState) override;

private:
    FUBotMultirotorAirframe Airframe;
};

/** Factory of the built-in backend, registered by the module as UBot::Aerial::BuiltInBackendName. */
class FUBotBuiltInMultirotorBackendFactory final : public IUBotMultirotorBackendFactory
{
public:
    virtual FName GetExtensionName() const override;
    virtual TSharedPtr<IUBotMultirotorDynamicsBackend> CreateBackend(const FUBotMultirotorAirframe& Airframe) const override;
};
