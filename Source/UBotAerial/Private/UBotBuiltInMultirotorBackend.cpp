#include "UBotBuiltInMultirotorBackend.h"

#include "UBotAerial.h"

void FUBotBuiltInMultirotorBackend::Reset(const FUBotMultirotorAirframe& InAirframe, const UBot::Multirotor::FRigidBodyState& State)
{
    Airframe = InAirframe;
}

void FUBotBuiltInMultirotorBackend::Step(TConstArrayView<double> MotorCommands, const FUBotMultirotorEnvironment& Environment, double StepSeconds,
    UBot::Multirotor::FRigidBodyState& InOutState)
{
    UBot::Multirotor::StepRigidBody(Airframe, MotorCommands, Environment.Gravity, Environment.Wind, StepSeconds, InOutState);
}

FName FUBotBuiltInMultirotorBackendFactory::GetExtensionName() const
{
    return UBot::Aerial::BuiltInBackendName;
}

TSharedPtr<IUBotMultirotorDynamicsBackend> FUBotBuiltInMultirotorBackendFactory::CreateBackend(const FUBotMultirotorAirframe& Airframe) const
{
    return MakeShared<FUBotBuiltInMultirotorBackend>();
}

namespace UBot::Aerial
{
    TSharedPtr<IUBotMultirotorBackendFactory> FindBackendFactory(FName Name)
    {
        for (const TSharedRef<IUBotMultirotorBackendFactory>& Factory
            : FUBotExtensionRegistry::Get().GetExtensionsAs<IUBotMultirotorBackendFactory>(DynamicsBackendExtensionPoint))
        {
            if (Factory->GetExtensionName() == Name)
            {
                return Factory;
            }
        }
        return nullptr;
    }
}
