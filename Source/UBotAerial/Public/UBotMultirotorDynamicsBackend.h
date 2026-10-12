#pragma once

#include "CoreMinimal.h"
#include "UBotAerialTypes.h"
#include "UBotExtensionRegistry.h"
#include "UBotMultirotorDynamics.h"

/** What a dynamics backend sees of the world during one physics step. */
struct FUBotMultirotorEnvironment
{
    /** Gravity, m/s². */
    FVector Gravity = FVector(0.0, 0.0, -9.80665);
    /** Wind velocity at the drone, m/s. */
    FVector Wind = FVector::ZeroVector;
};

/**
 * Computes the motion of one physics multirotor, one fixed step at a time. Experimental: the
 * interface may change between minor versions while it has a single implementation.
 *
 * The multirotor component owns the state, the clock and collisions: it calls Step once per physics
 * step, then sweeps the body from its old to its new position and corrects the state on contact.
 */
class IUBotMultirotorDynamicsBackend
{
public:
    virtual ~IUBotMultirotorDynamicsBackend() = default;

    /** Called when the flight starts or the airframe changes. */
    virtual void Reset(const FUBotMultirotorAirframe& Airframe, const UBot::Multirotor::FRigidBodyState& State) = 0;

    /** Advances InOutState by StepSeconds with normalized motor commands (0..1, motor order). */
    virtual void Step(TConstArrayView<double> MotorCommands, const FUBotMultirotorEnvironment& Environment, double StepSeconds,
        UBot::Multirotor::FRigidBodyState& InOutState) = 0;
};

/**
 * Creates dynamics backends. Register one at the extension point
 * UBot::Aerial::DynamicsBackendExtensionPoint; its extension name is what a multirotor's Dynamics
 * Backend Name selects.
 */
class IUBotMultirotorBackendFactory : public IUBotExtension
{
public:
    /** A new backend for one multirotor using Airframe. */
    virtual TSharedPtr<IUBotMultirotorDynamicsBackend> CreateBackend(const FUBotMultirotorAirframe& Airframe) const = 0;
};

namespace UBot::Aerial
{
    /** The backend factory registered under Name, or nullptr. */
    UBOTAERIAL_API TSharedPtr<IUBotMultirotorBackendFactory> FindBackendFactory(FName Name);
}
