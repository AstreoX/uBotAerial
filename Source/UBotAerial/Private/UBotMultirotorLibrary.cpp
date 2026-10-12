#include "UBotMultirotorLibrary.h"

#include "GameFramework/Actor.h"
#include "UBotMultirotorDynamics.h"
#include "UBotMultirotorMovementComponent.h"

FUBotMultirotorAirframe UUBotMultirotorLibrary::MakeQuadXAirframe(double ArmLengthMeters, double MassKilograms)
{
    FUBotMultirotorAirframe Airframe;
    const double Arm = FMath::Max(ArmLengthMeters, 0.01);
    const double Mass = FMath::Max(MassKilograms, 0.01);
    const double Scale = (Mass / Airframe.MassKilograms) * FMath::Square(Arm / 0.225);
    Airframe.MassKilograms = Mass;
    Airframe.InertiaKilogramSquareMeters *= Scale;
    Airframe.Rotors = UBot::Multirotor::MakeQuadXRotors(Arm);
    return Airframe;
}

UUBotMultirotorMovementComponent* UUBotMultirotorLibrary::FindMultirotorMovement(const AActor* Actor)
{
    return Actor ? Actor->FindComponentByClass<UUBotMultirotorMovementComponent>() : nullptr;
}
