#include "UBotAerialTestTypes.h"

#include "Components/BoxComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/CollisionProfile.h"
#include "UBotIdentityComponent.h"
#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorManualInputComponent.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotRotorComponent.h"
#include "UBotSimpleFlightComponent.h"

const FName AUBotAerialTestDrone::MovementComponentName(TEXT("Movement"));

AUBotAerialTestDrone::AUBotAerialTestDrone(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    PrimaryActorTick.bCanEverTick = false;
    AutoPossessAI = EAutoPossessAI::Disabled;
    bReplicates = false;

    CollisionBox = CreateDefaultSubobject<UBoxComponent>(TEXT("CollisionBox"));
    CollisionBox->InitBoxExtent(FVector(29.0, 29.0, 9.0));
    CollisionBox->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);
    CollisionBox->SetSimulatePhysics(false);
    CollisionBox->SetCanEverAffectNavigation(false);
    CollisionBox->SetMobility(EComponentMobility::Movable);
    RootComponent = CollisionBox;

    VisualRoot = CreateDefaultSubobject<USceneComponent>(TEXT("VisualRoot"));
    VisualRoot->SetupAttachment(CollisionBox);

    Identity = CreateDefaultSubobject<UUBotIdentityComponent>(TEXT("Identity"));
    ManualInput = CreateDefaultSubobject<UUBotMultirotorManualInputComponent>(TEXT("ManualInput"));

    Movement = CreateDefaultSubobject<UUBotMultirotorMovementComponent>(MovementComponentName);
    TArray<FVector> Positions;
    TArray<EUBotRotorSpinDirection> Spins;
    if (Movement)
    {
        Movement->SetUpdatedComponent(CollisionBox);
        Movement->GetRotorLayout(Positions, Spins);
    }

    for (int32 Index = 0; Index < 4; ++Index)
    {
        UUBotRotorComponent* Rotor = CreateDefaultSubobject<UUBotRotorComponent>(*FString::Printf(TEXT("Rotor%d"), Index));
        Rotor->SetupAttachment(VisualRoot);
        Rotor->RotorIndex = Index;
        const FVector Position = Positions.IsValidIndex(Index) ? Positions[Index] * 100.0 : FVector::ZeroVector;
        Rotor->SetRelativeLocation(FVector(Position.X, Position.Y, 4.0));
        Rotors.Add(Rotor);
    }
}

AUBotAerialTestNavigationDrone::AUBotAerialTestNavigationDrone(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UUBotSimpleFlightComponent>(AUBotAerialTestDrone::MovementComponentName))
{
    if (UUBotSimpleFlightComponent* SimpleFlight = GetSimpleFlight())
    {
        SimpleFlight->VisualComponent.ComponentProperty = GET_MEMBER_NAME_CHECKED(AUBotAerialTestDrone, VisualRoot);
    }
}

UUBotSimpleFlightComponent* AUBotAerialTestNavigationDrone::GetSimpleFlight() const
{
    return Cast<UUBotSimpleFlightComponent>(Movement);
}

AUBotAerialTestPhysicsDrone::AUBotAerialTestPhysicsDrone(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UUBotMultirotorDynamicsComponent>(AUBotAerialTestDrone::MovementComponentName))
{
}

UUBotMultirotorDynamicsComponent* AUBotAerialTestPhysicsDrone::GetDynamics() const
{
    return Cast<UUBotMultirotorDynamicsComponent>(Movement);
}

AUBotAerialTestBareDrone::AUBotAerialTestBareDrone(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UUBotMultirotorDynamicsComponent>(AUBotAerialTestDrone::MovementComponentName))
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

UUBotMultirotorDynamicsComponent* AUBotAerialTestBareDrone::GetDynamics() const
{
    return Cast<UUBotMultirotorDynamicsComponent>(Movement);
}
