#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Camera/CameraComponent.h"
#include "Components/BoxComponent.h"
#include "UBotAerialTestWorld.h"
#include "UBotIdentityComponent.h"
#include "UBotNavigationDrone.h"
#include "UBotRotorComponent.h"
#include "UBotSimpleFlightComponent.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialPawnNavigationTest, "UBotAerial.Pawn.Navigation.Components", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialPawnNavigationTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialTests;
    FScopedAerialTestWorld Scope;
    AUBotNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotNavigationDrone>(FVector(0.0, 0.0, 500.0));
    if (!TestNotNull(TEXT("Drone"), Drone))
    {
        return false;
    }

    TestTrue(TEXT("Collision box is the root"), Drone->GetRootComponent() == Drone->CollisionBox);
    TestTrue(TEXT("Box covers the rotor disks"), Drone->CollisionBox->GetUnscaledBoxExtent().Equals(FVector(29.0, 29.0, 9.0)));
    TestEqual(TEXT("Pawn collision profile"), Drone->CollisionBox->GetCollisionProfileName(), FName(TEXT("Pawn")));
    TestTrue(TEXT("Movement is Simple Flight"), Drone->GetSimpleFlight() != nullptr && Drone->GetMultirotorMovement() == Drone->GetSimpleFlight());
    TestTrue(TEXT("Movement moves the root"), Drone->GetMultirotorMovement()->UpdatedComponent == Drone->CollisionBox);
    TestTrue(TEXT("Velocity commandable found on the actor"), UBot::Actuation::FindVelocityCommandable(Drone) == Drone->GetMultirotorMovement());
    TestNotNull(TEXT("Identity"), Drone->Identity.Get());
    TestTrue(TEXT("Visual component resolves to the visual root"),
        Drone->GetSimpleFlight()->VisualComponent.GetComponent(Drone) == Drone->VisualRoot);
    TestTrue(TEXT("Body mesh loaded"), Drone->Body->GetStaticMesh() != nullptr);
    TestFalse(TEXT("Camera inactive without a local player"), Drone->Camera->IsActive());
    TestEqual(TEXT("Actuator type"), IUBotActuator::Execute_GetActuatorType(Drone->GetMultirotorMovement()), FName(TEXT("Multirotor")));
    TestTrue(TEXT("Actuator ready"), IUBotActuator::Execute_IsActuatorReady(Drone->GetMultirotorMovement()));

    if (!TestEqual(TEXT("Four rotors"), Drone->Rotors.Num(), 4))
    {
        return false;
    }
    TArray<FVector> Positions;
    TArray<EUBotRotorSpinDirection> Spins;
    Drone->GetMultirotorMovement()->GetRotorLayout(Positions, Spins);
    for (int32 Index = 0; Index < 4; ++Index)
    {
        const UUBotRotorComponent* Rotor = Drone->Rotors[Index];
        const FVector Expected(Positions[Index].X * 100.0, Positions[Index].Y * 100.0, Rotor->GetRelativeLocation().Z);
        TestTrue(FString::Printf(TEXT("Rotor %d at its airframe position"), Index), Rotor->GetRelativeLocation().Equals(Expected, 0.1));
        TestEqual(FString::Printf(TEXT("Rotor %d index"), Index), Rotor->RotorIndex, Index);
        TestEqual(FString::Printf(TEXT("Rotor %d spin"), Index), Rotor->SpinDirection, Spins[Index]);
        TestTrue(FString::Printf(TEXT("Rotor %d has no collision"), Index), Rotor->GetCollisionEnabled() == ECollisionEnabled::NoCollision);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialPawnRotorSpinTest, "UBotAerial.Pawn.RotorSpinDirection", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialPawnRotorSpinTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialTests;
    FScopedAerialTestWorld Scope;
    AUBotNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotNavigationDrone>(FVector(0.0, 0.0, 500.0));
    if (!TestNotNull(TEXT("Drone"), Drone))
    {
        return false;
    }
    Scope.Tick(FrameSeconds);
    const double Start0 = Drone->Rotors[0]->GetRelativeRotation().Yaw;
    const double Start2 = Drone->Rotors[2]->GetRelativeRotation().Yaw;
    Scope.Tick(FrameSeconds);
    const double Turn0 = FMath::FindDeltaAngleDegrees(Start0, Drone->Rotors[0]->GetRelativeRotation().Yaw);
    const double Turn2 = FMath::FindDeltaAngleDegrees(Start2, Drone->Rotors[2]->GetRelativeRotation().Yaw);
    TestTrue(FString::Printf(TEXT("Counter-clockwise rotor turns left (%.2f)"), Turn0), Turn0 < -0.5);
    TestTrue(FString::Printf(TEXT("Clockwise rotor turns right (%.2f)"), Turn2), Turn2 > 0.5);
    return true;
}

#endif
