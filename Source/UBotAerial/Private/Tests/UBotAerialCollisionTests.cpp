#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "UBotAerialTestTypes.h"
#include "UBotAerialTestWorld.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotRotorComponent.h"
#include "UBotSimpleFlightComponent.h"
#include "UObject/StrongObjectPtr.h"

namespace UBotAerialCollisionTests
{
    using namespace UBotAerialTests;

    TStrongObjectPtr<UUBotAerialTestListener> Listen(UUBotMultirotorMovementComponent& Movement)
    {
        TStrongObjectPtr<UUBotAerialTestListener> Listener(NewObject<UUBotAerialTestListener>());
        Movement.OnImpact.AddDynamic(Listener.Get(), &UUBotAerialTestListener::HandleImpact);
        return Listener;
    }

    // Half height of the drones' collision box, cm.
    constexpr double HalfHeightCm = 9.0;
    constexpr double HalfWidthCm = 29.0;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialCollisionWallStopTest, "UBotAerial.Collision.WallStop", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialCollisionWallStopTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialCollisionTests;
    FScopedAerialTestWorld Scope;
    // Wall face at x = 200 cm.
    Scope.SpawnBox(FVector(250.0, 0.0, 300.0), FVector(50.0, 500.0, 500.0));
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 300.0));
    UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
    if (!TestNotNull(TEXT("Drone"), Flight))
    {
        return false;
    }
    TStrongObjectPtr<UUBotAerialTestListener> Listener = Listen(*Flight);

    SendVelocity(*Flight, FVector(1.0, 0.0, 0.0));
    Scope.RunFor(4.0);

    const FUBotMultirotorState State = Flight->GetMultirotorState();
    const double GapCm = 200.0 - (State.PositionMeters.X * 100.0 + HalfWidthCm);
    TestTrue(FString::Printf(TEXT("Stops at the wall, gap %.4f cm"), GapCm), GapCm >= -1e-3 && GapCm <= 1.0);
    TestTrue(FString::Printf(TEXT("No speed into the wall (%.5f)"), State.LinearVelocityMetersPerSecond.X), State.LinearVelocityMetersPerSecond.X <= 0.06);
    TestEqual(TEXT("One impact event"), Listener->ImpactSpeeds.Num(), 1);
    if (Listener->ImpactSpeeds.Num() > 0)
    {
        TestEqual(TEXT("Impact at the approach speed"), Listener->ImpactSpeeds[0], 1.0f, 0.05f);
        TestTrue(TEXT("Normal points back"), Listener->LastImpactNormal.Equals(FVector(-1.0, 0.0, 0.0), 1e-3));
    }
    TestEqual(TEXT("Did not move sideways"), State.PositionMeters.Y, 0.0, 1e-6);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialCollisionSlideTest, "UBotAerial.Collision.Slide45", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialCollisionSlideTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialCollisionTests;
    FScopedAerialTestWorld Scope;
    // A wall turned 45 degrees ahead: flying forward into it slides toward -Y.
    Scope.SpawnBox(FVector(300.0, 0.0, 300.0), FVector(50.0, 2000.0, 500.0), FRotator(0.0, 45.0, 0.0));
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 300.0));
    UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
    if (!TestNotNull(TEXT("Drone"), Flight))
    {
        return false;
    }
    SendVelocity(*Flight, FVector(2.0, 0.0, 0.0));
    Scope.RunFor(4.0);
    const FUBotMultirotorState State = Flight->GetMultirotorState();
    const FVector Tangent = FVector(1.0, -1.0, 0.0).GetSafeNormal();
    const double Sliding = FVector::DotProduct(State.LinearVelocityMetersPerSecond, Tangent);
    TestTrue(FString::Printf(TEXT("Slides along the wall at least 0.7 m/s (%.3f)"), Sliding), Sliding >= 0.7);
    TestTrue(TEXT("Moved toward -Y"), State.PositionMeters.Y < -1.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialCollisionFloorTest, "UBotAerial.Collision.FloorIsASurface", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialCollisionFloorTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialCollisionTests;
    FScopedAerialTestWorld Scope;
    Scope.SpawnFloor(0.0);
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 100.0));
    UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
    if (!TestNotNull(TEXT("Drone"), Flight))
    {
        return false;
    }
    TStrongObjectPtr<UUBotAerialTestListener> Listener = Listen(*Flight);

    // Descending onto the floor: it stops there, like at any other surface.
    SendVelocity(*Flight, FVector(0.0, 0.0, -1.0));
    Scope.RunFor(3.0);
    const FUBotMultirotorState OnFloor = Flight->GetMultirotorState();
    TestEqual(TEXT("Stopped on the floor"), OnFloor.PositionMeters.Z * 100.0, HalfHeightCm, 0.2);
    TestTrue(FString::Printf(TEXT("No speed into the floor (%.4f)"), OnFloor.LinearVelocityMetersPerSecond.Z), OnFloor.LinearVelocityMetersPerSecond.Z > -0.05);
    TestEqual(TEXT("One impact event"), Listener->ImpactSpeeds.Num(), 1);
    TestTrue(TEXT("Rotors keep spinning"), Drone->Rotors[0]->GetRotorSpeed() > 100.0f);

    // On the floor it still follows commands: forward (while still pressing down) slides along the floor.
    SendVelocity(*Flight, FVector(1.0, 0.0, -0.5));
    Scope.RunFor(2.0);
    const FUBotMultirotorState Slid = Flight->GetMultirotorState();
    TestTrue(FString::Printf(TEXT("Moved forward along the floor (%.3f m)"), Slid.PositionMeters.X - OnFloor.PositionMeters.X),
        Slid.PositionMeters.X - OnFloor.PositionMeters.X >= 1.0);
    TestEqual(TEXT("Still on the floor"), Slid.PositionMeters.Z * 100.0, HalfHeightCm, 0.3);

    // A climb command climbs at once.
    SendVelocity(*Flight, FVector(0.0, 0.0, 1.0));
    Scope.RunFor(0.5);
    TestTrue(FString::Printf(TEXT("Climbing after 0.5 s (%.3f m)"), Flight->GetMultirotorState().PositionMeters.Z),
        Flight->GetMultirotorState().PositionMeters.Z >= HalfHeightCm / 100.0 + 0.15);
    Scope.RunFor(2.0);
    TestTrue(FString::Printf(TEXT("At least 1 m up after 2.5 s (%.3f)"), Flight->GetMultirotorState().PositionMeters.Z),
        Flight->GetMultirotorState().PositionMeters.Z >= 1.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialCollisionStartOnFloorTest, "UBotAerial.Collision.StartOnFloor", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialCollisionStartOnFloorTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialCollisionTests;
    FScopedAerialTestWorld Scope;
    Scope.SpawnFloor(0.0);

    // Placed on the floor, it hovers where it is: no snapping, no sinking.
    const FVector Start(0.0, 0.0, HalfHeightCm + 0.5);
    AUBotAerialTestNavigationDrone* Ground = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(Start);
    if (!TestNotNull(TEXT("Drone on the floor"), Ground))
    {
        return false;
    }
    Scope.RunFor(1.0);
    TestTrue(FString::Printf(TEXT("Stays where it was placed (%s)"), *Ground->GetActorLocation().ToString()), Ground->GetActorLocation().Equals(Start, 1e-3));
    SendVelocity(*Ground->GetSimpleFlight(), FVector(1.0, 0.0, 0.0));
    Scope.RunFor(1.0);
    TestTrue(FString::Printf(TEXT("Moves off at once (%.3f m)"), Ground->GetActorLocation().X / 100.0), Ground->GetActorLocation().X >= 50.0);

    // Reset Flight stops the drone where it is.
    AUBotAerialTestNavigationDrone* High = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(500.0, 0.0, 300.0));
    if (High)
    {
        SendVelocity(*High->GetSimpleFlight(), FVector(2.0, 0.0, 0.0));
        Scope.RunFor(1.0);
        High->GetSimpleFlight()->ResetFlight();
        TestTrue(TEXT("Reset stops the drone"), High->GetSimpleFlight()->GetMultirotorState().LinearVelocityMetersPerSecond.IsZero());
        TestTrue(TEXT("Reset clears the command"), High->GetSimpleFlight()->GetLastVelocityCommand().LinearVelocityMetersPerSecond.IsZero());
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialCollisionExternalTeleportTest, "UBotAerial.Collision.ExternalTeleport", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialCollisionExternalTeleportTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialCollisionTests;
    FScopedAerialTestWorld Scope;
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 300.0));
    UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
    if (!TestNotNull(TEXT("Drone"), Flight))
    {
        return false;
    }
    SendVelocity(*Flight, FVector(1.0, 0.0, 0.0));
    Scope.RunFor(3.0);

    // Moved from outside (an editor, a reset script): the flight model takes over the new pose.
    Drone->SetActorLocation(FVector(0.0, 500.0, 300.0), false, nullptr, ETeleportType::TeleportPhysics);
    Scope.Tick(FrameSeconds);
    const FUBotMultirotorState State = Flight->GetMultirotorState();
    TestEqual(TEXT("New Y adopted"), State.PositionMeters.Y, 5.0, 1e-6);
    TestTrue(TEXT("Kept flying forward"), State.LinearVelocityMetersPerSecond.X > 0.9 && State.PositionMeters.X < 0.1);

    // Teleport Multirotor with Stop Motion: hovers at the new pose.
    Flight->TeleportMultirotor(FVector(100.0, 100.0, 400.0), FRotator(0.0, 90.0, 0.0), true);
    Scope.RunFor(0.5);
    TestTrue(TEXT("Hovers at the teleport target"), Flight->GetMultirotorState().PositionMeters.Equals(FVector(1.0, 1.0, 4.0), 1e-6));
    TestEqual(TEXT("Facing the new heading"), Drone->GetActorRotation().Yaw, 90.0, 1e-3);
    return true;
}

#endif
