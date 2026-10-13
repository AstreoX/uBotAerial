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
        Movement.OnCrashed.AddDynamic(Listener.Get(), &UUBotAerialTestListener::HandleCrash);
        Movement.OnFlightStateChanged.AddDynamic(Listener.Get(), &UUBotAerialTestListener::HandleStateChanged);
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
    TestEqual(TEXT("No crash (Simple Flight never crashes by default)"), Listener->CrashSpeeds.Num(), 0);
    TestEqual(TEXT("Still flying"), Flight->GetFlightState(), EUBotMultirotorFlightState::Flying);
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialCollisionLandTakeOffTest, "UBotAerial.Collision.LandAndTakeOff", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialCollisionLandTakeOffTest::RunTest(const FString& Parameters)
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
    TestEqual(TEXT("Starts flying 1 m up"), Flight->GetFlightState(), EUBotMultirotorFlightState::Flying);

    SendVelocity(*Flight, FVector(0.0, 0.0, -1.0));
    double LandedAt = -1.0;
    double FirstContactAt = -1.0;
    Scope.RunFor(3.0, FrameSeconds, [&] {
        if (FirstContactAt < 0.0 && Listener->ImpactSpeeds.Num() > 0)
        {
            FirstContactAt = Scope.Now();
        }
        if (LandedAt < 0.0 && Flight->IsLanded())
        {
            LandedAt = Scope.Now();
        }
    });
    TestTrue(TEXT("Landed"), Flight->IsLanded());
    TestTrue(FString::Printf(TEXT("Landed within 0.1 s of touching (%.3f, %.3f)"), FirstContactAt, LandedAt),
        FirstContactAt > 0.0 && LandedAt >= 0.0 && LandedAt - FirstContactAt <= 0.1);
    const FUBotMultirotorState Landed = Flight->GetMultirotorState();
    TestEqual(TEXT("Resting on the floor"), Landed.PositionMeters.Z * 100.0, HalfHeightCm, 0.2);
    TestTrue(TEXT("Not moving"), Landed.LinearVelocityMetersPerSecond.IsZero());
    TestTrue(TEXT("Level"), Landed.Orientation.Equals(FQuat::Identity, 1e-9));
    TestEqual(TEXT("Rotors stopped"), Drone->Rotors[0]->GetRotorSpeed(), 0.0f);
    TestTrue(TEXT("Specific force +g on the ground"), Landed.SpecificForceMetersPerSecondSquared.Equals(-Scope.Gravity(), 1e-9));

    // Horizontal and descent commands do not move a landed drone.
    SendVelocity(*Flight, FVector(1.0, 0.0, -0.5));
    Scope.RunFor(1.0);
    TestTrue(TEXT("Still landed in place"), Flight->IsLanded() && Flight->GetMultirotorState().PositionMeters == Landed.PositionMeters);

    // A climb command takes off.
    SendVelocity(*Flight, FVector(0.0, 0.0, 1.0));
    double TookOffAt = -1.0;
    const double CommandAt = Scope.Now();
    Scope.RunFor(2.5, FrameSeconds, [&] {
        if (TookOffAt < 0.0 && !Flight->IsLanded())
        {
            TookOffAt = Scope.Now() - CommandAt;
        }
    });
    TestTrue(FString::Printf(TEXT("Took off within 0.5 s (%.3f)"), TookOffAt), TookOffAt >= 0.0 && TookOffAt <= 0.5);
    TestTrue(FString::Printf(TEXT("At least 1 m up after 2.5 s (%.3f)"), Flight->GetMultirotorState().PositionMeters.Z), Flight->GetMultirotorState().PositionMeters.Z >= 1.0);
    TestTrue(TEXT("Rotors spin when flying"), Drone->Rotors[0]->GetRotorSpeed() > 100.0f);
    TestTrue(TEXT("State changes Landed then Flying"), Listener->States.Num() == 2 && Listener->States[0] == EUBotMultirotorFlightState::Landed
        && Listener->States[1] == EUBotMultirotorFlightState::Flying);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialCollisionInitialStateTest, "UBotAerial.Collision.InitialState", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialCollisionInitialStateTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialCollisionTests;
    FScopedAerialTestWorld Scope;
    Scope.SpawnFloor(0.0);

    // 2 cm above the floor: Auto lands and snaps down.
    AUBotAerialTestNavigationDrone* Low = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, HalfHeightCm + 2.0));
    TestTrue(TEXT("Auto starts landed near the ground"), Low && Low->GetSimpleFlight()->IsLanded());
    if (Low)
    {
        TestEqual(TEXT("Snapped onto the floor"), Low->GetActorLocation().Z, HalfHeightCm + 0.1, 0.05);
    }

    // 3 m up: Auto hovers.
    AUBotAerialTestNavigationDrone* High = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(500.0, 0.0, 300.0));
    TestTrue(TEXT("Auto hovers in the air"), High && High->GetSimpleFlight()->GetFlightState() == EUBotMultirotorFlightState::Flying);

    // Hovering on the ground stays flying; Landed in the air stays landed until a climb.
    AUBotAerialTestNavigationDrone* Hovering = Scope.Spawn<AUBotAerialTestNavigationDrone>(FVector(1000.0, 0.0, HalfHeightCm + 2.0));
    Hovering->GetSimpleFlight()->InitialState = EUBotMultirotorInitialState::Hovering;
    Hovering->DispatchBeginPlay();
    TestEqual(TEXT("Hovering forced"), Hovering->GetSimpleFlight()->GetFlightState(), EUBotMultirotorFlightState::Flying);
    AUBotAerialTestNavigationDrone* Parked = Scope.Spawn<AUBotAerialTestNavigationDrone>(FVector(1500.0, 0.0, 300.0));
    Parked->GetSimpleFlight()->InitialState = EUBotMultirotorInitialState::Landed;
    Parked->DispatchBeginPlay();
    Scope.RunFor(0.5);
    TestTrue(TEXT("Landed forced"), Parked->GetSimpleFlight()->IsLanded());
    TestEqual(TEXT("Landed does not fall"), Parked->GetActorLocation().Z, 300.0, 1e-6);

    // Reset Flight starts again from the current pose.
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
