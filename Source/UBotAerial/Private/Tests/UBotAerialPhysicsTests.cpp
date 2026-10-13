#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "UBotAerialTestTypes.h"
#include "UBotAerialTestWorld.h"
#include "UBotCascadedFlightController.h"
#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorMath.h"
#include "UBotMultirotorMovementComponent.h"
#include "UObject/StrongObjectPtr.h"

namespace UBotAerialPhysicsTests
{
    using namespace UBotAerialTests;

    TStrongObjectPtr<UUBotAerialTestListener> Listen(UUBotMultirotorMovementComponent& Movement)
    {
        TStrongObjectPtr<UUBotAerialTestListener> Listener(NewObject<UUBotAerialTestListener>());
        Movement.OnImpact.AddDynamic(Listener.Get(), &UUBotAerialTestListener::HandleImpact);
        return Listener;
    }

    FUBotMultirotorState RunPhysicsScenario(float DeltaSeconds)
    {
        FScopedAerialTestWorld Scope;
        Scope.SpawnFloor(0.0);
        Scope.SpawnBox(FVector(600.0, 0.0, 300.0), FVector(50.0, 500.0, 500.0), FRotator(0.0, 30.0, 0.0));
        AUBotAerialTestPhysicsDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestPhysicsDrone>(FVector(0.0, 0.0, 200.0));
        UUBotMultirotorDynamicsComponent* Dynamics = Drone->GetDynamics();
        SendVelocity(*Dynamics, FVector(2.0, 0.0, 0.5), 0.4);
        Scope.RunFor(3.0, DeltaSeconds);
        FUBotAttitudeCommand Attitude;
        Attitude.Orientation = FQuat(FVector::ForwardVector, FMath::DegreesToRadians(8.0));
        IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Dynamics, Attitude);
        Scope.RunFor(1.0, DeltaSeconds);
        IUBotVelocityCommandable::Execute_StopMotion(Dynamics);
        Scope.RunFor(1.0, DeltaSeconds);
        return Dynamics->GetMultirotorState();
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialPhysicsHardImpactTest, "UBotAerial.Collision.HardImpact", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialPhysicsHardImpactTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialPhysicsTests;
    FScopedAerialTestWorld Scope;
    Scope.SpawnFloor(0.0);
    // Wall face 8 m ahead.
    Scope.SpawnBox(FVector(850.0, 0.0, 500.0), FVector(50.0, 1000.0, 1000.0));
    AUBotAerialTestPhysicsDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestPhysicsDrone>(FVector(0.0, 0.0, 300.0));
    UUBotMultirotorDynamicsComponent* Dynamics = Drone ? Drone->GetDynamics() : nullptr;
    if (!TestNotNull(TEXT("Physics drone"), Dynamics))
    {
        return false;
    }
    TStrongObjectPtr<UUBotAerialTestListener> Listener = Listen(*Dynamics);
    SendVelocity(*Dynamics, FVector(6.0, 0.0, 0.0));
    Scope.RunFor(4.0);
    TestTrue(TEXT("Hit the wall"), Listener->ImpactSpeeds.Num() >= 1);
    if (Listener->ImpactSpeeds.Num() > 0)
    {
        TestTrue(FString::Printf(TEXT("Fast impact (%.3f m/s)"), Listener->ImpactSpeeds[0]), Listener->ImpactSpeeds[0] >= 3.0f);
    }
    const FUBotMultirotorState AtWall = Dynamics->GetMultirotorState();
    TestTrue(FString::Printf(TEXT("Stopped at the wall (%.3f m)"), AtWall.PositionMeters.X), AtWall.PositionMeters.X < 8.0 && AtWall.PositionMeters.X > 7.0);

    // A hard hit changes nothing: the drone keeps flying and takes commands.
    TestTrue(TEXT("Ready after the hit"), IUBotActuator::Execute_IsActuatorReady(Dynamics));
    TestTrue(TEXT("Commands accepted after the hit"), SendVelocity(*Dynamics, FVector(-1.0, 0.0, 0.0)));
    Scope.RunFor(3.0);
    const FUBotMultirotorState After = Dynamics->GetMultirotorState();
    TestTrue(FString::Printf(TEXT("Flew back from the wall (%.3f m)"), AtWall.PositionMeters.X - After.PositionMeters.X),
        AtWall.PositionMeters.X - After.PositionMeters.X >= 1.0);
    TestTrue(FString::Printf(TEXT("Still in the air (%.3f m)"), After.PositionMeters.Z), After.PositionMeters.Z > 1.5);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialPhysicsFloorTest, "UBotAerial.Collision.PhysicsFloor", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialPhysicsFloorTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialPhysicsTests;
    FScopedAerialTestWorld Scope;
    Scope.SpawnFloor(0.0);
    AUBotAerialTestPhysicsDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestPhysicsDrone>(FVector(0.0, 0.0, 200.0));
    UUBotMultirotorDynamicsComponent* Dynamics = Drone ? Drone->GetDynamics() : nullptr;
    if (!TestNotNull(TEXT("Physics drone"), Dynamics))
    {
        return false;
    }
    TStrongObjectPtr<UUBotAerialTestListener> Listener = Listen(*Dynamics);

    // Descending onto the floor and holding the descent: the floor stops it, the motors stay at hover.
    SendVelocity(*Dynamics, FVector(0.0, 0.0, -1.0));
    Scope.RunFor(5.0);
    const FUBotMultirotorState OnFloor = Dynamics->GetMultirotorState();
    TestEqual(TEXT("Stopped on the floor"), OnFloor.PositionMeters.Z * 100.0, 9.0, 0.5);
    TestTrue(FString::Printf(TEXT("At rest (%.4f)"), OnFloor.LinearVelocityMetersPerSecond.Size()), OnFloor.LinearVelocityMetersPerSecond.Size() < 0.05);
    TestTrue(FString::Printf(TEXT("Level (%.3f deg)"), FMath::RadiansToDegrees(UBot::Multirotor::GetTilt(OnFloor.Orientation))),
        FMath::RadiansToDegrees(UBot::Multirotor::GetTilt(OnFloor.Orientation)) < 1.0);
    TestTrue(FString::Printf(TEXT("Motors near hover (%.1f rad/s)"), OnFloor.RotorSpeedsRadiansPerSecond[0]), OnFloor.RotorSpeedsRadiansPerSecond[0] > 600.0);

    // Forward while still pressing down: it slides along the floor at the commanded speed.
    SendVelocity(*Dynamics, FVector(1.0, 0.0, -1.0));
    Scope.RunFor(3.0);
    const FUBotMultirotorState Slid = Dynamics->GetMultirotorState();
    TestTrue(FString::Printf(TEXT("Moved forward along the floor (%.3f m)"), Slid.PositionMeters.X - OnFloor.PositionMeters.X),
        Slid.PositionMeters.X - OnFloor.PositionMeters.X >= 1.5);
    TestTrue(FString::Printf(TEXT("Near 1 m/s (%.3f)"), Slid.LinearVelocityMetersPerSecond.X), FMath::Abs(Slid.LinearVelocityMetersPerSecond.X - 1.0) < 0.15);
    TestTrue(FString::Printf(TEXT("Still on the floor (%.3f cm)"), Slid.PositionMeters.Z * 100.0), Slid.PositionMeters.Z * 100.0 < 12.0);

    // Climbing right after pressing down: no delay from the time spent on the floor.
    SendVelocity(*Dynamics, FVector(0.0, 0.0, 1.0));
    Scope.RunFor(0.5);
    TestTrue(FString::Printf(TEXT("Climbing after 0.5 s (%.3f m)"), Dynamics->GetMultirotorState().PositionMeters.Z),
        Dynamics->GetMultirotorState().PositionMeters.Z >= 0.25);
    Scope.RunFor(2.0);
    TestTrue(FString::Printf(TEXT("At least 1 m up after 2.5 s (%.3f)"), Dynamics->GetMultirotorState().PositionMeters.Z),
        Dynamics->GetMultirotorState().PositionMeters.Z >= 1.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialPhysicsDeterminismTest, "UBotAerial.Determinism.PhysicsTwoRuns", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialPhysicsDeterminismTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialPhysicsTests;
    const FUBotMultirotorState First = RunPhysicsScenario(FrameSeconds);
    const FUBotMultirotorState Second = RunPhysicsScenario(FrameSeconds);
    TestTrue(FString::Printf(TEXT("Same position (%s vs %s)"), *First.PositionMeters.ToString(), *Second.PositionMeters.ToString()),
        First.PositionMeters == Second.PositionMeters);
    TestTrue(TEXT("Same velocity"), First.LinearVelocityMetersPerSecond == Second.LinearVelocityMetersPerSecond);
    TestTrue(TEXT("Same attitude"), First.Orientation.X == Second.Orientation.X && First.Orientation.Y == Second.Orientation.Y
        && First.Orientation.Z == Second.Orientation.Z && First.Orientation.W == Second.Orientation.W);
    TestTrue(TEXT("Same rotor speeds"), First.RotorSpeedsRadiansPerSecond == Second.RotorSpeedsRadiansPerSecond);

    // Frame rate does not matter for a constant command: 3000 steps at 1 kHz from 60 Hz and 24 Hz frames.
    const auto Run = [](float DeltaSeconds)
    {
        FScopedAerialTestWorld Scope;
        AUBotAerialTestPhysicsDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestPhysicsDrone>(FVector(0.0, 0.0, 1000.0));
        SendVelocity(*Drone->GetDynamics(), FVector(1.5, -0.5, 0.3), 0.2);
        Scope.RunFor(3.0, DeltaSeconds);
        return Drone->GetDynamics()->GetMultirotorState();
    };
    const FUBotMultirotorState At60 = Run(1.0f / 60.0f);
    const FUBotMultirotorState At24 = Run(1.0f / 24.0f);
    TestTrue(FString::Printf(TEXT("Same state after 3 s (%s vs %s)"), *At60.PositionMeters.ToString(), *At24.PositionMeters.ToString()),
        At60.PositionMeters == At24.PositionMeters && At60.LinearVelocityMetersPerSecond == At24.LinearVelocityMetersPerSecond);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDynamicsComponentDefaultsTest, "UBotAerial.Dynamics.ComponentDefaults", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDynamicsComponentDefaultsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialTests;
    FScopedAerialTestWorld Scope;
    AUBotAerialTestPhysicsDrone* Physics = Scope.SpawnAndBegin<AUBotAerialTestPhysicsDrone>(FVector(0.0, 0.0, 500.0));
    UUBotMultirotorDynamicsComponent* Dynamics = Physics ? Physics->GetDynamics() : nullptr;
    if (!TestNotNull(TEXT("Movement is Multirotor Dynamics"), Dynamics))
    {
        return false;
    }
    TestTrue(TEXT("Default controller is the cascaded one"), Cast<UUBotCascadedFlightController>(Dynamics->FlightController) != nullptr);
    TestEqual(TEXT("1 kHz physics"), Dynamics->PhysicsRateHz, 1000.0f);
    TestTrue(TEXT("Velocity commands accepted by default"), Dynamics->bAcceptVelocityCommands);
    TestTrue(TEXT("Attitude commands accepted by default"), Dynamics->bAcceptAttitudeCommands);
    TestTrue(TEXT("Body-rate commands accepted by default"), Dynamics->bAcceptBodyRateCommands);
    TestTrue(TEXT("Motor commands accepted by default"), Dynamics->bAcceptMotorCommands);
    return true;
}

#endif
