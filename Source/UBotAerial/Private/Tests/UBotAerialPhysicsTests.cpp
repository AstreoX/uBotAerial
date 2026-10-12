#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "UBotAerialTestTypes.h"
#include "UBotAerialTestWorld.h"
#include "UBotBareAirframeDrone.h"
#include "UBotCascadedFlightController.h"
#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorMath.h"
#include "UBotPhysicsDrone.h"
#include "UBotRotorComponent.h"
#include "UObject/StrongObjectPtr.h"

namespace UBotAerialPhysicsTests
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

    FUBotMultirotorState RunPhysicsScenario(float DeltaSeconds)
    {
        FScopedAerialTestWorld Scope;
        Scope.SpawnFloor(0.0);
        Scope.SpawnBox(FVector(600.0, 0.0, 300.0), FVector(50.0, 500.0, 500.0), FRotator(0.0, 30.0, 0.0));
        AUBotPhysicsDrone* Drone = Scope.SpawnAndBegin<AUBotPhysicsDrone>(FVector(0.0, 0.0, 200.0));
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialPhysicsCrashTest, "UBotAerial.Collision.Crash", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialPhysicsCrashTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialPhysicsTests;
    FScopedAerialTestWorld Scope;
    Scope.SpawnFloor(0.0);
    // Wall face 8 m ahead.
    Scope.SpawnBox(FVector(850.0, 0.0, 500.0), FVector(50.0, 1000.0, 1000.0));
    AUBotPhysicsDrone* Drone = Scope.SpawnAndBegin<AUBotPhysicsDrone>(FVector(0.0, 0.0, 300.0));
    UUBotMultirotorDynamicsComponent* Dynamics = Drone ? Drone->GetDynamics() : nullptr;
    if (!TestNotNull(TEXT("Physics drone"), Dynamics))
    {
        return false;
    }
    TStrongObjectPtr<UUBotAerialTestListener> Listener = Listen(*Dynamics);
    SendVelocity(*Dynamics, FVector(6.0, 0.0, 0.0));
    double CrashedAt = -1.0;
    Scope.RunFor(4.0, FrameSeconds, [&] {
        if (CrashedAt < 0.0 && Dynamics->GetFlightState() == EUBotMultirotorFlightState::Crashed)
        {
            CrashedAt = Scope.Now();
        }
    });
    TestTrue(TEXT("Crashed into the wall"), CrashedAt > 0.0);
    TestEqual(TEXT("On Crashed fired once"), Listener->CrashSpeeds.Num(), 1);
    if (Listener->CrashSpeeds.Num() > 0)
    {
        TestTrue(FString::Printf(TEXT("Crash speed at least 3 m/s (%.3f)"), Listener->CrashSpeeds[0]), Listener->CrashSpeeds[0] >= 3.0f);
    }
    TestFalse(TEXT("Commands rejected while crashed"), SendVelocity(*Dynamics, FVector(1.0, 0.0, 0.0)));
    TestFalse(TEXT("Not ready while crashed"), IUBotActuator::Execute_IsActuatorReady(Dynamics));

    Scope.RunFor(3.0);
    const FUBotMultirotorState Rest = Dynamics->GetMultirotorState();
    for (const double Speed : Rest.RotorSpeedsRadiansPerSecond)
    {
        TestTrue(FString::Printf(TEXT("Motors stopped (%.3f)"), Speed), Speed < 1.0);
    }
    TestTrue(FString::Printf(TEXT("Fell to the floor (%.3f m)"), Rest.PositionMeters.Z), Rest.PositionMeters.Z < 0.5);
    TestTrue(FString::Printf(TEXT("At rest (v %s, p %s, tilt %.1f deg)"), *Rest.LinearVelocityMetersPerSecond.ToString(), *Rest.PositionMeters.ToString(),
        FMath::RadiansToDegrees(UBot::Multirotor::GetTilt(Rest.Orientation))), Rest.LinearVelocityMetersPerSecond.Size() < 0.05);
    TestEqual(TEXT("Still crashed"), Dynamics->GetFlightState(), EUBotMultirotorFlightState::Crashed);

    Dynamics->ResetFlight();
    TestTrue(TEXT("Reset Flight recovers, landed on the floor"), Dynamics->IsLanded());
    TestTrue(TEXT("Commands accepted again"), SendVelocity(*Dynamics, FVector(0.0, 0.0, 1.0)));
    Scope.RunFor(2.0);
    TestEqual(TEXT("Flies again"), Dynamics->GetFlightState(), EUBotMultirotorFlightState::Flying);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialPhysicsLandTakeOffTest, "UBotAerial.Collision.PhysicsLandAndTakeOff", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialPhysicsLandTakeOffTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialPhysicsTests;
    FScopedAerialTestWorld Scope;
    Scope.SpawnFloor(0.0);
    AUBotPhysicsDrone* Drone = Scope.SpawnAndBegin<AUBotPhysicsDrone>(FVector(0.0, 0.0, 200.0));
    UUBotMultirotorDynamicsComponent* Dynamics = Drone ? Drone->GetDynamics() : nullptr;
    if (!TestNotNull(TEXT("Physics drone"), Dynamics))
    {
        return false;
    }
    TStrongObjectPtr<UUBotAerialTestListener> Listener = Listen(*Dynamics);
    SendVelocity(*Dynamics, FVector(0.0, 0.0, -1.0));
    Scope.RunFor(4.0);
    TestTrue(TEXT("Landed"), Dynamics->IsLanded());
    TestEqual(TEXT("No crash"), Listener->CrashSpeeds.Num(), 0);
    const FUBotMultirotorState Landed = Dynamics->GetMultirotorState();
    TestEqual(TEXT("Resting on the floor"), Landed.PositionMeters.Z * 100.0, 9.0, 0.3);
    TestTrue(TEXT("Level"), UBot::Multirotor::GetTilt(Landed.Orientation) < 1e-6);
    TestTrue(TEXT("Motors spun down"), Landed.RotorSpeedsRadiansPerSecond[0] < 1.0);

    // Attitude commands without thrust or climb leave it on the ground.
    FUBotAttitudeCommand Level;
    IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Dynamics, Level);
    Scope.RunFor(1.0);
    TestTrue(TEXT("Stays landed without a climb"), Dynamics->IsLanded());

    // A climb command takes off.
    SendVelocity(*Dynamics, FVector(0.0, 0.0, 1.0));
    double TookOffAt = -1.0;
    const double CommandAt = Scope.Now();
    Scope.RunFor(2.5, FrameSeconds, [&] {
        if (TookOffAt < 0.0 && !Dynamics->IsLanded())
        {
            TookOffAt = Scope.Now() - CommandAt;
        }
    });
    TestTrue(FString::Printf(TEXT("Took off within 0.5 s (%.3f)"), TookOffAt), TookOffAt >= 0.0 && TookOffAt <= 0.5);
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
        AUBotPhysicsDrone* Drone = Scope.SpawnAndBegin<AUBotPhysicsDrone>(FVector(0.0, 0.0, 1000.0));
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialPawnPhysicsTest, "UBotAerial.Pawn.Physics.Components", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialPawnPhysicsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialTests;
    FScopedAerialTestWorld Scope;
    AUBotPhysicsDrone* Physics = Scope.SpawnAndBegin<AUBotPhysicsDrone>(FVector(0.0, 0.0, 500.0));
    AUBotBareAirframeDrone* Bare = Scope.SpawnAndBegin<AUBotBareAirframeDrone>(FVector(500.0, 0.0, 500.0));
    if (!TestNotNull(TEXT("Physics drone"), Physics) || !TestNotNull(TEXT("Bare airframe"), Bare))
    {
        return false;
    }
    for (AUBotMultirotorPawn* Drone : { static_cast<AUBotMultirotorPawn*>(Physics), static_cast<AUBotMultirotorPawn*>(Bare) })
    {
        UUBotMultirotorDynamicsComponent* Dynamics = Cast<UUBotMultirotorDynamicsComponent>(Drone->GetMultirotorMovement());
        if (!TestNotNull(TEXT("Movement is Multirotor Dynamics"), Dynamics))
        {
            continue;
        }
        TestTrue(TEXT("Default controller is the cascaded one"), Cast<UUBotCascadedFlightController>(Dynamics->FlightController) != nullptr);
        TestEqual(TEXT("1 kHz physics"), Dynamics->PhysicsRateHz, 1000.0f);
        TestEqual(TEXT("Crash speed"), Dynamics->CrashImpactSpeedMetersPerSecond, 3.0f);
        TArray<FVector> Positions;
        TArray<EUBotRotorSpinDirection> Spins;
        Dynamics->GetRotorLayout(Positions, Spins);
        for (int32 Index = 0; Index < 4; ++Index)
        {
            const FVector Location = Drone->Rotors[Index]->GetRelativeLocation();
            TestTrue(FString::Printf(TEXT("Rotor %d at its airframe position"), Index),
                FVector2D(Location).Equals(FVector2D(Positions[Index] * 100.0), 0.1));
        }
    }
    TestTrue(TEXT("Physics drone accepts velocity"), Physics->GetDynamics()->bAcceptVelocityCommands && Physics->GetDynamics()->bAcceptAttitudeCommands);
    TestFalse(TEXT("Bare airframe refuses velocity"), Bare->GetDynamics()->bAcceptVelocityCommands);
    TestFalse(TEXT("Bare airframe refuses attitude"), Bare->GetDynamics()->bAcceptAttitudeCommands);
    TestTrue(TEXT("Bare airframe takes body rates and motors"), Bare->GetDynamics()->bAcceptBodyRateCommands && Bare->GetDynamics()->bAcceptMotorCommands);
    return true;
}

#endif
