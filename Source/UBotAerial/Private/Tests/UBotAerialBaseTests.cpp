#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "UBotAerialTestWorld.h"
#include "UBotNavigationDrone.h"
#include "UBotSimpleFlightComponent.h"

namespace UBotAerialBaseTests
{
    using namespace UBotAerialTests;

    // Flies a fixed command schedule into a wall and returns the final state.
    FUBotMultirotorState RunWallScenario(float DeltaSeconds)
    {
        FScopedAerialTestWorld Scope;
        Scope.SpawnFloor(0.0);
        Scope.SpawnBox(FVector(450.0, 0.0, 300.0), FVector(50.0, 500.0, 500.0), FRotator(0.0, 20.0, 0.0));
        AUBotNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotNavigationDrone>(FVector(0.0, 0.0, 200.0));
        UUBotSimpleFlightComponent* Flight = Drone->GetSimpleFlight();
        SendVelocity(*Flight, FVector(2.0, 0.0, 0.3), 0.3);
        Scope.RunFor(2.0, DeltaSeconds);
        SendVelocity(*Flight, FVector(0.5, 1.5, -0.5), -0.2);
        Scope.RunFor(2.0, DeltaSeconds);
        IUBotVelocityCommandable::Execute_StopMotion(Flight);
        Scope.RunFor(1.0, DeltaSeconds);
        return Flight->GetMultirotorState();
    }

    bool BitwiseEqual(const FUBotMultirotorState& A, const FUBotMultirotorState& B)
    {
        return A.PositionMeters == B.PositionMeters && A.LinearVelocityMetersPerSecond == B.LinearVelocityMetersPerSecond
            && A.Orientation.X == B.Orientation.X && A.Orientation.Y == B.Orientation.Y && A.Orientation.Z == B.Orientation.Z
            && A.Orientation.W == B.Orientation.W && A.AngularVelocityRadiansPerSecond == B.AngularVelocityRadiansPerSecond
            && A.FlightState == B.FlightState && A.SimTimeSeconds == B.SimTimeSeconds;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialBasePauseTest, "UBotAerial.Base.PauseNoMotion", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialBasePauseTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialBaseTests;
    FScopedAerialTestWorld Scope;
    AUBotNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotNavigationDrone>(FVector(0.0, 0.0, 500.0));
    UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
    if (!TestNotNull(TEXT("Drone"), Flight))
    {
        return false;
    }
    SendVelocity(*Flight, FVector(2.0, 0.0, 0.0));
    Scope.RunFor(1.0);

    const FUBotMultirotorState Before = Flight->GetMultirotorState();
    const int64 StepsBefore = Flight->GetPhysicsStepCount();
    const FVector ActorBefore = Drone->GetActorLocation();
    if (!TestTrue(TEXT("Paused"), Scope.SetPaused(true)))
    {
        return false;
    }
    Scope.RunFor(0.5);
    TestEqual(TEXT("No physics steps while paused"), Flight->GetPhysicsStepCount(), StepsBefore);
    TestTrue(TEXT("State unchanged while paused"), BitwiseEqual(Flight->GetMultirotorState(), Before));
    TestTrue(TEXT("Actor unchanged while paused"), Drone->GetActorLocation() == ActorBefore);

    Scope.SetPaused(false);
    Scope.Tick(FrameSeconds);
    const int64 Steps = Flight->GetPhysicsStepCount() - StepsBefore;
    TestTrue(FString::Printf(TEXT("No catch-up after the pause (%lld steps)"), Steps), Steps >= 8 && Steps <= 9);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDeterminismTwoRunsTest, "UBotAerial.Determinism.TwoRunsBitwise", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDeterminismTwoRunsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialBaseTests;
    const FUBotMultirotorState First = RunWallScenario(FrameSeconds);
    const FUBotMultirotorState Second = RunWallScenario(FrameSeconds);
    TestTrue(FString::Printf(TEXT("Two runs end bit-identical (%s vs %s)"), *First.PositionMeters.ToString(), *Second.PositionMeters.ToString()),
        BitwiseEqual(First, Second));
    TestTrue(TEXT("The scenario reached the wall or the floor"), First.PositionMeters.X > 2.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDeterminismFrameRateTest, "UBotAerial.Determinism.FrameRateIndependent", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDeterminismFrameRateTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialBaseTests;
    // A constant command from the start: 60 Hz and 24 Hz frames run the same 3000 physics steps.
    const auto Run = [](float DeltaSeconds, int64& OutSteps)
    {
        FScopedAerialTestWorld Scope;
        AUBotNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotNavigationDrone>(FVector(0.0, 0.0, 500.0));
        UUBotSimpleFlightComponent* Flight = Drone->GetSimpleFlight();
        SendVelocity(*Flight, FVector(1.5, 0.5, 0.2), 0.2);
        Scope.RunFor(6.0, DeltaSeconds);
        OutSteps = Flight->GetPhysicsStepCount();
        return Flight->GetMultirotorState();
    };
    int64 Steps60 = 0;
    int64 Steps24 = 0;
    const FUBotMultirotorState At60 = Run(1.0f / 60.0f, Steps60);
    const FUBotMultirotorState At24 = Run(1.0f / 24.0f, Steps24);
    TestEqual(TEXT("3000 steps at 60 Hz"), Steps60, static_cast<int64>(3000));
    TestEqual(TEXT("3000 steps at 24 Hz"), Steps24, static_cast<int64>(3000));
    TestTrue(FString::Printf(TEXT("Same state at step 3000 (%s vs %s)"), *At60.PositionMeters.ToString(), *At24.PositionMeters.ToString()),
        At60.PositionMeters == At24.PositionMeters && At60.LinearVelocityMetersPerSecond == At24.LinearVelocityMetersPerSecond);
    return true;
}

#endif
