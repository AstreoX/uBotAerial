#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "UBotAerialTestTypes.h"
#include "UBotAerialTestWorld.h"
#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotRotorComponent.h"
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
        AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 200.0));
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
            && A.SimTimeSeconds == B.SimTimeSeconds;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialBasePauseTest, "UBotAerial.Base.PauseNoMotion", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialBasePauseTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialBaseTests;
    FScopedAerialTestWorld Scope;
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 500.0));
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
        AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 500.0));
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialBaseRotorSpinFromLayoutTest, "UBotAerial.Base.RotorSpinFromLayout", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialBaseRotorSpinFromLayoutTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialTests;
    FScopedAerialTestWorld Scope;
    AUBotAerialTestNavigationDrone* Drone = Scope.Spawn<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 500.0));
    if (!TestNotNull(TEXT("Navigation drone"), Drone))
    {
        return false;
    }
    if (!TestEqual(TEXT("Four navigation rotors"), Drone->Rotors.Num(), 4))
    {
        return false;
    }
    for (const UUBotRotorComponent* Rotor : Drone->Rotors)
    {
        if (!TestNotNull(TEXT("Navigation rotor"), Rotor))
        {
            return false;
        }
        TestEqual(FString::Printf(TEXT("Rotor %d defaults to clockwise before BeginPlay"), Rotor->RotorIndex),
            Rotor->SpinDirection, EUBotRotorSpinDirection::Clockwise);
    }

    Drone->DispatchBeginPlay();
    TArray<FVector> Positions;
    TArray<EUBotRotorSpinDirection> Spins;
    Drone->GetMultirotorMovement()->GetRotorLayout(Positions, Spins);
    for (const UUBotRotorComponent* Rotor : Drone->Rotors)
    {
        if (!TestTrue(TEXT("Navigation rotor index is in the layout"), Spins.IsValidIndex(Rotor->RotorIndex)))
        {
            return false;
        }
        TestEqual(FString::Printf(TEXT("Navigation rotor %d follows its layout"), Rotor->RotorIndex),
            Rotor->SpinDirection, Spins[Rotor->RotorIndex]);
    }
    TestEqual(TEXT("PX4 rotor 0 is counter-clockwise"), Drone->Rotors[0]->SpinDirection, EUBotRotorSpinDirection::CounterClockwise);
    TestEqual(TEXT("PX4 rotor 2 is clockwise"), Drone->Rotors[2]->SpinDirection, EUBotRotorSpinDirection::Clockwise);

    Scope.Tick(FrameSeconds);
    const double Start0 = Drone->Rotors[0]->GetRelativeRotation().Yaw;
    const double Start2 = Drone->Rotors[2]->GetRelativeRotation().Yaw;
    Scope.Tick(FrameSeconds);
    const double Turn0 = FMath::FindDeltaAngleDegrees(Start0, Drone->Rotors[0]->GetRelativeRotation().Yaw);
    const double Turn2 = FMath::FindDeltaAngleDegrees(Start2, Drone->Rotors[2]->GetRelativeRotation().Yaw);
    TestTrue(FString::Printf(TEXT("Counter-clockwise rotor turns left (%.2f)"), Turn0), Turn0 < -0.5);
    TestTrue(FString::Printf(TEXT("Clockwise rotor turns right (%.2f)"), Turn2), Turn2 > 0.5);

    AUBotAerialTestPhysicsDrone* Physics = Scope.Spawn<AUBotAerialTestPhysicsDrone>(FVector(500.0, 0.0, 500.0));
    UUBotMultirotorDynamicsComponent* Dynamics = Physics ? Physics->GetDynamics() : nullptr;
    if (!TestNotNull(TEXT("Physics drone dynamics"), Dynamics))
    {
        return false;
    }
    if (!TestEqual(TEXT("Four physics rotors"), Physics->Rotors.Num(), 4)
        || !TestEqual(TEXT("Four airframe rotors"), Dynamics->Airframe.Rotors.Num(), 4))
    {
        return false;
    }
    for (int32 Index = 0; Index < Dynamics->Airframe.Rotors.Num(); ++Index)
    {
        EUBotRotorSpinDirection& Spin = Dynamics->Airframe.Rotors[Index].SpinDirection;
        Spin = Spin == EUBotRotorSpinDirection::Clockwise
            ? EUBotRotorSpinDirection::CounterClockwise : EUBotRotorSpinDirection::Clockwise;
    }
    Physics->DispatchBeginPlay();
    for (const UUBotRotorComponent* Rotor : Physics->Rotors)
    {
        if (!TestNotNull(TEXT("Physics rotor"), Rotor))
        {
            return false;
        }
        if (!TestTrue(TEXT("Physics rotor index is in the airframe"), Dynamics->Airframe.Rotors.IsValidIndex(Rotor->RotorIndex)))
        {
            return false;
        }
        TestEqual(FString::Printf(TEXT("Physics rotor %d follows the reversed airframe spin"), Rotor->RotorIndex),
            Rotor->SpinDirection, Dynamics->Airframe.Rotors[Rotor->RotorIndex].SpinDirection);
    }
    return true;
}

#endif
