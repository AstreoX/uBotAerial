#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "UBotAerialTestTypes.h"
#include "UBotAerialTestWorld.h"
#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorMath.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotSimpleFlightComponent.h"

// Commands that reach a drone before its own begin play (another actor's begin play can run first) are
// reported as accepted, so the drone has to follow them once the flight starts.
namespace UBotAerialStartCommandTests
{
    using namespace UBotAerialTests;

    struct FUnstartedPhysicsDrone
    {
        FScopedAerialTestWorld Scope;
        AUBotAerialTestPhysicsDrone* Drone = nullptr;
        UUBotMultirotorDynamicsComponent* Dynamics = nullptr;

        FUnstartedPhysicsDrone()
        {
            Drone = Scope.Spawn<AUBotAerialTestPhysicsDrone>(FVector(0.0, 0.0, 1000.0));
            Dynamics = Drone ? Drone->GetDynamics() : nullptr;
        }

        bool IsValid() const { return Dynamics != nullptr; }
        void Begin() const { Drone->DispatchBeginPlay(); }
        FUBotMultirotorState State() const { return Dynamics->GetMultirotorState(); }
    };

    struct FUnstartedNavigationDrone
    {
        FScopedAerialTestWorld Scope;
        AUBotAerialTestNavigationDrone* Drone = nullptr;
        UUBotSimpleFlightComponent* Flight = nullptr;

        FUnstartedNavigationDrone()
        {
            Drone = Scope.Spawn<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 1000.0));
            Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
        }

        bool IsValid() const { return Flight != nullptr; }
        void Begin() const { Drone->DispatchBeginPlay(); }
        FUBotMultirotorState State() const { return Flight->GetMultirotorState(); }
    };
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialStartCommandNavigationTest, "UBotAerial.StartCommand.NavigationVelocity", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialStartCommandNavigationTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialStartCommandTests;
    FUnstartedNavigationDrone Fixture;
    if (!TestTrue(TEXT("Navigation drone spawned"), Fixture.IsValid()))
    {
        return false;
    }
    TestFalse(TEXT("Not ready before begin play"), IUBotActuator::Execute_IsActuatorReady(Fixture.Flight));
    TestTrue(TEXT("Command accepted before begin play"), SendVelocity(*Fixture.Flight, FVector(1.0, 0.0, 0.0)));

    Fixture.Begin();
    TestTrue(TEXT("Ready after begin play"), IUBotActuator::Execute_IsActuatorReady(Fixture.Flight));
    TestEqual(TEXT("Last command is the one sent before begin play"), Fixture.Flight->GetLastVelocityCommand().LinearVelocityMetersPerSecond.X, 1.0);
    Fixture.Scope.RunFor(2.0);

    const double Forward = Fixture.State().LinearVelocityMetersPerSecond.X;
    TestTrue(FString::Printf(TEXT("Follows the command given before begin play (%.4f m/s)"), Forward), Forward >= 0.9);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialStartCommandPhysicsTest, "UBotAerial.StartCommand.PhysicsVelocity", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialStartCommandPhysicsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialStartCommandTests;
    FUnstartedPhysicsDrone Fixture;
    if (!TestTrue(TEXT("Physics drone spawned"), Fixture.IsValid()))
    {
        return false;
    }
    TestTrue(TEXT("Command accepted before begin play"), SendVelocity(*Fixture.Dynamics, FVector(1.0, 0.0, 0.0)));

    Fixture.Begin();
    Fixture.Scope.RunFor(2.0);

    TestEqual(TEXT("Velocity mode"), Fixture.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::Velocity);
    const double Forward = Fixture.State().LinearVelocityMetersPerSecond.X;
    TestTrue(FString::Printf(TEXT("Follows the command given before begin play (%.4f m/s)"), Forward), Forward >= 0.9);
    TestTrue(FString::Printf(TEXT("Holds altitude (%.4f m)"), Fixture.State().PositionMeters.Z), FMath::Abs(Fixture.State().PositionMeters.Z - 10.0) <= 0.15);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialStartCommandAttitudeTest, "UBotAerial.StartCommand.PhysicsAttitude", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialStartCommandAttitudeTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialStartCommandTests;
    FUnstartedPhysicsDrone Fixture;
    if (!TestTrue(TEXT("Physics drone spawned"), Fixture.IsValid()))
    {
        return false;
    }
    FUBotAttitudeCommand Roll;
    Roll.Orientation = FQuat(FVector::ForwardVector, FMath::DegreesToRadians(10.0));
    TestTrue(TEXT("Attitude command accepted before begin play"), IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Fixture.Dynamics, Roll));

    Fixture.Begin();
    TestEqual(TEXT("Attitude mode right after begin play"), Fixture.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::Attitude);
    Fixture.Scope.RunFor(1.0);

    const double ErrorDegrees = FMath::RadiansToDegrees(Fixture.State().Orientation.AngularDistance(Roll.Orientation));
    TestTrue(FString::Printf(TEXT("Reaches the attitude given before begin play (%.3f deg off)"), ErrorDegrees), ErrorDegrees <= 1.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialStartCommandBodyRateTest, "UBotAerial.StartCommand.PhysicsBodyRate", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialStartCommandBodyRateTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialStartCommandTests;
    FUnstartedPhysicsDrone Fixture;
    if (!TestTrue(TEXT("Physics drone spawned"), Fixture.IsValid()))
    {
        return false;
    }
    FUBotBodyRateCommand Rate;
    Rate.BodyRateRadiansPerSecond = FVector(0.0, 0.0, 0.3);
    TestTrue(TEXT("Body-rate command accepted before begin play"), IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Fixture.Dynamics, Rate));

    Fixture.Begin();
    TestEqual(TEXT("Body-rate mode right after begin play"), Fixture.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::BodyRate);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialStartCommandMotorTest, "UBotAerial.StartCommand.PhysicsMotor", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialStartCommandMotorTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialStartCommandTests;
    FUnstartedPhysicsDrone Fixture;
    if (!TestTrue(TEXT("Physics drone spawned"), Fixture.IsValid()))
    {
        return false;
    }
    FUBotMotorCommand Motors;
    Motors.Values.Init(0.0, 4);
    Motors.Source = TEXT("Test");
    TestTrue(TEXT("Motor command accepted before begin play"), IUBotMotorCommandable::Execute_ApplyMotorCommand(Fixture.Dynamics, Motors));

    Fixture.Begin();
    TestEqual(TEXT("Motor mode right after begin play"), Fixture.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::Motor);
    Fixture.Scope.RunFor(0.5);
    TestTrue(FString::Printf(TEXT("Falls with the motors off (%.3f m/s)"), Fixture.State().LinearVelocityMetersPerSecond.Z), Fixture.State().LinearVelocityMetersPerSecond.Z < -3.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialStartCommandLatestWinsTest, "UBotAerial.StartCommand.LatestWinsAndStopDrops", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialStartCommandLatestWinsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialStartCommandTests;
    {
        // Two commands before begin play: the second one counts.
        FUnstartedNavigationDrone Fixture;
        if (!TestTrue(TEXT("Navigation drone spawned"), Fixture.IsValid()))
        {
            return false;
        }
        SendVelocity(*Fixture.Flight, FVector(1.0, 0.0, 0.0));
        SendVelocity(*Fixture.Flight, FVector(0.0, 1.0, 0.0));
        Fixture.Begin();
        Fixture.Scope.RunFor(2.0);
        const FVector Velocity = Fixture.State().LinearVelocityMetersPerSecond;
        TestTrue(FString::Printf(TEXT("Second command wins (%.3f, %.3f)"), Velocity.X, Velocity.Y), Velocity.Y >= 0.9 && FMath::Abs(Velocity.X) <= 0.05);
    }
    {
        // A stop before begin play leaves nothing to run.
        FUnstartedNavigationDrone Fixture;
        if (!TestTrue(TEXT("Navigation drone spawned"), Fixture.IsValid()))
        {
            return false;
        }
        SendVelocity(*Fixture.Flight, FVector(1.0, 0.0, 0.0));
        IUBotVelocityCommandable::Execute_StopMotion(Fixture.Flight);
        Fixture.Begin();
        Fixture.Scope.RunFor(2.0);
        TestTrue(FString::Printf(TEXT("Stopped before begin play: hovers (%.3f m/s)"), Fixture.State().LinearVelocityMetersPerSecond.Size()),
            Fixture.State().LinearVelocityMetersPerSecond.Size() <= 0.05);
    }
    {
        // Releasing the attitude command before begin play leaves nothing to run either.
        FUnstartedPhysicsDrone Fixture;
        if (!TestTrue(TEXT("Physics drone spawned"), Fixture.IsValid()))
        {
            return false;
        }
        FUBotAttitudeCommand Roll;
        Roll.Orientation = FQuat(FVector::ForwardVector, FMath::DegreesToRadians(10.0));
        IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Fixture.Dynamics, Roll);
        IUBotAttitudeCommandable::Execute_ReleaseAttitudeCommand(Fixture.Dynamics);
        Fixture.Begin();
        TestEqual(TEXT("Velocity (hover) mode after the release"), Fixture.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::Velocity);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialStartCommandResetFlightTest, "UBotAerial.StartCommand.ResetFlightStillClears", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialStartCommandResetFlightTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialStartCommandTests;
    FUnstartedNavigationDrone Fixture;
    if (!TestTrue(TEXT("Navigation drone spawned"), Fixture.IsValid()))
    {
        return false;
    }
    SendVelocity(*Fixture.Flight, FVector(1.0, 0.0, 0.0));
    Fixture.Begin();
    Fixture.Scope.RunFor(1.0);
    TestTrue(TEXT("Flying forward"), Fixture.State().LinearVelocityMetersPerSecond.X > 0.5);

    // Reset Flight is a restart by design: it clears the command and hovers, and nothing comes back.
    Fixture.Flight->ResetFlight();
    Fixture.Scope.RunFor(2.0);
    TestTrue(FString::Printf(TEXT("Hovering after Reset Flight (%.3f m/s)"), Fixture.State().LinearVelocityMetersPerSecond.Size()),
        Fixture.State().LinearVelocityMetersPerSecond.Size() <= 0.05);
    TestEqual(TEXT("Last command cleared"), Fixture.Flight->GetLastVelocityCommand().LinearVelocityMetersPerSecond.Size(), 0.0);
    return true;
}

#endif
