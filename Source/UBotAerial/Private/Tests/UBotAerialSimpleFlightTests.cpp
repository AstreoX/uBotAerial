#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include <limits>

#include "Components/SceneComponent.h"
#include "UBotAerialTestTypes.h"
#include "UBotAerialTestWorld.h"
#include "UBotMultirotorMath.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotSimpleFlightComponent.h"

namespace UBotAerialSimpleFlightTests
{
    using namespace UBotAerialTests;

    // A navigation drone hovering 10 m up in an empty world.
    struct FHoveringDrone
    {
        FScopedAerialTestWorld Scope;
        AUBotAerialTestNavigationDrone* Drone = nullptr;
        UUBotSimpleFlightComponent* Flight = nullptr;

        explicit FHoveringDrone(EUBotSimpleFlightTiltMode TiltMode = EUBotSimpleFlightTiltMode::Visual)
        {
            Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 1000.0));
            Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
            if (Flight)
            {
                Flight->TiltMode = TiltMode;
            }
        }

        bool IsValid() const { return Flight != nullptr; }
        FVector Position() const { return Flight->GetMultirotorState().PositionMeters; }
        FVector Velocity() const { return Flight->GetMultirotorState().LinearVelocityMetersPerSecond; }
        double Yaw() const { return UBot::Multirotor::GetYaw(Flight->GetMultirotorState().Orientation); }
    };
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialSimpleFlightStepResponseTest, "UBotAerial.SimpleFlight.StepResponse", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialSimpleFlightStepResponseTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialSimpleFlightTests;
    FHoveringDrone Hover;
    if (!TestTrue(TEXT("Drone spawned with Simple Flight"), Hover.IsValid()))
    {
        return false;
    }
    TestEqual(TEXT("Starts flying in the air"), Hover.Flight->GetFlightState(), EUBotMultirotorFlightState::Flying);
    TestTrue(TEXT("Velocity command accepted"), SendVelocity(*Hover.Flight, FVector(2.0, 0.0, 0.0)));

    double MaxSpeed = 0.0;
    Hover.Scope.RunFor(1.5, FrameSeconds, [&] { MaxSpeed = FMath::Max(MaxSpeed, Hover.Velocity().X); });
    const double SpeedAt1500ms = Hover.Velocity().X;
    Hover.Scope.RunFor(2.0, FrameSeconds, [&] { MaxSpeed = FMath::Max(MaxSpeed, Hover.Velocity().X); });

    TestTrue(FString::Printf(TEXT("At least 1.9 m/s after 1.5 s (%.4f)"), SpeedAt1500ms), SpeedAt1500ms >= 1.9);
    TestTrue(FString::Printf(TEXT("Overshoot at most 2%% (%.4f)"), MaxSpeed), MaxSpeed <= 2.04);
    TestEqual(TEXT("Holds altitude"), Hover.Position().Z, 10.0, 1e-9);
    TestEqual(TEXT("Moves forward only"), Hover.Position().Y, 0.0, 1e-9);
    TestEqual(TEXT("Physics ran 500 steps per second"), Hover.Flight->GetPhysicsStepCount(), static_cast<int64>(1750));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialSimpleFlightYawRateTest, "UBotAerial.SimpleFlight.YawRate", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialSimpleFlightYawRateTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialSimpleFlightTests;
    FHoveringDrone Hover;
    if (!TestTrue(TEXT("Drone spawned"), Hover.IsValid()))
    {
        return false;
    }
    SendVelocity(*Hover.Flight, FVector::ZeroVector, 0.5);
    Hover.Scope.RunFor(4.0);
    // First-order lag of 0.15 s: 0.5 * (4 - 0.15) rad.
    TestEqual(TEXT("Yaw after 4 s at 0.5 rad/s"), Hover.Yaw(), 1.925, 0.03);
    TestTrue(TEXT("Positive yaw turns right (actor yaw increases)"), Hover.Drone->GetActorRotation().Yaw > 100.0);

    // Forward in the heading frame follows the new heading.
    SendVelocity(*Hover.Flight, FVector(1.0, 0.0, 0.0));
    const FVector Start = Hover.Position();
    Hover.Scope.RunFor(3.0);
    const FVector Moved = Hover.Position() - Start;
    const double Heading = FMath::Atan2(Moved.Y, Moved.X);
    TestEqual(TEXT("Moves along the heading"), Heading, Hover.Yaw(), 0.02);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialSimpleFlightLimitsTest, "UBotAerial.SimpleFlight.Limits", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialSimpleFlightLimitsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialSimpleFlightTests;
    FHoveringDrone Hover;
    if (!TestTrue(TEXT("Drone spawned"), Hover.IsValid()))
    {
        return false;
    }
    FUBotVelocityCommand Command = MakeVelocityCommand(FVector(30.0, 0.0, 20.0), 5.0);
    Command.AngularVelocityRadiansPerSecond.X = 1.0;
    Command.AngularVelocityRadiansPerSecond.Y = -1.0;
    TestTrue(TEXT("Accepted"), IUBotVelocityCommandable::Execute_ApplyVelocityCommand(Hover.Flight, Command));
    const FUBotVelocityCommand Stored = Hover.Flight->GetLastVelocityCommand();
    TestTrue(TEXT("Clamped to 10 m/s horizontal, 3 m/s climb"), Stored.LinearVelocityMetersPerSecond.Equals(FVector(10.0, 0.0, 3.0), 1e-6));
    TestTrue(TEXT("Yaw rate clamped, roll and pitch rates ignored"), Stored.AngularVelocityRadiansPerSecond.Equals(FVector(0.0, 0.0, UE_HALF_PI), 1e-6));

    FUBotVelocityCommand Invalid = MakeVelocityCommand(FVector(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    TestFalse(TEXT("NaN rejected"), IUBotVelocityCommandable::Execute_ApplyVelocityCommand(Hover.Flight, Invalid));

    Hover.Scope.RunFor(8.0);
    TestTrue(FString::Printf(TEXT("Never faster than the limit (%.4f)"), FVector2D(Hover.Velocity()).Size()), FVector2D(Hover.Velocity()).Size() <= 10.0 + 1e-6);
    TestEqual(TEXT("Climbs at most 3 m/s"), Hover.Velocity().Z, 3.0, 1e-3);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialSimpleFlightStopBoundedTest, "UBotAerial.SimpleFlight.StopBounded", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialSimpleFlightStopBoundedTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialSimpleFlightTests;
    {
        FHoveringDrone Hover;
        if (!TestTrue(TEXT("Drone spawned"), Hover.IsValid()))
        {
            return false;
        }
        SendVelocity(*Hover.Flight, FVector(2.0, 0.0, 0.0));
        Hover.Scope.RunFor(4.0);
        TestEqual(TEXT("Cruising at 2 m/s"), Hover.Velocity().X, 2.0, 1e-3);

        IUBotVelocityCommandable::Execute_StopMotion(Hover.Flight);
        TestTrue(TEXT("Braking"), Hover.Flight->IsBraking());
        const FVector StopStart = Hover.Position();
        double StoppedAt = -1.0;
        const double T0 = Hover.Scope.Now();
        Hover.Scope.RunFor(1.0, FrameSeconds, [&] {
            if (StoppedAt < 0.0 && Hover.Velocity().IsZero())
            {
                StoppedAt = Hover.Scope.Now() - T0;
            }
        });
        const double Distance = (Hover.Position() - StopStart).Size();
        TestTrue(FString::Printf(TEXT("Brake distance at most 0.6 m (%.4f)"), Distance), Distance <= 0.6);
        TestTrue(FString::Printf(TEXT("Velocity exactly zero within 1 s (%.3f)"), StoppedAt), StoppedAt >= 0.0 && StoppedAt <= 1.0);

        const FVector Rest = Hover.Position();
        Hover.Scope.RunFor(5.0);
        TestTrue(TEXT("Stays exactly where it stopped"), Hover.Position() == Rest);
    }
    {
        // The legacy acceptance: from 0.1 m/s, at most 2 cm within 0.65 s.
        FHoveringDrone Hover;
        SendVelocity(*Hover.Flight, FVector(0.1, 0.0, 0.0));
        Hover.Scope.RunFor(3.0);
        IUBotVelocityCommandable::Execute_StopMotion(Hover.Flight);
        const FVector StopStart = Hover.Position();
        Hover.Scope.RunFor(0.65);
        const double DriftCm = (Hover.Position() - StopStart).Size() * 100.0;
        TestTrue(FString::Printf(TEXT("Drift at most 2 cm (%.3f cm)"), DriftCm), DriftCm <= 2.0);
        TestTrue(TEXT("Stopped"), Hover.Velocity().IsZero());
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialSimpleFlightLegacyAcceptanceTest, "UBotAerial.SimpleFlight.LegacyAcceptance", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialSimpleFlightLegacyAcceptanceTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialSimpleFlightTests;
    // Each axis: 2.5 s of a 1 m/s command moves at least 12 cm the right way; yaw turns at least 2 degrees.
    const FVector Axes[] = { FVector(1, 0, 0), FVector(-1, 0, 0), FVector(0, 1, 0), FVector(0, -1, 0), FVector(0, 0, 1), FVector(0, 0, -1) };
    for (const FVector& Axis : Axes)
    {
        FHoveringDrone Hover;
        if (!TestTrue(TEXT("Drone spawned"), Hover.IsValid()))
        {
            return false;
        }
        const FVector Start = Hover.Position();
        SendVelocity(*Hover.Flight, Axis);
        Hover.Scope.RunFor(2.5);
        const double Along = FVector::DotProduct(Hover.Position() - Start, Axis) * 100.0;
        TestTrue(FString::Printf(TEXT("Axis %s moves %.1f cm"), *Axis.ToString(), Along), Along >= 12.0);
    }
    for (const double YawRate : { 0.5, -0.5 })
    {
        FHoveringDrone Hover;
        SendVelocity(*Hover.Flight, FVector::ZeroVector, YawRate);
        Hover.Scope.RunFor(2.5);
        TestTrue(FString::Printf(TEXT("Yaw rate %.1f turns at least 2 degrees"), YawRate), FMath::RadiansToDegrees(Hover.Yaw() * FMath::Sign(YawRate)) >= 2.0);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialSimpleFlightSpecificForceTest, "UBotAerial.SimpleFlight.SpecificForce", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialSimpleFlightSpecificForceTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialSimpleFlightTests;
    {
        // Hovering, Visual tilt: an IMU on the root reads +g up.
        FHoveringDrone Hover(EUBotSimpleFlightTiltMode::Visual);
        FFrameImuProbe Probe;
        bool bChecked = false;
        Hover.Scope.RunFor(1.0, FrameSeconds, [&] {
            if (Probe.Sample(*Hover.Drone->GetRootComponent(), Hover.Scope.Now(), Hover.Scope.Gravity()))
            {
                bChecked = true;
                TestTrue(FString::Printf(TEXT("Hover reads +g (%s)"), *Probe.SpecificForce.ToString()),
                    Probe.SpecificForce.Equals(-Hover.Scope.Gravity(), 0.02));
            }
        });
        TestTrue(TEXT("Probe sampled"), bChecked);
        TestTrue(TEXT("State reports the same"), Hover.Flight->GetMultirotorState().SpecificForceMetersPerSecondSquared.Equals(-Hover.Scope.Gravity(), 1e-9));
    }
    {
        // Body tilt: while accelerating, the root tilts with the thrust, so an IMU reads the force along its Z only.
        FHoveringDrone Hover(EUBotSimpleFlightTiltMode::Body);
        FFrameImuProbe Probe;
        Hover.Scope.RunFor(0.2, FrameSeconds, [&] { Probe.Sample(*Hover.Drone->GetRootComponent(), Hover.Scope.Now(), Hover.Scope.Gravity()); });
        SendVelocity(*Hover.Flight, FVector(3.0, 0.0, 0.0));
        Hover.Scope.RunFor(0.3, FrameSeconds, [&] { Probe.Sample(*Hover.Drone->GetRootComponent(), Hover.Scope.Now(), Hover.Scope.Gravity()); });
        const FVector Force = Probe.SpecificForce;
        TestTrue(FString::Printf(TEXT("Accelerating at the limit (%s)"), *Probe.Acceleration.ToString()), FMath::Abs(Probe.Acceleration.X - 4.0) < 0.1);
        TestTrue(FString::Printf(TEXT("Force along body Z only (%s)"), *Force.ToString()), FMath::Abs(Force.X) <= 0.15 && FMath::Abs(Force.Y) <= 0.15);
        TestEqual(TEXT("Magnitude of a - g"), Force.Z, (Probe.Acceleration - Hover.Scope.Gravity()).Size(), 0.1);
        TestTrue(TEXT("Nose down"), Hover.Drone->GetActorRotation().Pitch < -5.0);
    }
    {
        // Visual tilt: the root stays level while the visual root tilts.
        FHoveringDrone Hover(EUBotSimpleFlightTiltMode::Visual);
        SendVelocity(*Hover.Flight, FVector(3.0, 0.0, 0.0));
        Hover.Scope.RunFor(0.3);
        TestEqual(TEXT("Root level"), Hover.Drone->GetActorRotation().Pitch, 0.0, 1e-6);
        TestTrue(TEXT("Visual root tilts nose down"), Hover.Drone->VisualRoot->GetComponentRotation().Pitch < -5.0);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialSimpleFlightPresentationTest, "UBotAerial.SimpleFlight.PresentationNoJitter", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialSimpleFlightPresentationTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialSimpleFlightTests;
    FHoveringDrone Hover;
    if (!TestTrue(TEXT("Drone spawned"), Hover.IsValid()))
    {
        return false;
    }
    SendVelocity(*Hover.Flight, FVector(3.0, 0.0, 0.0));
    Hover.Scope.RunFor(4.0);
    // Frames alternate between 8 and 9 physics steps; the presented pose still moves evenly.
    FFrameImuProbe Probe;
    double MaxAcceleration = 0.0;
    double MinSpeed = 1e9;
    double MaxSpeed = 0.0;
    Hover.Scope.RunFor(1.0, FrameSeconds, [&] {
        if (Probe.Sample(*Hover.Drone->GetRootComponent(), Hover.Scope.Now(), Hover.Scope.Gravity()))
        {
            MaxAcceleration = FMath::Max(MaxAcceleration, Probe.Acceleration.Size());
            MinSpeed = FMath::Min(MinSpeed, Probe.Velocity.X);
            MaxSpeed = FMath::Max(MaxSpeed, Probe.Velocity.X);
        }
    });
    TestTrue(FString::Printf(TEXT("Frame-sampled acceleration at most 0.05 m/s² (%.4f)"), MaxAcceleration), MaxAcceleration <= 0.05);
    TestTrue(FString::Printf(TEXT("Frame-sampled speed steady (%.4f..%.4f)"), MinSpeed, MaxSpeed), MaxSpeed - MinSpeed <= 0.002);
    return true;
}

#endif
