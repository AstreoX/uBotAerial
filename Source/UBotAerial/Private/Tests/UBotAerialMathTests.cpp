#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "UBotAerialTypes.h"
#include "UBotMultirotorMath.h"
#include "UBotSimpleFlightModel.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialMathFixedStepClockTest, "UBotAerial.Math.FixedStepClock", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialMathFixedStepClockTest::RunTest(const FString& Parameters)
{
    UBot::Multirotor::FFixedStepClock Clock;
    TestEqual(TEXT("Nothing is due before Reset"), Clock.GetStepsDue(10.0, 100), 0);

    Clock.Reset(10.0, 500.0);
    TestEqual(TEXT("Step length"), Clock.GetStepSeconds(), 0.002);
    TestEqual(TEXT("Nothing is due at the anchor"), Clock.GetStepsDue(10.0, 100), 0);

    // Frames of 1/60 s give 8 or 9 steps; over one second exactly 500.
    int32 Total = 0;
    double Now = 10.0;
    for (int32 Frame = 0; Frame < 60; ++Frame)
    {
        Now += 1.0 / 60.0;
        const int32 Due = Clock.GetStepsDue(Now, 100);
        TestTrue(TEXT("8 or 9 steps per frame"), Due == 8 || Due == 9);
        for (int32 Step = 0; Step < Due; ++Step)
        {
            Clock.AdvanceStep();
        }
        Total += Due;
    }
    TestEqual(TEXT("500 steps per second"), Total, 500);
    TestEqual(TEXT("Physics time follows the step index"), Clock.GetTimeSeconds(), 10.0 + 500 * 0.002, 1e-9);

    // A long frame is capped; the rest is skipped, not caught up.
    const int32 Capped = Clock.GetStepsDue(Now + 1.0, 100);
    TestEqual(TEXT("Capped at the maximum"), Capped, 100);
    for (int32 Step = 0; Step < Capped; ++Step)
    {
        Clock.AdvanceStep();
    }
    TestEqual(TEXT("Skipped steps are counted"), Clock.GetDroppedSteps(), static_cast<int64>(400));
    TestEqual(TEXT("Physics time reached sim time"), Clock.GetTimeSeconds(), Now + 1.0, 1e-9);
    TestEqual(TEXT("No catch-up afterwards"), Clock.GetStepsDue(Now + 1.0, 100), 0);

    // A new rate starts from the current physics time.
    const double Before = Clock.GetTimeSeconds();
    Clock.SetRate(1000.0);
    TestEqual(TEXT("Rate change keeps the time"), Clock.GetTimeSeconds(), Before, 1e-12);
    TestEqual(TEXT("New step length"), Clock.GetStepsDue(Before + 0.01, 100), 10);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialMathAttitudeTest, "UBotAerial.Math.AttitudeFromThrustAndYaw", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialMathAttitudeTest::RunTest(const FString& Parameters)
{
    using namespace UBot::Multirotor;

    const FQuat Level = AttitudeFromThrustDirectionAndYaw(FVector::UpVector, 0.0);
    TestTrue(TEXT("Up and yaw 0 is identity"), Level.Equals(FQuat::Identity, 1e-9));

    const FQuat Right = AttitudeFromThrustDirectionAndYaw(FVector::UpVector, UE_DOUBLE_HALF_PI);
    TestTrue(TEXT("Yaw +90 deg points X to world +Y (right)"), Right.GetAxisX().Equals(FVector(0.0, 1.0, 0.0), 1e-9));
    TestEqual(TEXT("GetYaw reads it back"), GetYaw(Right), UE_DOUBLE_HALF_PI, 1e-9);

    // Thrust tilted forward: the nose dips (body X points down a little), heading kept.
    const FVector Forward = FVector(1.0, 0.0, 4.0).GetSafeNormal();
    const FQuat Pitched = AttitudeFromThrustDirectionAndYaw(Forward, 0.0);
    TestTrue(TEXT("Body Z along the thrust"), Pitched.GetAxisZ().Equals(Forward, 1e-9));
    TestTrue(TEXT("Nose down when accelerating forward"), Pitched.GetAxisX().Z < 0.0);
    TestEqual(TEXT("Heading kept"), GetYaw(Pitched), 0.0, 1e-9);
    TestEqual(TEXT("Tilt"), GetTilt(Pitched), FMath::Atan(0.25), 1e-9);

    const FVector Limited = LimitTilt(FVector(1.0, 1.0, 0.1), FMath::DegreesToRadians(30.0));
    TestEqual(TEXT("Tilt limited"), FMath::RadiansToDegrees(FMath::Acos(Limited.Z)), 30.0, 1e-9);
    TestEqual(TEXT("Azimuth kept"), FMath::Atan2(Limited.Y, Limited.X), UE_DOUBLE_PI / 4.0, 1e-9);

    // A body rate of 1 rad/s about Z for 1 s turns right by 1 rad; small steps add up the same.
    FQuat Q = FQuat::Identity;
    for (int32 Step = 0; Step < 1000; ++Step)
    {
        Q = IntegrateBodyRate(Q, FVector(0.0, 0.0, 1.0), 0.001);
    }
    TestEqual(TEXT("Integrated yaw"), GetYaw(Q), 1.0, 1e-6);
    const FVector Rate = BodyRateBetween(FQuat::Identity, Q, 1.0);
    TestTrue(TEXT("Body rate between two attitudes"), Rate.Equals(FVector(0.0, 0.0, 1.0), 1e-6));

    TestEqual(TEXT("Wrap above pi"), WrapAngle(UE_DOUBLE_PI + 0.5), -UE_DOUBLE_PI + 0.5, 1e-12);
    TestEqual(TEXT("Wrap keeps pi"), WrapAngle(UE_DOUBLE_PI), UE_DOUBLE_PI, 1e-12);

    // Hovering: specific force is +g along body Z.
    const FVector Hover = SpecificForce(FQuat::Identity, FVector::ZeroVector, FVector(0.0, 0.0, -9.81));
    TestTrue(TEXT("Hover specific force"), Hover.Equals(FVector(0.0, 0.0, 9.81), 1e-12));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialMathBodyRateSmallAnglesTest, "UBotAerial.Math.BodyRateBetweenSmallAngles", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialMathBodyRateSmallAnglesTest::RunTest(const FString& Parameters)
{
    using namespace UBot::Multirotor;

    // A turn of less than 2e-4 rad has a quaternion vector part below 1e-4, where FQuat::ToAxisAndAngle falls back to the
    // X axis: the rate came out about X whatever the axis of the turn. Rates from far below that to far above it, about
    // axes that are not X, in the frame of a tilted attitude, must come out about their own axis. The increment is built
    // as IntegrateBodyRate builds it, so the expected rate is exact up to round-off (about 1e-16 / angle).
    const FQuat From = FQuat(FVector(1.0, 2.0, 3.0).GetSafeNormal(), 0.7);
    const FVector Axes[] = { FVector::ZAxisVector, FVector::YAxisVector, FVector(0.0, 1.0, 1.0).GetSafeNormal(), FVector(-1.0, 2.0, -0.5).GetSafeNormal() };
    const double StepSeconds = 0.002;
    for (const FVector& Axis : Axes)
    {
        for (const double Rate : { 1.0e-4, 1.0e-3, 0.01, 0.05, 0.0999, 0.1001, 0.5, 3.0 })
        {
            const FVector BodyRate = Axis * Rate;
            const FQuat To = IntegrateBodyRate(From, BodyRate, StepSeconds);
            const FVector Recovered = BodyRateBetween(From, To, StepSeconds);
            // Relative to the rate: at the smallest rate the vector part of the increment is Rate * 0.002 / 2 = 1e-7 and the
            // round-off of the quaternion products is a few 1e-16 of it, so about 4e-9 relative; 1e-6 leaves a margin of
            // 250 and is still far below the old error (the full rate, or more).
            TestTrue(FString::Printf(TEXT("Rate %g about %s is recovered (got %s)"), Rate, *Axis.ToString(), *Recovered.ToString()), (Recovered - BodyRate).Size() <= 1.0e-6 * Rate);
        }
    }

    // No rotation at all is no rate, and the shortest way round is taken for a quaternion of the opposite sign.
    TestTrue(TEXT("No rotation is no rate"), BodyRateBetween(From, From, StepSeconds).IsNearlyZero(1.0e-12));
    const FQuat Turned = IntegrateBodyRate(From, FVector(0.0, 0.0, 0.3), StepSeconds);
    const FQuat Opposite(-Turned.X, -Turned.Y, -Turned.Z, -Turned.W);
    TestTrue(TEXT("The sign of the quaternion does not matter"), BodyRateBetween(From, Opposite, StepSeconds).Equals(FVector(0.0, 0.0, 0.3), 1.0e-9));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialMathVelocityCommandTest, "UBotAerial.Math.ClampVelocityCommand", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialMathVelocityCommandTest::RunTest(const FString& Parameters)
{
    FUBotMultirotorVelocityLimits Limits;
    FVector Linear(30.0, 0.0, 20.0);
    double YawRate = 5.0;
    UBot::Multirotor::ClampVelocityCommand(Limits, Linear, YawRate);
    TestTrue(TEXT("Horizontal and climb clamped"), Linear.Equals(FVector(10.0, 0.0, 3.0), 1e-9));
    TestEqual(TEXT("Yaw rate clamped"), YawRate, static_cast<double>(UE_HALF_PI), 1e-6);

    Linear = FVector(8.0, 8.0, -9.0);
    YawRate = -9.0;
    UBot::Multirotor::ClampVelocityCommand(Limits, Linear, YawRate);
    TestEqual(TEXT("Diagonal clamped by its norm"), FVector(Linear.X, Linear.Y, 0.0).Size(), 10.0, 1e-9);
    TestEqual(TEXT("Direction kept"), Linear.X, Linear.Y, 1e-12);
    TestEqual(TEXT("Descent clamped"), Linear.Z, -2.0, 1e-9);
    TestEqual(TEXT("Negative yaw rate clamped"), YawRate, -static_cast<double>(UE_HALF_PI), 1e-6);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialMathQuadLayoutTest, "UBotAerial.Math.QuadXLayout", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialMathQuadLayoutTest::RunTest(const FString& Parameters)
{
    TArray<FVector> Positions;
    TArray<EUBotRotorSpinDirection> Spins;
    UBot::Multirotor::GetQuadXRotorLayout(0.225, Positions, Spins);
    if (!TestEqual(TEXT("Four rotors"), Positions.Num(), 4) || !TestEqual(TEXT("Four spin directions"), Spins.Num(), 4))
    {
        return false;
    }
    const double D = 0.225 / UE_DOUBLE_SQRT_2;
    TestTrue(TEXT("0 front right"), Positions[0].Equals(FVector(D, D, 0.0), 1e-12));
    TestTrue(TEXT("1 rear left"), Positions[1].Equals(FVector(-D, -D, 0.0), 1e-12));
    TestTrue(TEXT("2 front left"), Positions[2].Equals(FVector(D, -D, 0.0), 1e-12));
    TestTrue(TEXT("3 rear right"), Positions[3].Equals(FVector(-D, D, 0.0), 1e-12));
    TestTrue(TEXT("Diagonal pairs spin the same way"), Spins[0] == Spins[1] && Spins[2] == Spins[3] && Spins[0] != Spins[2]);
    TestEqual(TEXT("Front right spins counter-clockwise"), Spins[0], EUBotRotorSpinDirection::CounterClockwise);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialSimpleFlightModelTest, "UBotAerial.Math.SimpleFlightModel", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialSimpleFlightModelTest::RunTest(const FString& Parameters)
{
    using namespace UBot::SimpleFlight;
    const FParams Params;
    constexpr double H = 0.002;

    // Step response to 2 m/s: no overshoot, bounded acceleration and jerk.
    FState State;
    FInput Input;
    Input.TargetVelocity = FVector(2.0, 0.0, 0.0);
    double MaxSpeed = 0.0;
    double MaxAcceleration = 0.0;
    double MaxJerk = 0.0;
    double SpeedAt1500ms = 0.0;
    for (int32 Index = 1; Index <= 1500; ++Index)
    {
        const FVector AccelerationBefore = State.Acceleration;
        Step(Params, Input, H, State);
        MaxSpeed = FMath::Max(MaxSpeed, State.Velocity.X);
        MaxAcceleration = FMath::Max(MaxAcceleration, State.Acceleration.Size());
        MaxJerk = FMath::Max(MaxJerk, (State.Acceleration - AccelerationBefore).Size() / H);
        if (Index == 750)
        {
            SpeedAt1500ms = State.Velocity.X;
        }
    }
    TestTrue(FString::Printf(TEXT("Reaches 1.9 m/s by 1.5 s (%.4f)"), SpeedAt1500ms), SpeedAt1500ms >= 1.9);
    TestTrue(FString::Printf(TEXT("Overshoot at most 2%% (%.4f)"), MaxSpeed), MaxSpeed <= 2.04);
    TestTrue(FString::Printf(TEXT("Acceleration within the limit (%.4f)"), MaxAcceleration), MaxAcceleration <= Params.MaxHorizontalAcceleration + 1e-9);
    TestTrue(FString::Printf(TEXT("Jerk within the limit (%.4f)"), MaxJerk), MaxJerk <= Params.JerkLimit + 1e-6);

    // Braking from 2 m/s stops exactly, within 0.6 m and 1 s.
    Input.TargetVelocity = FVector::ZeroVector;
    Input.bBraking = true;
    double Distance = 0.0;
    int32 StopStep = -1;
    for (int32 Index = 1; Index <= 1000; ++Index)
    {
        Distance += Step(Params, Input, H, State).X;
        if (StopStep < 0 && State.Velocity.IsZero())
        {
            StopStep = Index;
        }
    }
    TestTrue(FString::Printf(TEXT("Brake distance at most 0.6 m (%.4f)"), Distance), Distance <= 0.6);
    TestTrue(FString::Printf(TEXT("Stopped exactly within 1 s (step %d)"), StopStep), StopStep > 0 && StopStep * H <= 1.0);

    // Steady wind is compensated; a step of 3 m/s drifts at most WindResponse * 3 m/s * 1 s.
    FState Windy;
    FInput Calm;
    Calm.Wind = FVector(3.0, 0.0, 0.0);
    double Drift = 0.0;
    for (int32 Index = 0; Index < 3000; ++Index)
    {
        Drift += Step(Params, Calm, H, Windy).X;
    }
    TestEqual(TEXT("Steady wind does not move the drone"), Drift, 0.0, 1e-12);
    FState Gusty;
    FInput Gust;
    Step(Params, Gust, H, Gusty);
    Gust.Wind = FVector(3.0, 0.0, 0.0);
    Drift = 0.0;
    for (int32 Index = 0; Index < 3000; ++Index)
    {
        Drift += Step(Params, Gust, H, Gusty).X;
    }
    TestTrue(FString::Printf(TEXT("Wind step drift bounded (%.4f)"), Drift), Drift > 1.0 && Drift <= 1.52);
    TestTrue(TEXT("Drift has ended after 6 s"), Gusty.WindDrift.Size() < 0.01);
    return true;
}

#endif
