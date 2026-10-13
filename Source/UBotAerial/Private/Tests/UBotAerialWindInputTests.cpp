#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "UBotAerialTestTypes.h"
#include "UBotAerialTestWorld.h"
#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorManualInputComponent.h"
#include "UBotMultirotorMath.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotSimpleFlightComponent.h"
#include "UBotWind.h"
#include "UBotWindSubsystem.h"

namespace UBotAerialWindInputTests
{
    using namespace UBotAerialTests;

    FUBotWindSettings MakeWind(const FVector& Constant, const FVector& Gust = FVector::ZeroVector, int32 Seed = 0)
    {
        FUBotWindSettings Settings;
        Settings.ConstantVelocityMetersPerSecond = Constant;
        Settings.GustStandardDeviationMetersPerSecond = Gust;
        Settings.Seed = Seed;
        return Settings;
    }

    AUBotWind* SpawnWind(const FScopedAerialTestWorld& Scope, const FUBotWindSettings& Settings)
    {
        AUBotWind* Wind = Scope.Spawn<AUBotWind>(FVector::ZeroVector);
        Wind->Settings = Settings;
        Wind->DispatchBeginPlay();
        return Wind;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialWindGustTest, "UBotAerial.Wind.GustDeterministic", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialWindGustTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialWindInputTests;
    const FUBotWindSettings Settings = MakeWind(FVector::ZeroVector, FVector(1.0, 2.0, 0.5), 7);
    UBot::Wind::FGustModel A;
    UBot::Wind::FGustModel B;
    UBot::Wind::FGustModel Other;
    A.Build(Settings);
    B.Build(Settings);
    FUBotWindSettings OtherSettings = Settings;
    OtherSettings.Seed = 8;
    Other.Build(OtherSettings);

    bool bSame = true;
    bool bDiffers = false;
    FVector Sum = FVector::ZeroVector;
    FVector SumSquares = FVector::ZeroVector;
    constexpr int32 Samples = 60000;
    for (int32 Index = 0; Index < Samples; ++Index)
    {
        const double Time = Index * 0.01;
        const FVector Gust = A.Evaluate(Time);
        bSame &= Gust == B.Evaluate(Time);
        bDiffers |= !Gust.Equals(Other.Evaluate(Time), 1e-6);
        Sum += Gust;
        SumSquares += Gust * Gust;
    }
    TestTrue(TEXT("Same settings, same gusts"), bSame);
    TestTrue(TEXT("Another seed, other gusts"), bDiffers);
    for (int32 Axis = 0; Axis < 3; ++Axis)
    {
        const double Sigma = Settings.GustStandardDeviationMetersPerSecond[Axis];
        const double Mean = Sum[Axis] / Samples;
        const double Rms = FMath::Sqrt(SumSquares[Axis] / Samples);
        TestTrue(FString::Printf(TEXT("Axis %d: RMS %.3f within 15%% of %.3f"), Axis, Rms, Sigma), FMath::Abs(Rms - Sigma) <= 0.15 * Sigma);
        TestTrue(FString::Printf(TEXT("Axis %d: mean %.3f near zero"), Axis, Mean), FMath::Abs(Mean) <= 0.1 * Sigma);
    }

    UBot::Wind::FGustModel Calm;
    Calm.Build(MakeWind(FVector(3.0, 0.0, 0.0)));
    TestTrue(TEXT("No gusts without a deviation"), Calm.Evaluate(12.3).IsZero());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialWindSourcesTest, "UBotAerial.Wind.ActorAndScripted", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialWindSourcesTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialWindInputTests;
    FScopedAerialTestWorld Scope;
    UUBotWindSubsystem* Wind = Scope.Get()->GetSubsystem<UUBotWindSubsystem>();
    if (!TestNotNull(TEXT("Wind subsystem in a game world"), Wind))
    {
        return false;
    }
    TestTrue(TEXT("Calm by default"), Wind->GetWindVelocityAt(FVector::ZeroVector).IsZero());

    AUBotWind* First = SpawnWind(Scope, MakeWind(FVector(2.0, 0.0, 0.0)));
    SpawnWind(Scope, MakeWind(FVector(0.0, -1.0, 0.0)));
    TestTrue(TEXT("Actors add up"), Wind->GetWindVelocityAt(FVector::ZeroVector).Equals(FVector(2.0, -1.0, 0.0), 1e-12));
    Wind->SetScriptedWind(MakeWind(FVector(0.0, 0.0, 0.5)));
    TestTrue(TEXT("Scripted wind adds too"), Wind->GetWindVelocityAt(FVector(1000.0, 0.0, 0.0)).Equals(FVector(2.0, -1.0, 0.5), 1e-12));
    First->SetWindSettings(MakeWind(FVector(4.0, 0.0, 0.0)));
    TestTrue(TEXT("Changed settings apply"), Wind->GetWindVelocityAt(FVector::ZeroVector).Equals(FVector(4.0, -1.0, 0.5), 1e-12));
    First->SetWindEnabled(false);
    Wind->SetScriptedWind(FUBotWindSettings(), false);
    TestTrue(TEXT("Disabled and cleared"), Wind->GetWindVelocityAt(FVector::ZeroVector).Equals(FVector(0.0, -1.0, 0.0), 1e-12));
    First->RouteEndPlay(EEndPlayReason::Destroyed);
    TestTrue(TEXT("Ended actors leave"), Wind->GetWindVelocityAt(FVector::ZeroVector).Equals(FVector(0.0, -1.0, 0.0), 1e-12));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialWindPhysicsTest, "UBotAerial.Wind.PhysicsSteady", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialWindPhysicsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialWindInputTests;
    FScopedAerialTestWorld Scope;
    SpawnWind(Scope, MakeWind(FVector(5.0, 0.0, 0.0)));
    AUBotAerialTestPhysicsDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestPhysicsDrone>(FVector(0.0, 0.0, 1000.0));
    UUBotMultirotorDynamicsComponent* Dynamics = Drone ? Drone->GetDynamics() : nullptr;
    if (!TestNotNull(TEXT("Physics drone"), Dynamics))
    {
        return false;
    }
    double Peak = 0.0;
    Scope.RunFor(10.0, FrameSeconds, [&] { Peak = FMath::Max(Peak, FVector2D(Dynamics->GetMultirotorState().PositionMeters).Size()); });
    double MaxSpeed = 0.0;
    Scope.RunFor(10.0, FrameSeconds, [&] { MaxSpeed = FMath::Max(MaxSpeed, Dynamics->GetMultirotorState().LinearVelocityMetersPerSecond.Size()); });
    const double Tilt = FMath::RadiansToDegrees(UBot::Multirotor::GetTilt(Dynamics->GetMultirotorState().Orientation));
    TestTrue(FString::Printf(TEXT("Pushed at most 1 m (%.3f)"), Peak), Peak <= 1.0 && Peak > 0.1);
    TestTrue(FString::Printf(TEXT("Holds still after 10 s (%.4f)"), MaxSpeed), MaxSpeed < 0.05);
    TestEqual(TEXT("Leans into the wind"), Tilt, 7.7, 1.0);
    TestTrue(TEXT("Leans upwind (thrust points against the wind)"), Dynamics->GetMultirotorState().Orientation.GetAxisZ().X < 0.0);

    // Without wind on this drone it does not lean.
    Dynamics->bEnableWind = false;
    Scope.RunFor(10.0);
    TestTrue(TEXT("Wind off: level again"), UBot::Multirotor::GetTilt(Dynamics->GetMultirotorState().Orientation) < FMath::DegreesToRadians(0.5));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialWindSimpleTest, "UBotAerial.Wind.SimpleFlightStep", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialWindSimpleTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialWindInputTests;
    FScopedAerialTestWorld Scope;
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 1000.0));
    UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
    if (!TestNotNull(TEXT("Navigation drone"), Flight))
    {
        return false;
    }
    Scope.RunFor(1.0);
    // A 3 m/s gust front arrives: the drone drifts, then compensates.
    Scope.Get()->GetSubsystem<UUBotWindSubsystem>()->SetScriptedWind(MakeWind(FVector(0.0, 3.0, 0.0)));
    const FVector Start = Flight->GetMultirotorState().PositionMeters;
    double Drift = 0.0;
    Scope.RunFor(8.0, FrameSeconds, [&] { Drift = FMath::Max(Drift, (Flight->GetMultirotorState().PositionMeters - Start).Size()); });
    TestTrue(FString::Printf(TEXT("Drift bounded by 0.5 * 3 m/s * 1 s (%.3f)"), Drift), Drift > 1.0 && Drift <= 1.52);
    TestTrue(FString::Printf(TEXT("Compensated after 8 s (%.4f)"), Flight->GetMultirotorState().LinearVelocityMetersPerSecond.Size()),
        Flight->GetMultirotorState().LinearVelocityMetersPerSecond.Size() < 0.01);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialInputMappingTest, "UBotAerial.Input.Mapping", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialInputMappingTest::RunTest(const FString& Parameters)
{
    using namespace UBot::ManualFlight;
    const FSettings Settings;

    FUBotManualFlightAxes Raw;
    Raw.Forward = 1.0f;
    FUBotVelocityCommand Command = MakeVelocityCommand(ConditionAxes(Raw, Settings.DeadZone), Settings);
    TestTrue(TEXT("W: forward at full speed"), Command.LinearVelocityMetersPerSecond.Equals(FVector(5.0, 0.0, 0.0), 1e-6));

    Raw.Right = 1.0f;
    Command = MakeVelocityCommand(ConditionAxes(Raw, Settings.DeadZone), Settings);
    TestEqual(TEXT("W+D: not faster on the diagonal"), FVector2D(Command.LinearVelocityMetersPerSecond).Size(), 5.0, 1e-5);
    TestEqual(TEXT("W+D: 45 degrees right"), Command.LinearVelocityMetersPerSecond.X, Command.LinearVelocityMetersPerSecond.Y, 1e-5);

    Raw = FUBotManualFlightAxes();
    Raw.Up = -1.0f;
    Raw.Yaw = 1.0f;
    Command = MakeVelocityCommand(ConditionAxes(Raw, Settings.DeadZone), Settings);
    TestEqual(TEXT("Ctrl: descend"), Command.LinearVelocityMetersPerSecond.Z, -2.0, 1e-6);
    TestEqual(TEXT("E: yaw right"), Command.AngularVelocityRadiansPerSecond.Z, UE_DOUBLE_HALF_PI, 1e-6);

    TestEqual(TEXT("Inside the dead zone"), ApplyDeadZone(0.1, 0.15), 0.0);
    TestEqual(TEXT("Dead zone rescales"), ApplyDeadZone(0.5, 0.15), 0.35 / 0.85, 1e-12);
    TestEqual(TEXT("Full deflection stays full"), ApplyDeadZone(-1.0, 0.15), -1.0, 1e-12);

    // Angle mode: full right stick rolls 30 degrees right; the yaw axis turns the heading.
    FUBotManualFlightAxes Stick;
    Stick.Right = 1.0f;
    double Yaw = 0.0;
    FUBotAttitudeCommand Attitude = MakeAttitudeCommand(Stick, Settings, Yaw, 0.0);
    TestEqual(TEXT("Rolled right: body Y points down"), Attitude.Orientation.GetAxisY().Z, -0.5, 1e-6);
    TestEqual(TEXT("Heading kept"), UBot::Multirotor::GetYaw(Attitude.Orientation), 0.0, 1e-9);
    Stick = FUBotManualFlightAxes();
    Stick.Forward = 1.0f;
    Attitude = MakeAttitudeCommand(Stick, Settings, Yaw, 0.0);
    TestTrue(TEXT("Forward stick: nose down"), Attitude.Orientation.GetAxisX().Z < -0.4);
    Stick = FUBotManualFlightAxes();
    Stick.Yaw = 1.0f;
    Stick.Up = 0.5f;
    for (int32 Frame = 0; Frame < 60; ++Frame)
    {
        Attitude = MakeAttitudeCommand(Stick, Settings, Yaw, 1.0 / 60.0);
    }
    TestEqual(TEXT("Yaw turned 90 degrees in 1 s"), FMath::RadiansToDegrees(Yaw), 90.0, 1e-6);
    TestEqual(TEXT("Climb rate from the up axis"), Attitude.VerticalVelocityMetersPerSecond, 1.0, 1e-6);
    TestFalse(TEXT("No thrust: the drone holds the climb rate"), Attitude.bUseThrust);

    // Rate mode signs in the UE rotation sense.
    Stick = FUBotManualFlightAxes();
    Stick.Right = 1.0f;
    Stick.Forward = 1.0f;
    Stick.Yaw = -1.0f;
    const FUBotBodyRateCommand Rates = MakeBodyRateCommand(Stick, Settings);
    TestTrue(TEXT("Right stick: roll right (-X)"), Rates.BodyRateRadiansPerSecond.X < 0.0);
    TestTrue(TEXT("Forward stick: nose down (+Y)"), Rates.BodyRateRadiansPerSecond.Y > 0.0);
    TestTrue(TEXT("Q: yaw left (-Z)"), Rates.BodyRateRadiansPerSecond.Z < 0.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialInputBindingsTest, "UBotAerial.Input.DefaultBindings", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialInputBindingsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialTests;
    const TArray<UBot::ManualFlight::FKeyBinding> Bindings = UBot::ManualFlight::GetDefaultBindings();
    TSet<FKey> Keys;
    for (const UBot::ManualFlight::FKeyBinding& Binding : Bindings)
    {
        TestFalse(FString::Printf(TEXT("%s bound once"), *Binding.Key.ToString()), Keys.Contains(Binding.Key));
        Keys.Add(Binding.Key);
    }
    // Every axis has a key both ways and a stick.
    for (int32 Axis = 0; Axis < 4; ++Axis)
    {
        TestTrue(FString::Printf(TEXT("Axis %d has a positive key"), Axis),
            Bindings.ContainsByPredicate([Axis](const UBot::ManualFlight::FKeyBinding& B) { return B.Axis == Axis && B.Scale > 0.0 && !B.Key.IsGamepadKey(); }));
        TestTrue(FString::Printf(TEXT("Axis %d has a negative key"), Axis),
            Bindings.ContainsByPredicate([Axis](const UBot::ManualFlight::FKeyBinding& B) { return B.Axis == Axis && B.Scale < 0.0; }));
        TestTrue(FString::Printf(TEXT("Axis %d has a stick"), Axis),
            Bindings.ContainsByPredicate([Axis](const UBot::ManualFlight::FKeyBinding& B) { return B.Axis == Axis && B.Key.IsGamepadKey(); }));
    }

    FScopedAerialTestWorld Scope;
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 500.0));
    UUBotMultirotorManualInputComponent* Input = Drone ? Drone->ManualInput.Get() : nullptr;
    if (!TestNotNull(TEXT("Manual input on the drone"), Input))
    {
        return false;
    }
    TestFalse(TEXT("Idle without a player"), Input->IsControlling());
    TestFalse(TEXT("Does not tick without a player"), Input->IsComponentTickEnabled());

    Input->BuildDefaultInput();
    const UInputMappingContext* Context = Input->GetDefaultMappingContext();
    if (!TestNotNull(TEXT("Mapping context built"), Context))
    {
        return false;
    }
    TestEqual(TEXT("One mapping per binding"), Context->GetMappings().Num(), Bindings.Num());
    for (const FEnhancedActionKeyMapping& Mapping : Context->GetMappings())
    {
        const UBot::ManualFlight::FKeyBinding* Binding = Bindings.FindByPredicate([&Mapping](const UBot::ManualFlight::FKeyBinding& B) { return B.Key == Mapping.Key; });
        if (!TestNotNull(TEXT("Mapped key is a default binding"), Binding))
        {
            continue;
        }
        const bool bNegated = Mapping.Modifiers.ContainsByPredicate([](const TObjectPtr<UInputModifier>& M) { return M && M->IsA<UInputModifierNegate>(); });
        TestEqual(FString::Printf(TEXT("%s negated as bound"), *Mapping.Key.ToString()), bNegated, Binding->Scale < 0.0);
        const UInputModifierSwizzleAxis* Swizzle = nullptr;
        for (const TObjectPtr<UInputModifier>& Modifier : Mapping.Modifiers)
        {
            if (const UInputModifierSwizzleAxis* Found = Cast<UInputModifierSwizzleAxis>(Modifier))
            {
                Swizzle = Found;
            }
        }
        if (Binding->Axis == 1)
        {
            TestTrue(FString::Printf(TEXT("%s moves to Y"), *Mapping.Key.ToString()), Swizzle && Swizzle->Order == EInputAxisSwizzle::YXZ);
        }
        else if (Binding->Axis == 2)
        {
            TestTrue(FString::Printf(TEXT("%s moves to Z"), *Mapping.Key.ToString()), Swizzle && Swizzle->Order == EInputAxisSwizzle::ZYX);
        }
        else
        {
            TestNull(FString::Printf(TEXT("%s stays on X"), *Mapping.Key.ToString()), Swizzle);
        }
        TestEqual(FString::Printf(TEXT("%s uses the right action"), *Mapping.Key.ToString()),
            Mapping.Action->ValueType, Binding->Axis == 3 ? EInputActionValueType::Axis1D : EInputActionValueType::Axis3D);
    }

    // Each drone flies with the mode it understands.
    AUBotAerialTestPhysicsDrone* Physics = Scope.SpawnAndBegin<AUBotAerialTestPhysicsDrone>(FVector(500.0, 0.0, 500.0));
    AUBotAerialTestBareDrone* Bare = Scope.SpawnAndBegin<AUBotAerialTestBareDrone>(FVector(1000.0, 0.0, 500.0));
    TestEqual(TEXT("Navigation drone: velocity"), Drone->ManualInput->FlightMode, EUBotManualFlightMode::Velocity);
    TestEqual(TEXT("Physics drone: velocity"), Physics->ManualInput->FlightMode, EUBotManualFlightMode::Velocity);
    TestEqual(TEXT("Bare airframe: rate"), Bare->ManualInput->FlightMode, EUBotManualFlightMode::Rate);
    return true;
}

#endif
