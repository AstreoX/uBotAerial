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
#include "UBotAerialWindTestTypes.h"
#include "UBotWindQuerySubsystem.h"
#include "UObject/StrongObjectPtr.h"

namespace UBotAerialWindInputTests
{
    using namespace UBotAerialTests;

    // Isolate the flight response from whichever optional environment the host has enabled.
    class FScopedWindProvider
    {
    public:
        UE_NONCOPYABLE(FScopedWindProvider);

        explicit FScopedWindProvider(const FScopedAerialTestWorld& Scope, const FVector& Velocity = FVector::ZeroVector)
            : Query(Scope.Get()->GetSubsystem<UUBotWindQuerySubsystem>())
            , Provider(NewObject<UUBotAerialTestWindProvider>(Scope.Get()))
        {
            check(Query);
            Scope.Get()->ForEachSubsystem<UWorldSubsystem>([this](UWorldSubsystem* Subsystem)
            {
                if (Cast<IUBotWindProvider>(Subsystem))
                {
                    PreviousProvider = Subsystem;
                    Query->UnregisterProvider(Subsystem);
                }
            });
            Provider->Velocity = Velocity;
            const bool bRegistered = Query->RegisterProvider(Provider.Get());
            check(bRegistered);
        }

        ~FScopedWindProvider()
        {
            Query->UnregisterProvider(Provider.Get());
            if (PreviousProvider.IsValid())
            {
                Query->RegisterProvider(PreviousProvider.Get());
            }
        }

        UUBotAerialTestWindProvider* Get() const { return Provider.Get(); }

    private:
        UUBotWindQuerySubsystem* Query;
        TStrongObjectPtr<UUBotAerialTestWindProvider> Provider;
        TWeakObjectPtr<UObject> PreviousProvider;
    };
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialWindSamplingTest, "UBotAerial.Wind.CoreSampling", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialWindSamplingTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialWindInputTests;
    FScopedAerialTestWorld Scope;
    FScopedWindProvider Wind(Scope);
    const FVector StartCentimeters(125.0, -250.0, 1000.0);
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(StartCentimeters);
    UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
    if (!TestNotNull(TEXT("Navigation drone"), Flight))
    {
        return false;
    }
    Scope.RunFor(0.1);
    TestTrue(TEXT("The flight model queries the Core provider"), Wind.Get()->Queries > 1);
    TestTrue(TEXT("Provider receives world position in centimeters"), Wind.Get()->LastLocationCentimeters.Equals(StartCentimeters, 1e-9));
    const double StepSeconds = 1.0 / Flight->PhysicsRateHz;
    TestEqual(TEXT("Provider receives the start time of the latest fixed physics step"),
        Wind.Get()->LastTimeSeconds, Flight->GetMultirotorState().SimTimeSeconds - StepSeconds, 1e-12);
    const int32 QueriesBeforeDisabling = Wind.Get()->Queries;
    Flight->bEnableWind = false;
    Scope.RunFor(0.1);
    TestEqual(TEXT("Disabling wind bypasses the provider"), Wind.Get()->Queries, QueriesBeforeDisabling);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialWindPhysicsTest, "UBotAerial.Wind.PhysicsSteady", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialWindPhysicsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialWindInputTests;
    FScopedAerialTestWorld Scope;
    FScopedWindProvider Wind(Scope, FVector(5.0, 0.0, 0.0));
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
    FScopedWindProvider Wind(Scope);
    AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 1000.0));
    UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
    if (!TestNotNull(TEXT("Navigation drone"), Flight))
    {
        return false;
    }
    Scope.RunFor(1.0);
    // A 3 m/s gust front arrives: the drone drifts, then compensates.
    Wind.Get()->Velocity = FVector(0.0, 3.0, 0.0);
    const FVector Start = Flight->GetMultirotorState().PositionMeters;
    double Drift = 0.0;
    Scope.RunFor(8.0, FrameSeconds, [&] { Drift = FMath::Max(Drift, (Flight->GetMultirotorState().PositionMeters - Start).Size()); });
    TestTrue(FString::Printf(TEXT("Drift bounded by 0.5 * 3 m/s * 1 s (%.3f)"), Drift), Drift > 1.0 && Drift <= 1.52);
    TestTrue(FString::Printf(TEXT("Compensated after 8 s (%.4f)"), Flight->GetMultirotorState().LinearVelocityMetersPerSecond.Size()),
        Flight->GetMultirotorState().LinearVelocityMetersPerSecond.Size() < 0.01);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialWindStepCollisionTest, "UBotAerial.Wind.StepCollision", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialWindStepCollisionTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialWindInputTests;
    FScopedAerialTestWorld Scope;
    FScopedWindProvider Wind(Scope);
    const FVector Normal = FVector(1.0, 1.0, 0.0).GetSafeNormal();
    const FVector StartCm(0.0, 0.0, 1000.0);
    constexpr double WallFaceCm = 44.5;
    constexpr double WallHalfDepthCm = 10.0;
    // Issue #5: the old free-space sphere stopped at 44.231 cm, before this diagonal wall.
    Scope.SpawnBox(StartCm + Normal * (WallFaceCm + WallHalfDepthCm),
        FVector(WallHalfDepthCm, 1000.0, 1000.0), FRotator(0.0, 45.0, 0.0));
    AUBotAerialTestNavigationDrone* Drone = Scope.Spawn<AUBotAerialTestNavigationDrone>(StartCm);
    UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
    if (!TestNotNull(TEXT("Navigation drone"), Flight))
    {
        return false;
    }
    Flight->PhysicsRateHz = 500.0f;
    Flight->WindResponse = 0.5f;
    Flight->WindRejectionTimeConstantSeconds = 1.0f;
    Drone->DispatchBeginPlay();
    TStrongObjectPtr<UUBotAerialTestListener> Listener(NewObject<UUBotAerialTestListener>());
    Flight->OnImpact.AddDynamic(Listener.Get(), &UUBotAerialTestListener::HandleImpact);

    // Initialize compensation in zero wind, then advance exactly eight 2 ms steps in one frame.
    Scope.Tick(0.032f);
    const int64 StepsBefore = Flight->GetPhysicsStepCount();
    Wind.Get()->Velocity = Normal * 5.0;
    Scope.Tick(0.016f);
    TestEqual(TEXT("Gust frame advances eight fixed steps"), Flight->GetPhysicsStepCount() - StepsBefore, int64(8));

    const FVector HalfExtentCm = Drone->CollisionBox->GetScaledBoxExtent();
    const double SupportCm = HalfExtentCm.X * Normal.X + HalfExtentCm.Y * Normal.Y;
    const FUBotMultirotorState State = Flight->GetMultirotorState();
    const double TravelCm = FVector::DotProduct(State.PositionMeters * 100.0 - StartCm, Normal);
    const double GapCm = WallFaceCm - SupportCm - TravelCm;
    TestTrue(FString::Printf(TEXT("Wind reaches the wall without penetrating (gap %.6f cm)"), GapCm), GapCm >= -1e-3 && GapCm <= 0.2);
    TestTrue(TEXT("Wind-driven velocity no longer points into the wall"), FVector::DotProduct(State.LinearVelocityMetersPerSecond, Normal) <= 1e-6);
    TestEqual(TEXT("Pure wind collision emits one impact"), Listener->ImpactSpeeds.Num(), 1);
    if (!Listener->ImpactSpeeds.IsEmpty())
    {
        TestTrue(TEXT("Impact uses the wind-driven approach speed"), Listener->ImpactSpeeds[0] > 2.4f);
        TestTrue(TEXT("Impact normal points away from the wall"), FVector::DotProduct(Listener->LastImpactNormal, Normal) < -0.999);
    }
    TestTrue(TEXT("Presented body also remains before the wall"),
        FVector::DotProduct(Drone->GetActorLocation() - StartCm, Normal) + SupportCm <= WallFaceCm + 1e-3);
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
