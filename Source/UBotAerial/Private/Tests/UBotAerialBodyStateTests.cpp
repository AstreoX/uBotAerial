#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/SceneComponent.h"
#include "UBotActuatorInterface.h"
#include "UBotAerial.h"
#include "UBotAerialTestTypes.h"
#include "UBotAerialTestWorld.h"
#include "UBotBodyState.h"
#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorMath.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotRotorComponent.h"
#include "UBotSimpleFlightComponent.h"

namespace UBotAerialBodyStateTests
{
    using namespace UBotAerialTests;

    // The physics clock puts a step up to 1e-6 of a step early, so physics time may pass sim time by this much.
    constexpr double TimeToleranceSeconds = 1.0e-7;

    FString Labeled(const TCHAR* Label, const FString& What)
    {
        return FString::Printf(TEXT("%s: %s"), Label, *What);
    }

    // 1 - |p . q| is about the square of half the angle between two unit quaternions; it ignores their sign.
    double RotationError(const FQuat& A, const FQuat& B)
    {
        return 1.0 - FMath::Abs(A | B);
    }

    bool SameState(const FUBotBodyState& A, const FUBotBodyState& B)
    {
        return A.PositionMeters == B.PositionMeters && A.Orientation.Equals(B.Orientation, 0.0)
            && A.LinearVelocityMetersPerSecond == B.LinearVelocityMetersPerSecond
            && A.AngularVelocityRadiansPerSecond == B.AngularVelocityRadiansPerSecond
            && A.LinearAccelerationMetersPerSecondSquared == B.LinearAccelerationMetersPerSecondSquared
            && A.AngularAccelerationRadiansPerSecondSquared == B.AngularAccelerationRadiansPerSecondSquared
            && A.bHasLinearAcceleration == B.bHasLinearAcceleration && A.bHasAngularAcceleration == B.bHasAngularAcceleration
            && A.TimestampSeconds == B.TimestampSeconds && A.Source == B.Source;
    }

    // What the frames of one test drone showed about the state it reports.
    struct FIntegratorChecks
    {
        bool bAlwaysAvailable = true;
        bool bAlwaysSameAsIntegrator = true;
        bool bAlwaysSameFromChild = true;
        bool bTimingHolds = true;
        double MaxWorldRateError = 0.0;
        double MaxLocationErrorCm = 0.0;
        double MaxRotationError = 0.0;
        // Frames in which the drone moves and physics lags sim time, and those among them in which the component is away from the state.
        int32 LaggingFrames = 0;
        int32 PresentedAwayFrames = 0;
    };

    // Compares the body state with the movement component's own integrator state and with the pose it presents.
    void CheckIntegratorFrame(FIntegratorChecks& Checks, const FScopedAerialTestWorld& Scope, const UUBotMultirotorMovementComponent& Movement,
        const USceneComponent& Root, const USceneComponent& Child)
    {
        FUBotBodyState State;
        if (!Movement.GetBodyState(&Root, State))
        {
            Checks.bAlwaysAvailable = false;
            return;
        }
        const FUBotMultirotorState Integrator = Movement.GetMultirotorState();
        const double StepSeconds = Movement.GetPhysicsClock().GetStepSeconds();

        // The same numbers as the integrator's, not copies of the presented pose.
        const bool bSameNumbers = State.PositionMeters == Integrator.PositionMeters && State.Orientation.Equals(Integrator.Orientation, 0.0)
            && State.LinearVelocityMetersPerSecond == Integrator.LinearVelocityMetersPerSecond
            && State.LinearAccelerationMetersPerSecondSquared == Integrator.LinearAccelerationMetersPerSecondSquared
            && State.TimestampSeconds == Integrator.SimTimeSeconds && State.TimestampSeconds == Movement.GetPhysicsClock().GetTimeSeconds()
            && State.bHasLinearAcceleration && State.bHasAngularAcceleration && State.Source == UBot::Aerial::PackageName;
        if (!bSameNumbers)
        {
            Checks.bAlwaysSameAsIntegrator = false;
        }

        // The integrator keeps the body rate in the body frame; the state has it in the world frame.
        const FVector WorldRate = Integrator.Orientation.RotateVector(Integrator.AngularVelocityRadiansPerSecond);
        Checks.MaxWorldRateError = FMath::Max(Checks.MaxWorldRateError, (State.AngularVelocityRadiansPerSecond - WorldRate).Size());

        // Physics time is at or before sim time and less than a step behind it.
        const double Now = Scope.Now();
        const double Ahead = Now - State.TimestampSeconds;
        if (Ahead < -TimeToleranceSeconds || Ahead >= StepSeconds + TimeToleranceSeconds)
        {
            Checks.bTimingHolds = false;
        }

        // The component shows the state extrapolated to sim time.
        const double Extrapolation = FMath::Clamp(Ahead, 0.0, StepSeconds);
        const FVector ExpectedLocationCm = (State.PositionMeters + State.LinearVelocityMetersPerSecond * Extrapolation) * 100.0;
        const FQuat ExpectedRotation = UBot::Multirotor::IntegrateBodyRate(State.Orientation, Integrator.AngularVelocityRadiansPerSecond, Extrapolation);
        Checks.MaxLocationErrorCm = FMath::Max(Checks.MaxLocationErrorCm, FVector::Dist(Root.GetComponentLocation(), ExpectedLocationCm));
        Checks.MaxRotationError = FMath::Max(Checks.MaxRotationError, RotationError(Root.GetComponentQuat(), ExpectedRotation));

        // While the drone moves and physics lags, the component is not where the state is.
        if (Extrapolation > 2.0e-4 && Integrator.LinearVelocityMetersPerSecond.Size() > 0.3)
        {
            ++Checks.LaggingFrames;
            if (FVector::Dist(Root.GetComponentLocation(), State.PositionMeters * 100.0) > 1.0e-3)
            {
                ++Checks.PresentedAwayFrames;
            }
        }

        // A sensor on a child component reads the same state.
        FUBotBodyState FromChild;
        const USceneComponent* NearestBody = nullptr;
        if (!UBot::BodyState::GetNearestBodyState(&Child, FromChild, &NearestBody) || NearestBody != &Root || !SameState(FromChild, State))
        {
            Checks.bAlwaysSameFromChild = false;
        }
    }

    // Frames of exactly one physics step, comparing the angular acceleration and the angular velocity of
    // the state with those of the previous frame.
    struct FStepProbe
    {
        int32 Frames = 0;
        int32 Compared = 0;
        bool bAlwaysAvailable = true;
        double MaxAccelerationError = 0.0;
        double MaxRateError = 0.0;
        double PeakAcceleration = 0.0;
        double PeakRate = 0.0;
        FUBotBodyState Previous;
        int64 PreviousSteps = 0;
        bool bHavePrevious = false;
    };

    void RunStepFrames(FStepProbe& Probe, const FScopedAerialTestWorld& Scope, const UUBotMultirotorMovementComponent& Movement, const USceneComponent& Body,
        int32 FrameCount)
    {
        const double StepSeconds = Movement.GetPhysicsClock().GetStepSeconds();
        for (int32 Frame = 0; Frame < FrameCount; ++Frame)
        {
            Scope.Tick(static_cast<float>(StepSeconds));
            ++Probe.Frames;
            FUBotBodyState Current;
            if (!Movement.GetBodyState(&Body, Current))
            {
                Probe.bAlwaysAvailable = false;
                Probe.bHavePrevious = false;
                continue;
            }
            const int64 Steps = Movement.GetPhysicsStepCount();
            if (Probe.bHavePrevious && Steps == Probe.PreviousSteps + 1)
            {
                // The acceleration of the step: the world angular velocity after it minus the one before it.
                const FVector ExpectedAcceleration = (Current.AngularVelocityRadiansPerSecond - Probe.Previous.AngularVelocityRadiansPerSecond) / StepSeconds;
                Probe.MaxAccelerationError = FMath::Max(Probe.MaxAccelerationError, (Current.AngularAccelerationRadiansPerSecondSquared - ExpectedAcceleration).Size());

                // The world rotation rate of the orientation over the step, from the orientations alone.
                const FVector BodyRate = UBot::Multirotor::BodyRateBetween(Probe.Previous.Orientation, Current.Orientation, StepSeconds);
                const FVector OrientationRate = Probe.Previous.Orientation.RotateVector(BodyRate);
                Probe.MaxRateError = FMath::Max(Probe.MaxRateError, (Current.AngularVelocityRadiansPerSecond - OrientationRate).Size());

                Probe.PeakAcceleration = FMath::Max(Probe.PeakAcceleration, Current.AngularAccelerationRadiansPerSecondSquared.Size());
                Probe.PeakRate = FMath::Max(Probe.PeakRate, Current.AngularVelocityRadiansPerSecond.Size());
                ++Probe.Compared;
            }
            Probe.Previous = Current;
            Probe.PreviousSteps = Steps;
            Probe.bHavePrevious = true;
        }
    }

    // What a run in single-step frames must show: the state in every frame, one step per frame, and the orientation turning at the reported rate.
    void ExpectStepProbe(FAutomationTestBase& Test, const TCHAR* Label, const FStepProbe& Probe)
    {
        Test.TestTrue(Labeled(Label, TEXT("The state is available in every frame")), Probe.bAlwaysAvailable);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("Every frame took exactly one step (%d of %d compared)"), Probe.Compared, Probe.Frames - 1)),
            Probe.Compared == Probe.Frames - 1);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("The orientation turns at the reported world angular velocity (error %g rad/s, peak %g)"),
            Probe.MaxRateError, Probe.PeakRate)), Probe.MaxRateError <= 1.0e-6 && Probe.PeakRate > 0.1);
    }

    // The checks of "ProvidesUpdatedComponent" for one kind of test drone.
    template <typename TDrone>
    void CheckProvidesUpdatedComponent(FAutomationTestBase& Test, const TCHAR* Label)
    {
        FScopedAerialTestWorld Scope;
        TDrone* Drone = Scope.Spawn<TDrone>(FVector(0.0, 0.0, 1000.0));
        UUBotMultirotorMovementComponent* Movement = Drone ? Drone->GetMultirotorMovement() : nullptr;
        USceneComponent* Root = Drone ? Drone->GetRootComponent() : nullptr;
        if (!Test.TestNotNull(Labeled(Label, TEXT("Movement component")), Movement) || !Test.TestNotNull(Labeled(Label, TEXT("Root component")), Root))
        {
            return;
        }
        Test.TestTrue(Labeled(Label, TEXT("The movement component updates the root")), Movement->UpdatedComponent.Get() == Root);

        // Not a body that something else owns: the other components of the drone and the root of another actor.
        AActor* OtherActor = Scope.SpawnBox(FVector(500.0, 0.0, 1000.0), FVector(50.0, 50.0, 50.0));
        TArray<const USceneComponent*> Others;
        Others.Add(Drone->VisualRoot.Get());
        for (UUBotRotorComponent* Rotor : Drone->Rotors)
        {
            Others.Add(Rotor);
        }
        if (Test.TestNotNull(Labeled(Label, TEXT("Other actor")), OtherActor))
        {
            Others.Add(OtherActor->GetRootComponent());
        }

        // The claim holds from registration on, before begin play, and there is no state yet.
        FUBotBodyState State;
        Test.TestTrue(Labeled(Label, TEXT("It provides the root before begin play")), Movement->ProvidesBodyState(Root));
        Test.TestFalse(Labeled(Label, TEXT("It has no state before the flight starts")), Movement->GetBodyState(Root, State));
        Test.TestTrue(Labeled(Label, TEXT("FindSource finds it for the root before begin play")), UBot::BodyState::FindSource(Root) == Movement);
        const USceneComponent* NearestBody = nullptr;
        Test.TestFalse(Labeled(Label, TEXT("There is no nearest state before the flight starts")), UBot::BodyState::GetNearestBodyState(Root, State, &NearestBody));
        Test.TestTrue(Labeled(Label, TEXT("But the nearest body is already the root")), NearestBody == Root);
        Test.TestFalse(Labeled(Label, TEXT("It provides no null body")), Movement->ProvidesBodyState(nullptr));
        for (const USceneComponent* Other : Others)
        {
            Test.TestFalse(Labeled(Label, FString::Printf(TEXT("It does not provide %s before begin play"), *GetNameSafe(Other))), Movement->ProvidesBodyState(Other));
            Test.TestTrue(Labeled(Label, FString::Printf(TEXT("FindSource finds nothing for %s"), *GetNameSafe(Other))), UBot::BodyState::FindSource(Other) == nullptr);
        }

        // Once the flight started the root has a state; the other components still have none.
        Drone->DispatchBeginPlay();
        Test.TestTrue(Labeled(Label, TEXT("The root has a state once the flight started")), Movement->GetBodyState(Root, State));
        Test.TestTrue(Labeled(Label, TEXT("The state names the plugin as its source")), State.Source == UBot::Aerial::PackageName);
        for (const USceneComponent* Other : Others)
        {
            Test.TestFalse(Labeled(Label, FString::Printf(TEXT("It does not provide %s after begin play"), *GetNameSafe(Other))), Movement->ProvidesBodyState(Other));
            Test.TestFalse(Labeled(Label, FString::Printf(TEXT("%s has no state"), *GetNameSafe(Other))), Movement->GetBodyState(Other, State));
            Test.TestTrue(Labeled(Label, FString::Printf(TEXT("FindSource still finds nothing for %s"), *GetNameSafe(Other))), UBot::BodyState::FindSource(Other) == nullptr);
        }

        // From a part of the drone the nearest body is the root.
        const USceneComponent* Body = nullptr;
        if (Test.TestTrue(Labeled(Label, TEXT("A rotor exists")), Drone->Rotors.Num() > 0 && Drone->Rotors[0] != nullptr))
        {
            const UObject* Source = UBot::BodyState::FindSourceInParents(Drone->Rotors[0].Get(), Body);
            Test.TestTrue(Labeled(Label, TEXT("The source found above a rotor is the movement component")), Source == Movement);
            Test.TestTrue(Labeled(Label, TEXT("And the body it provides is the root")), Body == Root);
        }
    }

    // Flies a velocity command with a yaw rate and checks, frame by frame, that the body state is the integrator's.
    template <typename TDrone>
    void CheckIntegratorState(FAutomationTestBase& Test, const TCHAR* Label)
    {
        FScopedAerialTestWorld Scope;
        TDrone* Drone = Scope.SpawnAndBegin<TDrone>(FVector(0.0, 0.0, 1000.0));
        UUBotMultirotorMovementComponent* Movement = Drone ? Drone->GetMultirotorMovement() : nullptr;
        USceneComponent* Root = Drone ? Drone->GetRootComponent() : nullptr;
        const bool bHasRotor = Drone && Drone->Rotors.Num() > 0;
        if (!Test.TestNotNull(Labeled(Label, TEXT("Movement component")), Movement) || !Test.TestNotNull(Labeled(Label, TEXT("Root component")), Root)
            || !Test.TestTrue(Labeled(Label, TEXT("Rotor")), bHasRotor))
        {
            return;
        }
        const USceneComponent* Rotor = Drone->Rotors[0].Get();
        if (!Test.TestTrue(Labeled(Label, TEXT("Velocity command accepted")), SendVelocity(*Movement, FVector(2.0, 0.5, 0.3), 0.4)))
        {
            return;
        }

        // Frames of 1/60 s: physics runs a fraction of a step behind sim time in most of them.
        FIntegratorChecks Checks;
        Scope.RunFor(1.0, FrameSeconds, [&] { CheckIntegratorFrame(Checks, Scope, *Movement, *Root, *Rotor); });

        Test.TestTrue(Labeled(Label, TEXT("The root has a state in every frame")), Checks.bAlwaysAvailable);
        Test.TestTrue(Labeled(Label, TEXT("It is the integrator's position, orientation, velocity, acceleration and time")), Checks.bAlwaysSameAsIntegrator);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("Its angular velocity is the body rate rotated to the world frame (error %g)"), Checks.MaxWorldRateError)),
            Checks.MaxWorldRateError <= 1.0e-12);
        Test.TestTrue(Labeled(Label, TEXT("Physics time is at or before sim time and less than a step behind it")), Checks.bTimingHolds);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("The component shows the state extrapolated to sim time (%g cm, rotation %g)"),
            Checks.MaxLocationErrorCm, Checks.MaxRotationError)), Checks.MaxLocationErrorCm <= 1.0e-6 && Checks.MaxRotationError <= 1.0e-12);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("Physics lagged sim time in %d frames of a moving drone"), Checks.LaggingFrames)), Checks.LaggingFrames >= 10);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("In all of them the component is away from the state (%d of %d)"), Checks.PresentedAwayFrames, Checks.LaggingFrames)),
            Checks.PresentedAwayFrames == Checks.LaggingFrames);
        Test.TestTrue(Labeled(Label, TEXT("A child component of the drone gets the same state")), Checks.bAlwaysSameFromChild);

        // Frames of one step: the angular velocity of the state is the world rotation rate of its orientation.
        FStepProbe Probe;
        RunStepFrames(Probe, Scope, *Movement, *Root, 60);
        ExpectStepProbe(Test, Label, Probe);
    }

    // A drone rolled to a large angle turns about its body Y axis, which then points partly up: the state's
    // angular velocity must be in the world frame.
    void CheckRolledDroneAngularVelocity(FAutomationTestBase& Test)
    {
        const TCHAR* Label = TEXT("Rolled bare airframe");
        FScopedAerialTestWorld Scope;
        AUBotAerialTestBareDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestBareDrone>(FVector(0.0, 0.0, 5000.0));
        UUBotMultirotorDynamicsComponent* Dynamics = Drone ? Drone->GetDynamics() : nullptr;
        USceneComponent* Root = Drone ? Drone->GetRootComponent() : nullptr;
        if (!Test.TestNotNull(Labeled(Label, TEXT("Dynamics component")), Dynamics) || !Test.TestNotNull(Labeled(Label, TEXT("Root component")), Root))
        {
            return;
        }

        FUBotBodyRateCommand Rates;
        Rates.BodyRateRadiansPerSecond = FVector(1.5, 0.0, 0.0);
        IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Dynamics, Rates);
        Scope.RunFor(0.5);
        Rates.BodyRateRadiansPerSecond = FVector(0.0, 1.5, 0.0);
        IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Dynamics, Rates);
        Scope.RunFor(0.25);

        FStepProbe Probe;
        RunStepFrames(Probe, Scope, *Dynamics, *Root, 20);
        ExpectStepProbe(Test, Label, Probe);

        FUBotBodyState State;
        if (!Test.TestTrue(Labeled(Label, TEXT("The root has a state")), Dynamics->GetBodyState(Root, State)))
        {
            return;
        }
        const FUBotMultirotorState Integrator = Dynamics->GetMultirotorState();
        const FVector WorldRate = Integrator.Orientation.RotateVector(Integrator.AngularVelocityRadiansPerSecond);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("The state has the body rate rotated to the world frame (%s vs %s)"),
            *State.AngularVelocityRadiansPerSecond.ToString(), *WorldRate.ToString())), State.AngularVelocityRadiansPerSecond.Equals(WorldRate, 1.0e-12));
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("The rolled drone turns partly about the world vertical axis (%s)"), *State.AngularVelocityRadiansPerSecond.ToString())),
            FMath::Abs(State.AngularVelocityRadiansPerSecond.Z) > 0.3);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("So the world vector is not the body vector (%s vs %s)"), *State.AngularVelocityRadiansPerSecond.ToString(),
            *Integrator.AngularVelocityRadiansPerSecond.ToString())), (State.AngularVelocityRadiansPerSecond - Integrator.AngularVelocityRadiansPerSecond).Size() > 0.3);
    }

    // The angular acceleration of a probe run equals the change of the world angular velocity over each step.
    void ExpectAngularAcceleration(FAutomationTestBase& Test, const TCHAR* Label, const FStepProbe& Probe)
    {
        ExpectStepProbe(Test, Label, Probe);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("The angular acceleration equals the world angular velocity change over the step (max error %g rad/s^2)"),
            Probe.MaxAccelerationError)), Probe.MaxAccelerationError <= 1.0e-9);
        Test.TestTrue(Labeled(Label, FString::Printf(TEXT("The run really accelerates (peak %g rad/s^2)"), Probe.PeakAcceleration)), Probe.PeakAcceleration > 1.0);
    }

    // A navigation drone changing its yaw rate: the base component derives the acceleration for every flight model.
    void CheckNavigationAngularAcceleration(FAutomationTestBase& Test)
    {
        const TCHAR* Label = TEXT("Navigation drone");
        FScopedAerialTestWorld Scope;
        AUBotAerialTestNavigationDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestNavigationDrone>(FVector(0.0, 0.0, 1000.0));
        UUBotSimpleFlightComponent* Flight = Drone ? Drone->GetSimpleFlight() : nullptr;
        USceneComponent* Root = Drone ? Drone->GetRootComponent() : nullptr;
        if (!Test.TestNotNull(Labeled(Label, TEXT("Simple flight component")), Flight) || !Test.TestNotNull(Labeled(Label, TEXT("Root component")), Root))
        {
            return;
        }
        FStepProbe Probe;
        SendVelocity(*Flight, FVector::ZeroVector, 0.8);
        RunStepFrames(Probe, Scope, *Flight, *Root, 200);
        SendVelocity(*Flight, FVector::ZeroVector, -0.8);
        RunStepFrames(Probe, Scope, *Flight, *Root, 400);
        ExpectAngularAcceleration(Test, Label, Probe);
    }

    // A dynamics drone under body-rate commands. Wherever the linear acceleration is zeroed, the angular
    // acceleration is zeroed with it: a teleport that stops the motion, and a flight reset.
    void CheckDynamicsAngularAcceleration(FAutomationTestBase& Test)
    {
        const TCHAR* Label = TEXT("Dynamics drone");
        FScopedAerialTestWorld Scope;
        AUBotAerialTestBareDrone* Drone = Scope.SpawnAndBegin<AUBotAerialTestBareDrone>(FVector(0.0, 0.0, 5000.0));
        UUBotMultirotorDynamicsComponent* Dynamics = Drone ? Drone->GetDynamics() : nullptr;
        USceneComponent* Root = Drone ? Drone->GetRootComponent() : nullptr;
        if (!Test.TestNotNull(Labeled(Label, TEXT("Dynamics component")), Dynamics) || !Test.TestNotNull(Labeled(Label, TEXT("Root component")), Root))
        {
            return;
        }
        Test.TestTrue(Labeled(Label, TEXT("It steps at 1 kHz")), FMath::IsNearlyEqual(Dynamics->GetPhysicsClock().GetStepSeconds(), 0.001, 1.0e-12));

        FUBotBodyRateCommand Rates;
        const auto SendBodyRate = [&](const FVector& BodyRate) {
            Rates.BodyRateRadiansPerSecond = BodyRate;
            IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Dynamics, Rates);
        };

        FStepProbe Probe;
        RunStepFrames(Probe, Scope, *Dynamics, *Root, 20);
        SendBodyRate(FVector(1.5, 0.0, 0.0));
        RunStepFrames(Probe, Scope, *Dynamics, *Root, 150);
        SendBodyRate(FVector(0.0, 1.5, 0.0));
        RunStepFrames(Probe, Scope, *Dynamics, *Root, 150);
        SendBodyRate(FVector(-0.5, -1.0, 1.0));
        RunStepFrames(Probe, Scope, *Dynamics, *Root, 150);
        SendBodyRate(FVector::ZeroVector);
        RunStepFrames(Probe, Scope, *Dynamics, *Root, 100);
        ExpectAngularAcceleration(Test, Label, Probe);

        // Right after a rate change the drone accelerates; after the reset or the teleport nothing moves.
        const auto ExpectAccelerating = [&](const TCHAR* Case) {
            FUBotBodyState Moving;
            if (!Test.TestTrue(Labeled(Label, FString::Printf(TEXT("%s: the root has a state"), Case)), Dynamics->GetBodyState(Root, Moving)))
            {
                return;
            }
            const double Acceleration = Moving.AngularAccelerationRadiansPerSecondSquared.Size();
            Test.TestTrue(Labeled(Label, FString::Printf(TEXT("%s: the drone accelerates angularly before (%g rad/s^2)"), Case, Acceleration)), Acceleration > 0.1);
        };
        const auto ExpectAtRest = [&](const TCHAR* Case) {
            FUBotBodyState AtRest;
            if (!Test.TestTrue(Labeled(Label, FString::Printf(TEXT("%s: the root has a state"), Case)), Dynamics->GetBodyState(Root, AtRest)))
            {
                return;
            }
            Test.TestTrue(Labeled(Label, FString::Printf(TEXT("%s: the angular acceleration is zero"), Case)), AtRest.AngularAccelerationRadiansPerSecondSquared.IsZero());
            Test.TestTrue(Labeled(Label, FString::Printf(TEXT("%s: the linear acceleration is zero"), Case)), AtRest.LinearAccelerationMetersPerSecondSquared.IsZero());
            Test.TestTrue(Labeled(Label, FString::Printf(TEXT("%s: the angular velocity is zero"), Case)), AtRest.AngularVelocityRadiansPerSecond.IsZero());
        };

        FStepProbe Scratch;
        SendBodyRate(FVector(-1.5, 0.0, 0.0));
        RunStepFrames(Scratch, Scope, *Dynamics, *Root, 10);
        ExpectAccelerating(TEXT("Teleport"));
        Dynamics->TeleportMultirotor(FVector(0.0, 0.0, 3000.0), FRotator::ZeroRotator, true);
        ExpectAtRest(TEXT("Teleport"));

        SendBodyRate(FVector(1.5, 0.0, 0.0));
        RunStepFrames(Scratch, Scope, *Dynamics, *Root, 10);
        ExpectAccelerating(TEXT("Reset Flight"));
        Dynamics->ResetFlight();
        ExpectAtRest(TEXT("Reset Flight"));
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialBodyStateProvidesTest, "UBotAerial.BodyState.ProvidesUpdatedComponent", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialBodyStateProvidesTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialBodyStateTests;
    CheckProvidesUpdatedComponent<AUBotAerialTestNavigationDrone>(*this, TEXT("Navigation drone"));
    CheckProvidesUpdatedComponent<AUBotAerialTestPhysicsDrone>(*this, TEXT("Physics drone"));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialBodyStateIntegratorStateTest, "UBotAerial.BodyState.IntegratorState", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialBodyStateIntegratorStateTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialBodyStateTests;
    CheckIntegratorState<AUBotAerialTestNavigationDrone>(*this, TEXT("Navigation drone"));
    CheckIntegratorState<AUBotAerialTestPhysicsDrone>(*this, TEXT("Physics drone"));
    CheckRolledDroneAngularVelocity(*this);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialBodyStateAngularAccelerationTest, "UBotAerial.BodyState.AngularAcceleration", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialBodyStateAngularAccelerationTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialBodyStateTests;
    CheckDynamicsAngularAcceleration(*this);
    CheckNavigationAngularAcceleration(*this);
    return true;
}

#endif
