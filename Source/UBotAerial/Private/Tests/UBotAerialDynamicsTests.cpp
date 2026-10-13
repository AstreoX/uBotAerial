#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include <limits>

#include "UBotAerial.h"
#include "UBotAerialTestTypes.h"
#include "UBotAerialTestWorld.h"
#include "UBotExtensionRegistry.h"
#include "UBotMultirotorDynamics.h"
#include "UBotMultirotorDynamicsBackend.h"
#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorMath.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotRotorComponent.h"
#include "UObject/StrongObjectPtr.h"

namespace UBotAerialDynamicsTests
{
    using namespace UBotAerialTests;

    // A physics drone (or a bare airframe) hovering at HeightCm in an empty world.
    template <typename TDrone>
    struct FFlyingDrone
    {
        FScopedAerialTestWorld Scope;
        TDrone* Drone = nullptr;
        UUBotMultirotorDynamicsComponent* Dynamics = nullptr;

        explicit FFlyingDrone(double HeightCm = 1000.0, TFunctionRef<void(UUBotMultirotorDynamicsComponent&)> Configure = [](UUBotMultirotorDynamicsComponent&) {})
        {
            Drone = Scope.Spawn<TDrone>(FVector(0.0, 0.0, HeightCm));
            Dynamics = Drone ? Drone->GetDynamics() : nullptr;
            if (Dynamics)
            {
                Configure(*Dynamics);
                Drone->DispatchBeginPlay();
            }
        }

        bool IsValid() const { return Dynamics != nullptr; }
        FUBotMultirotorState State() const { return Dynamics->GetMultirotorState(); }
        FVector Position() const { return State().PositionMeters; }
        FVector Velocity() const { return State().LinearVelocityMetersPerSecond; }
        double Yaw() const { return UBot::Multirotor::GetYaw(State().Orientation); }
    };

    using FPhysics = FFlyingDrone<AUBotAerialTestPhysicsDrone>;
    using FBare = FFlyingDrone<AUBotAerialTestBareDrone>;

    double HoverRotorSpeed(const FScopedAerialTestWorld& Scope)
    {
        const FUBotMultirotorAirframe Airframe;
        return FMath::Sqrt(Airframe.MassKilograms * Scope.Gravity().Size() / 4.0 / Airframe.Rotors[0].ThrustCoefficient);
    }

    FUBotMotorCommand MotorCommand(double Value, int32 Count = 4)
    {
        FUBotMotorCommand Command;
        Command.Values.Init(Value, Count);
        Command.Source = TEXT("Test");
        return Command;
    }

    // A backend that counts its steps and integrates with the built-in model.
    class FCountingBackend final : public IUBotMultirotorDynamicsBackend
    {
    public:
        explicit FCountingBackend(int32& InSteps) : Steps(InSteps) {}
        virtual void Reset(const FUBotMultirotorAirframe& InAirframe, const UBot::Multirotor::FRigidBodyState& State) override { Airframe = InAirframe; }
        virtual void Step(TConstArrayView<double> Commands, const FUBotMultirotorEnvironment& Environment, double StepSeconds,
            UBot::Multirotor::FRigidBodyState& State) override
        {
            ++Steps;
            UBot::Multirotor::StepRigidBody(Airframe, Commands, Environment.Gravity, Environment.Wind, StepSeconds, State);
        }

    private:
        int32& Steps;
        FUBotMultirotorAirframe Airframe;
    };

    class FCountingBackendFactory final : public IUBotMultirotorBackendFactory
    {
    public:
        int32 Steps = 0;
        virtual FName GetExtensionName() const override { return TEXT("UBotAerialTestBackend"); }
        virtual TSharedPtr<IUBotMultirotorDynamicsBackend> CreateBackend(const FUBotMultirotorAirframe& Airframe) const override
        {
            return MakeShared<FCountingBackend>(const_cast<int32&>(Steps));
        }
    };
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDynamicsAllocationTest, "UBotAerial.Math.ThrustAllocation", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDynamicsAllocationTest::RunTest(const FString& Parameters)
{
    const FUBotMultirotorAirframe Airframe;
    UBot::Multirotor::FThrustAllocator Allocator;
    if (!TestTrue(TEXT("Allocator builds for the default quad"), Allocator.Build(Airframe)))
    {
        return false;
    }
    TestEqual(TEXT("Max total thrust"), Allocator.GetMaxTotalThrust(), 4.0 * 5.84e-6 * 1100.0 * 1100.0, 1e-9);

    // A request that fits comes back exactly when the rotor speeds produce it.
    const double Thrust = 15.0;
    const FVector Torque(0.3, -0.2, 0.05);
    TArray<double> Commands;
    Allocator.Allocate(Thrust, Torque, Commands);
    TArray<double> Speeds;
    for (int32 Index = 0; Index < Commands.Num(); ++Index)
    {
        Speeds.Add(UBot::Multirotor::CommandToRotorSpeed(Airframe.Rotors[Index], Airframe.MotorCommandMapping, Commands[Index]));
    }
    double OutThrust = 0.0;
    FVector OutTorque;
    UBot::Multirotor::ComputeRotorWrench(Airframe, Speeds, OutThrust, OutTorque);
    TestEqual(TEXT("Round trip thrust"), OutThrust, Thrust, 1e-9);
    TestTrue(FString::Printf(TEXT("Round trip torque (%s)"), *OutTorque.ToString()), OutTorque.Equals(Torque, 1e-9));

    // Physical signs: more thrust on the right rolls left (+X in the UE sense), more thrust at the front pitches up (-Y).
    double RightThrust = 0.0;
    FVector RightTorque;
    UBot::Multirotor::ComputeRotorWrench(Airframe, TArray<double>{ 800.0, 700.0, 700.0, 800.0 }, RightThrust, RightTorque);
    TestTrue(TEXT("Right side up is +X torque"), RightTorque.X > 0.0);
    double FrontThrust = 0.0;
    FVector FrontTorque;
    UBot::Multirotor::ComputeRotorWrench(Airframe, TArray<double>{ 800.0, 700.0, 800.0, 700.0 }, FrontThrust, FrontTorque);
    TestTrue(TEXT("Nose up is -Y torque"), FrontTorque.Y < 0.0);
    double SpinThrust = 0.0;
    FVector SpinTorque;
    UBot::Multirotor::ComputeRotorWrench(Airframe, TArray<double>{ 700.0, 700.0, 800.0, 800.0 }, SpinThrust, SpinTorque);
    TestTrue(TEXT("Faster clockwise rotors yaw the body left (-Z)"), SpinTorque.Z < 0.0);

    // Too much yaw: yaw gives way, roll and pitch stay, every command stays in range.
    Allocator.Allocate(Thrust, FVector(0.2, 0.0, 5.0), Commands);
    for (const double Command : Commands)
    {
        TestTrue(TEXT("Command within 0..1"), Command >= 0.0 && Command <= 1.0);
    }
    Speeds.Reset();
    for (int32 Index = 0; Index < Commands.Num(); ++Index)
    {
        Speeds.Add(UBot::Multirotor::CommandToRotorSpeed(Airframe.Rotors[Index], Airframe.MotorCommandMapping, Commands[Index]));
    }
    UBot::Multirotor::ComputeRotorWrench(Airframe, Speeds, OutThrust, OutTorque);
    TestEqual(TEXT("Roll kept while yaw saturates"), OutTorque.X, 0.2, 1e-6);
    TestEqual(TEXT("Thrust kept while yaw saturates"), OutThrust, Thrust, 1e-6);
    TestTrue(TEXT("Yaw reduced, same sign"), OutTorque.Z > 0.0 && OutTorque.Z < 5.0);

    TestEqual(TEXT("Hover command"), UBot::Multirotor::GetHoverCommand(Airframe, 9.8), 1.5 * 9.8 / (4.0 * 5.84e-6 * 1100.0 * 1100.0), 1e-12);
    FString Error;
    FUBotMultirotorAirframe Broken;
    Broken.Rotors.Reset();
    TestFalse(TEXT("No rotors is invalid"), UBot::Multirotor::IsAirframeValid(Broken, &Error));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDynamicsHoverTest, "UBotAerial.Dynamics.Hover10s", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDynamicsHoverTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FPhysics Drone;
    if (!TestTrue(TEXT("Physics drone spawned"), Drone.IsValid()))
    {
        return false;
    }
    const FVector Start = Drone.Position();
    double MaxDz = 0.0;
    double MaxDxy = 0.0;
    double MaxTilt = 0.0;
    FFrameImuProbe Probe;
    double MaxProbeError = 0.0;
    Drone.Scope.RunFor(10.0, FrameSeconds, [&] {
        const FUBotMultirotorState State = Drone.State();
        MaxDz = FMath::Max(MaxDz, FMath::Abs(State.PositionMeters.Z - Start.Z));
        MaxDxy = FMath::Max(MaxDxy, FVector2D(State.PositionMeters - Start).Size());
        MaxTilt = FMath::Max(MaxTilt, UBot::Multirotor::GetTilt(State.Orientation));
        if (Probe.Sample(*Drone.Drone->GetRootComponent(), Drone.Scope.Now(), Drone.Scope.Gravity()))
        {
            MaxProbeError = FMath::Max(MaxProbeError, (Probe.SpecificForce + Drone.Scope.Gravity()).Size());
        }
    });
    TestTrue(FString::Printf(TEXT("Altitude within 1 cm (%.4f cm)"), MaxDz * 100.0), MaxDz <= 0.01);
    TestTrue(FString::Printf(TEXT("Position within 0.5 cm (%.4f cm)"), MaxDxy * 100.0), MaxDxy <= 0.005);
    TestTrue(FString::Printf(TEXT("Level within 0.1 deg (%.5f)"), FMath::RadiansToDegrees(MaxTilt)), FMath::RadiansToDegrees(MaxTilt) <= 0.1);
    const FUBotMultirotorState State = Drone.State();
    const double Hover = HoverRotorSpeed(Drone.Scope);
    if (TestEqual(TEXT("Four rotor speeds"), State.RotorSpeedsRadiansPerSecond.Num(), 4))
    {
        for (const double Speed : State.RotorSpeedsRadiansPerSecond)
        {
            TestTrue(FString::Printf(TEXT("Rotor at hover speed (%.2f vs %.2f)"), Speed, Hover), FMath::Abs(Speed / Hover - 1.0) <= 0.005);
        }
    }
    TestTrue(TEXT("State specific force is +g"), State.SpecificForceMetersPerSecondSquared.Equals(-Drone.Scope.Gravity(), 0.01));
    TestTrue(FString::Printf(TEXT("Frame-sampled IMU reads +g (%.4f)"), MaxProbeError), MaxProbeError <= 0.05);
    TestTrue(TEXT("Rotors spin"), Drone.Drone->Rotors[0]->GetRotorSpeed() > 700.0f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDynamicsVelocityStepTest, "UBotAerial.Dynamics.VelocityStep", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDynamicsVelocityStepTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FPhysics Drone;
    if (!TestTrue(TEXT("Physics drone spawned"), Drone.IsValid()))
    {
        return false;
    }
    TestTrue(TEXT("Velocity command accepted"), SendVelocity(*Drone.Dynamics, FVector(2.0, 0.0, 0.0)));
    TestEqual(TEXT("Velocity mode"), Drone.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::Velocity);
    double MaxSpeed = 0.0;
    double MaxSideways = 0.0;
    double MaxDz = 0.0;
    double MaxTilt = 0.0;
    double SpeedAt2s = 0.0;
    const auto Track = [&] {
        const FUBotMultirotorState State = Drone.State();
        MaxSpeed = FMath::Max(MaxSpeed, State.LinearVelocityMetersPerSecond.X);
        MaxSideways = FMath::Max(MaxSideways, FMath::Abs(State.LinearVelocityMetersPerSecond.Y));
        MaxDz = FMath::Max(MaxDz, FMath::Abs(State.PositionMeters.Z - 10.0));
        MaxTilt = FMath::Max(MaxTilt, UBot::Multirotor::GetTilt(State.Orientation));
    };
    Drone.Scope.RunFor(2.0, FrameSeconds, Track);
    SpeedAt2s = Drone.Velocity().X;
    Drone.Scope.RunFor(4.0, FrameSeconds, Track);
    const double ErrorAt6s = FMath::Abs(Drone.Velocity().X - 2.0);

    TestTrue(FString::Printf(TEXT("At least 1.8 m/s after 2 s (%.4f)"), SpeedAt2s), SpeedAt2s >= 1.8);
    TestTrue(FString::Printf(TEXT("Peak at most 2.2 m/s (%.4f)"), MaxSpeed), MaxSpeed <= 2.2);
    TestTrue(FString::Printf(TEXT("Within 0.03 m/s after 6 s (%.4f)"), ErrorAt6s), ErrorAt6s <= 0.03);
    TestTrue(FString::Printf(TEXT("Altitude within 0.15 m (%.4f)"), MaxDz), MaxDz <= 0.15);
    TestTrue(FString::Printf(TEXT("No sideways drift (%.4f)"), MaxSideways), MaxSideways <= 0.05);
    TestTrue(FString::Printf(TEXT("Tilts to accelerate (%.2f deg)"), FMath::RadiansToDegrees(MaxTilt)), FMath::RadiansToDegrees(MaxTilt) > 5.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDynamicsYawRateTest, "UBotAerial.Dynamics.YawRate", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDynamicsYawRateTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FPhysics Drone;
    if (!TestTrue(TEXT("Physics drone spawned"), Drone.IsValid()))
    {
        return false;
    }
    SendVelocity(*Drone.Dynamics, FVector::ZeroVector, 0.5);
    double Unwrapped = 0.0;
    double Previous = Drone.Yaw();
    Drone.Scope.RunFor(4.0, FrameSeconds, [&] {
        const double Yaw = Drone.Yaw();
        Unwrapped += UBot::Multirotor::WrapAngle(Yaw - Previous);
        Previous = Yaw;
    });
    TestEqual(TEXT("Yaw after 4 s at 0.5 rad/s"), Unwrapped, 2.0, 0.15);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDynamicsStopTest, "UBotAerial.Dynamics.StopBounded", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDynamicsStopTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FPhysics Drone;
    if (!TestTrue(TEXT("Physics drone spawned"), Drone.IsValid()))
    {
        return false;
    }
    SendVelocity(*Drone.Dynamics, FVector(2.0, 0.0, 0.0));
    Drone.Scope.RunFor(6.0);
    IUBotVelocityCommandable::Execute_StopMotion(Drone.Dynamics);
    const FVector StopStart = Drone.Position();
    double Travel = 0.0;
    Drone.Scope.RunFor(3.0, FrameSeconds, [&] { Travel = FMath::Max(Travel, (Drone.Position() - StopStart).X); });
    TestTrue(FString::Printf(TEXT("Brake travel at most 1.6 m (%.3f)"), Travel), Travel <= 1.6);
    TestTrue(FString::Printf(TEXT("Slower than 0.05 m/s after 3 s (%.4f)"), Drone.Velocity().Size()), Drone.Velocity().Size() < 0.05);
    TestTrue(TEXT("Holding a position"), Drone.Dynamics->GetFlightSetpoint().bHoldPosition);

    FVector Min = Drone.Position();
    FVector Max = Min;
    Drone.Scope.RunFor(7.0, FrameSeconds, [&] {
        Min = Min.ComponentMin(Drone.Position());
        Max = Max.ComponentMax(Drone.Position());
    });
    const double Range = (Max - Min).Size();
    TestTrue(FString::Printf(TEXT("Stays within 5 cm afterwards (%.3f cm)"), Range * 100.0), Range <= 0.05);
    TestEqual(TEXT("Altitude kept"), Drone.Position().Z, 10.0, 0.05);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDynamicsFreeFallTest, "UBotAerial.Dynamics.MotorZeroFreeFall", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDynamicsFreeFallTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FBare Drone(5000.0, [](UUBotMultirotorDynamicsComponent& Dynamics) {
        Dynamics.Airframe.LinearDragCoefficients = FVector::ZeroVector;
        Dynamics.Airframe.QuadraticDragCoefficients = FVector::ZeroVector;
    });
    if (!TestTrue(TEXT("Bare airframe spawned"), Drone.IsValid()))
    {
        return false;
    }
    TestTrue(TEXT("Motor command accepted"), IUBotMotorCommandable::Execute_ApplyMotorCommand(Drone.Dynamics, MotorCommand(0.0)));
    TestEqual(TEXT("Motor mode"), Drone.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::Motor);
    Drone.Scope.RunFor(0.2);
    const double Hover = HoverRotorSpeed(Drone.Scope);
    for (const double Speed : Drone.State().RotorSpeedsRadiansPerSecond)
    {
        TestTrue(FString::Printf(TEXT("Motors stopped after 0.2 s (%.4f)"), Speed), Speed < 1e-3 * Hover);
    }
    Drone.Scope.RunFor(0.8);
    const FUBotMultirotorState At1 = Drone.State();
    Drone.Scope.RunFor(1.0);
    const FUBotMultirotorState At2 = Drone.State();
    const double Elapsed = At2.SimTimeSeconds - At1.SimTimeSeconds;
    const double DeltaV = At2.LinearVelocityMetersPerSecond.Z - At1.LinearVelocityMetersPerSecond.Z;
    TestEqual(TEXT("Falls at g"), DeltaV / Elapsed, Drone.Scope.Gravity().Z, 0.005);
    const double Drop = At2.PositionMeters.Z - At1.PositionMeters.Z;
    const double Expected = At1.LinearVelocityMetersPerSecond.Z * Elapsed + 0.5 * Drone.Scope.Gravity().Z * Elapsed * Elapsed;
    TestEqual(TEXT("Distance fallen"), Drop, Expected, 0.01);
    TestTrue(FString::Printf(TEXT("Accelerometer reads zero in free fall (%s)"), *At2.SpecificForceMetersPerSecondSquared.ToString()),
        At2.SpecificForceMetersPerSecondSquared.Size() < 0.05);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDynamicsCommandLossTest, "UBotAerial.Dynamics.CommandLoss", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDynamicsCommandLossTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    // Hover: an attitude command is released, the drone brakes and holds.
    {
        FPhysics Drone;
        FUBotAttitudeCommand Tilt;
        Tilt.Orientation = FQuat(FVector::ForwardVector, FMath::DegreesToRadians(10.0));
        TestTrue(TEXT("Attitude accepted"), IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Drone.Dynamics, Tilt));
        Drone.Scope.RunFor(1.0);
        IUBotAttitudeCommandable::Execute_ReleaseAttitudeCommand(Drone.Dynamics);
        TestEqual(TEXT("Back to velocity mode"), Drone.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::Velocity);
        Drone.Scope.RunFor(5.0);
        TestTrue(FString::Printf(TEXT("Hovering after release (%.4f)"), Drone.Velocity().Size()), Drone.Velocity().Size() < 0.05);
    }
    // Land: the drone descends in place until the floor stops it, and stays powered there.
    {
        FPhysics Drone(300.0, [](UUBotMultirotorDynamicsComponent& Dynamics) { Dynamics.CommandLossAction = EUBotCommandLossAction::Land; });
        Drone.Scope.SpawnFloor(0.0);
        IUBotMotorCommandable::Execute_ApplyMotorCommand(Drone.Dynamics, MotorCommand(Drone.Dynamics->GetHoverMotorCommand()));
        Drone.Scope.RunFor(0.5);
        const FVector Start = Drone.Position();
        IUBotMotorCommandable::Execute_ReleaseMotorCommand(Drone.Dynamics);
        Drone.Scope.RunFor(8.0);
        TestTrue(FString::Printf(TEXT("On the floor below where it was (%s)"), *Drone.Position().ToString()),
            FVector2D(Drone.Position() - Start).Size() < 0.3 && Drone.Position().Z < 0.1);
        TestTrue(FString::Printf(TEXT("At rest (%.4f)"), Drone.Velocity().Size()), Drone.Velocity().Size() < 0.05);
        const double Hover = HoverRotorSpeed(Drone.Scope);
        TestTrue(FString::Printf(TEXT("Motors still at hover on the floor (%.1f)"), Drone.State().RotorSpeedsRadiansPerSecond[0]),
            FMath::Abs(Drone.State().RotorSpeedsRadiansPerSecond[0] / Hover - 1.0) < 0.05);
    }
    // Cut Motors: the drone falls.
    {
        FPhysics Drone(1000.0, [](UUBotMultirotorDynamicsComponent& Dynamics) { Dynamics.CommandLossAction = EUBotCommandLossAction::CutMotors; });
        FUBotBodyRateCommand Rates;
        TestTrue(TEXT("Body rate accepted"), IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Drone.Dynamics, Rates));
        Drone.Scope.RunFor(0.5);
        IUBotAttitudeCommandable::Execute_ReleaseAttitudeCommand(Drone.Dynamics);
        Drone.Scope.RunFor(1.0);
        TestEqual(TEXT("Motor mode after cutting"), Drone.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::Motor);
        TestTrue(FString::Printf(TEXT("Falling (%.3f)"), Drone.Velocity().Z), Drone.Velocity().Z < -5.0);
    }
    // Timeout: commands older than Command Timeout Seconds count as lost.
    {
        FPhysics Drone(1000.0, [](UUBotMultirotorDynamicsComponent& Dynamics) { Dynamics.CommandTimeoutSeconds = 0.5f; });
        SendVelocity(*Drone.Dynamics, FVector(2.0, 0.0, 0.0));
        Drone.Scope.RunFor(0.4);
        TestTrue(TEXT("Not braking yet"), !Drone.Dynamics->GetFlightSetpoint().bHoldPosition && Drone.Velocity().X > 0.3);
        Drone.Scope.RunFor(3.0);
        TestTrue(FString::Printf(TEXT("Stopped after the timeout (%.4f)"), Drone.Velocity().Size()), Drone.Velocity().Size() < 0.05);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialDynamicsModeAcceptanceTest, "UBotAerial.Dynamics.ModeAcceptance", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialDynamicsModeAcceptanceTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FBare Bare;
    if (!TestTrue(TEXT("Bare airframe spawned"), Bare.IsValid()))
    {
        return false;
    }
    TestFalse(TEXT("Bare airframe rejects velocity"), SendVelocity(*Bare.Dynamics, FVector(1.0, 0.0, 0.0)));
    TestFalse(TEXT("Bare airframe rejects attitude"), IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Bare.Dynamics, FUBotAttitudeCommand()));
    TestTrue(TEXT("Bare airframe takes body rates"), IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Bare.Dynamics, FUBotBodyRateCommand()));
    TestTrue(TEXT("Bare airframe takes motor commands"), IUBotMotorCommandable::Execute_ApplyMotorCommand(Bare.Dynamics, MotorCommand(0.5)));
    TestFalse(TEXT("Wrong motor count rejected"), IUBotMotorCommandable::Execute_ApplyMotorCommand(Bare.Dynamics, MotorCommand(0.5, 3)));
    FUBotMotorCommand NotANumber = MotorCommand(0.5);
    NotANumber.Values[2] = std::numeric_limits<double>::quiet_NaN();
    TestFalse(TEXT("NaN motor command rejected"), IUBotMotorCommandable::Execute_ApplyMotorCommand(Bare.Dynamics, NotANumber));
    TestEqual(TEXT("Motor count"), IUBotMotorCommandable::Execute_GetMotorCount(Bare.Dynamics), 4);

    FUBotMotorCommand Speeds = MotorCommand(550.0);
    Speeds.Type = EUBotMotorCommandType::AngularVelocity;
    TestTrue(TEXT("Rotor speeds accepted"), IUBotMotorCommandable::Execute_ApplyMotorCommand(Bare.Dynamics, Speeds));
    Bare.Scope.RunFor(0.5);
    TestEqual(TEXT("Rotor speed follows the command"), Bare.State().RotorSpeedsRadiansPerSecond[0], 550.0, 0.5);

    FPhysics Physics;
    TestTrue(TEXT("Physics drone takes velocity"), SendVelocity(*Physics.Dynamics, FVector(1.0, 0.0, 0.0)));
    TestTrue(TEXT("Physics drone takes attitude"), IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Physics.Dynamics, FUBotAttitudeCommand()));
    TestEqual(TEXT("Attitude mode"), Physics.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::Attitude);
    FUBotAttitudeCommand ZeroQuat;
    ZeroQuat.Orientation = FQuat(0.0, 0.0, 0.0, 0.0);
    TestFalse(TEXT("Zero quaternion rejected"), IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Physics.Dynamics, ZeroQuat));
    TestTrue(TEXT("Attitude commandable found on the actor"), UBot::Actuation::FindAttitudeCommandable(Physics.Drone) == Physics.Dynamics);
    TestTrue(TEXT("Motor commandable found on the actor"), UBot::Actuation::FindMotorCommandable(Physics.Drone) == Physics.Dynamics);
    TestTrue(TEXT("Velocity commandable found on the actor"), UBot::Actuation::FindVelocityCommandable(Physics.Drone) == Physics.Dynamics);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialControllerAttitudeTest, "UBotAerial.Controller.AttitudeTracking", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialControllerAttitudeTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FPhysics Drone;
    if (!TestTrue(TEXT("Physics drone spawned"), Drone.IsValid()))
    {
        return false;
    }
    // Roll 10 degrees (right side up is +X in the UE sense), altitude held by the vertical loop.
    FUBotAttitudeCommand Roll;
    Roll.Orientation = FQuat(FVector::ForwardVector, FMath::DegreesToRadians(10.0));
    IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Drone.Dynamics, Roll);
    const auto AngleTo = [&](const FQuat& Target) { return FMath::RadiansToDegrees(Drone.State().Orientation.AngularDistance(Target)); };
    double MaxRoll = 0.0;
    double MaxDz = 0.0;
    Drone.Scope.RunFor(0.5, FrameSeconds, [&] {
        const FQuat Q = Drone.State().Orientation;
        MaxRoll = FMath::Max(MaxRoll, FMath::RadiansToDegrees(FMath::Atan2(Q.GetAxisY().Z, Q.GetAxisZ().Z)));
        MaxDz = FMath::Max(MaxDz, FMath::Abs(Drone.Position().Z - 10.0));
    });
    const double ErrorAtHalfSecond = AngleTo(Roll.Orientation);
    Drone.Scope.RunFor(0.5, FrameSeconds, [&] {
        const FQuat Q = Drone.State().Orientation;
        MaxRoll = FMath::Max(MaxRoll, FMath::RadiansToDegrees(FMath::Atan2(Q.GetAxisY().Z, Q.GetAxisZ().Z)));
        MaxDz = FMath::Max(MaxDz, FMath::Abs(Drone.Position().Z - 10.0));
    });
    TestTrue(FString::Printf(TEXT("Within 1 deg after 0.5 s (%.3f)"), ErrorAtHalfSecond), ErrorAtHalfSecond <= 1.0);
    TestTrue(FString::Printf(TEXT("Overshoot at most 3 deg (%.3f)"), MaxRoll - 10.0), MaxRoll <= 13.0);
    TestTrue(FString::Printf(TEXT("Altitude within 0.1 m (%.4f)"), MaxDz), MaxDz <= 0.1);
    TestTrue(TEXT("Rolled right side up and drifts left"), Drone.Velocity().Y < 0.0);

    // Yaw 45 degrees, level.
    FUBotAttitudeCommand Yaw;
    Yaw.Orientation = FQuat(FVector::UpVector, FMath::DegreesToRadians(45.0));
    IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Drone.Dynamics, Yaw);
    Drone.Scope.RunFor(2.0);
    TestTrue(FString::Printf(TEXT("Yaw within 2 deg after 2 s (%.3f)"), AngleTo(Yaw.Orientation)), AngleTo(Yaw.Orientation) <= 2.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialControllerBodyRateTest, "UBotAerial.Controller.BodyRateTracking", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialControllerBodyRateTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FBare Drone;
    if (!TestTrue(TEXT("Bare airframe spawned"), Drone.IsValid()))
    {
        return false;
    }
    FUBotBodyRateCommand Rates;
    Rates.BodyRateRadiansPerSecond = FVector(1.0, 0.0, 0.0);
    IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Drone.Dynamics, Rates);
    TestEqual(TEXT("Body rate mode"), Drone.Dynamics->GetActiveControlMode(), EUBotFlightControlMode::BodyRate);
    // 9 frames is 0.15 s.
    Drone.Scope.RunFor(0.15);
    const double ErrorEarly = FMath::Abs(Drone.State().AngularVelocityRadiansPerSecond.X - 1.0);
    Drone.Scope.RunFor(0.35);
    const double ErrorLate = FMath::Abs(Drone.State().AngularVelocityRadiansPerSecond.X - 1.0);
    TestTrue(FString::Printf(TEXT("Within 10%% after 0.15 s (%.4f)"), ErrorEarly), ErrorEarly <= 0.1);
    TestTrue(FString::Printf(TEXT("Within 2%% after 0.5 s (%.4f)"), ErrorLate), ErrorLate <= 0.02);

    Rates.BodyRateRadiansPerSecond = FVector::ZeroVector;
    IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Drone.Dynamics, Rates);
    Drone.Scope.RunFor(0.2);
    TestTrue(FString::Printf(TEXT("Stops turning within 0.2 s (%s)"), *Drone.State().AngularVelocityRadiansPerSecond.ToString()),
        Drone.State().AngularVelocityRadiansPerSecond.Size() < 0.05);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialControllerCustomTest, "UBotAerial.Controller.CustomController", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialControllerCustomTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FPhysics Drone;
    if (!TestTrue(TEXT("Physics drone spawned"), Drone.IsValid()))
    {
        return false;
    }
    UUBotAerialTestController* Controller = NewObject<UUBotAerialTestController>(Drone.Dynamics);
    Controller->Command = Drone.Dynamics->GetHoverMotorCommand();
    Drone.Dynamics->SetFlightController(Controller);
    const int64 StepsBefore = Drone.Dynamics->GetPhysicsStepCount();
    Drone.Scope.RunFor(1.0);
    const int64 Steps = Drone.Dynamics->GetPhysicsStepCount() - StepsBefore;
    TestEqual(TEXT("Reset once"), Controller->Resets, 1);
    TestEqual(TEXT("Called once per physics step"), static_cast<int64>(Controller->Calls), Steps);
    TestEqual(TEXT("Delta is one step"), Controller->LastDeltaSeconds, 0.001, 1e-12);
    TestEqual(TEXT("Its commands reach the motors"), Drone.Dynamics->GetLastMotorCommands()[0], Controller->Command, 1e-12);
    TestEqual(TEXT("The controller sees the velocity setpoint of the hover"), Controller->LastMode, EUBotFlightControlMode::Velocity);

    // Every fourth step with the divider.
    Drone.Dynamics->ControllerRateDivider = 4;
    Controller->Calls = 0;
    const int64 Before = Drone.Dynamics->GetPhysicsStepCount();
    Drone.Scope.RunFor(1.0);
    const int64 DividedSteps = Drone.Dynamics->GetPhysicsStepCount() - Before;
    TestTrue(FString::Printf(TEXT("Divider 4: %d calls for %lld steps"), Controller->Calls, DividedSteps),
        FMath::Abs(Controller->Calls * 4 - DividedSteps) <= 4);
    TestEqual(TEXT("Delta is four steps"), Controller->LastDeltaSeconds, 0.004, 1e-12);

    // A controller returning the wrong number of commands stops the motors instead of crashing.
    AddExpectedError(TEXT("motor commands for 4 rotors"), EAutomationExpectedErrorFlags::Contains, 1);
    Controller->MotorCount = 3;
    Drone.Scope.RunFor(0.1);
    TestEqual(TEXT("Wrong count stops the motors"), Drone.Dynamics->GetLastMotorCommands()[0], 0.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialBackendTest, "UBotAerial.Backend.Selection", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialBackendTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialDynamicsTests;
    FUBotExtensionRegistry& Registry = FUBotExtensionRegistry::Get();
    TestTrue(TEXT("Extension point declared"), Registry.IsExtensionPointDeclared(UBot::Aerial::DynamicsBackendExtensionPoint));
    TestTrue(TEXT("Built-in backend registered"), UBot::Aerial::FindBackendFactory(UBot::Aerial::BuiltInBackendName).IsValid());

    const TSharedRef<FCountingBackendFactory> Factory = MakeShared<FCountingBackendFactory>();
    const FName TestPackage(TEXT("UBotAerialTests"));
    TestTrue(TEXT("Test backend registered"), Registry.RegisterExtension(UBot::Aerial::DynamicsBackendExtensionPoint, TestPackage, Factory));
    {
        FPhysics Drone(1000.0, [](UUBotMultirotorDynamicsComponent& Dynamics) { Dynamics.DynamicsBackendName = TEXT("UBotAerialTestBackend"); });
        Drone.Scope.RunFor(0.5);
        TestEqual(TEXT("The selected backend runs every step"), static_cast<int64>(Factory->Steps), Drone.Dynamics->GetPhysicsStepCount());
        TestEqual(TEXT("And flies the drone"), Drone.Position().Z, 10.0, 0.01);
    }
    Registry.UnregisterAllFromPackage(TestPackage);

    AddExpectedError(TEXT("no dynamics backend named"), EAutomationExpectedErrorFlags::Contains, 1);
    FPhysics Unknown(1000.0, [](UUBotMultirotorDynamicsComponent& Dynamics) { Dynamics.DynamicsBackendName = TEXT("NoSuchBackend"); });
    Unknown.Scope.RunFor(0.5);
    TestEqual(TEXT("Unknown backend falls back to the built-in one"), Unknown.Position().Z, 10.0, 0.01);
    return true;
}

#endif
