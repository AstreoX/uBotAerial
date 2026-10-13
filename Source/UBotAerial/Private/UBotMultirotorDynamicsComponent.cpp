#include "UBotMultirotorDynamicsComponent.h"

#include "UBotAerial.h"
#include "UBotCascadedFlightController.h"
#include "UBotClock.h"
#include "UBotMultirotorCollision.h"
#include "UBotMultirotorDynamicsBackend.h"
#include "UBotMultirotorMath.h"

namespace
{
    // Braking: below this horizontal speed the drone holds the position where it is, m/s.
    constexpr double HoldLatchSpeed = 0.05;
    // Touching walkable ground slower than this (without climbing) lands, m/s.
    constexpr double LandingSpeed = 0.3;
    // A crashed drone resting on the ground slower than this stops moving, m/s.
    constexpr double RestSpeed = 0.05;

    bool IsFinite(const FQuat& Q)
    {
        return FMath::IsFinite(Q.X) && FMath::IsFinite(Q.Y) && FMath::IsFinite(Q.Z) && FMath::IsFinite(Q.W);
    }
}

UUBotMultirotorDynamicsComponent::UUBotMultirotorDynamicsComponent()
{
    PhysicsRateHz = 1000.0f;
    CrashImpactSpeedMetersPerSecond = 3.0f;
    FlightController = CreateDefaultSubobject<UUBotCascadedFlightController>(TEXT("FlightController"));
}

void UUBotMultirotorDynamicsComponent::BeginPlay()
{
    FString Error;
    bAirframeValid = UBot::Multirotor::IsAirframeValid(Airframe, &Error);
    if (!bAirframeValid)
    {
        UE_LOG(LogUBotAerial, Error, TEXT("%s: the airframe cannot fly (%s); the default quadrotor is used instead."), *GetPathNameSafe(this), *Error);
        Airframe = FUBotMultirotorAirframe();
        bAirframeValid = true;
    }

    FallbackController = NewObject<UUBotCascadedFlightController>(this, TEXT("FallbackController"), RF_Transient);

    TSharedPtr<IUBotMultirotorBackendFactory> Factory = UBot::Aerial::FindBackendFactory(DynamicsBackendName);
    if (!Factory)
    {
        UE_LOG(LogUBotAerial, Warning, TEXT("%s: no dynamics backend named '%s'; using '%s'."), *GetPathNameSafe(this),
            *DynamicsBackendName.ToString(), *UBot::Aerial::BuiltInBackendName.ToString());
        Factory = UBot::Aerial::FindBackendFactory(UBot::Aerial::BuiltInBackendName);
    }
    Backend = Factory ? Factory->CreateBackend(Airframe) : nullptr;

    Super::BeginPlay();
}

void UUBotMultirotorDynamicsComponent::SetFlightController(UUBotFlightController* NewController)
{
    FlightController = NewController;
    ActiveController.Reset();
    StepsUntilController = 0;
}

double UUBotMultirotorDynamicsComponent::GetHoverMotorCommand() const
{
    return UBot::Multirotor::GetHoverCommand(Airframe, GetGravity().Size());
}

void UUBotMultirotorDynamicsComponent::GetRotorLayout(TArray<FVector>& PositionsMeters, TArray<EUBotRotorSpinDirection>& SpinDirections) const
{
    PositionsMeters.Reset();
    SpinDirections.Reset();
    for (const FUBotRotorSpec& Rotor : Airframe.Rotors)
    {
        PositionsMeters.Add(Rotor.PositionMeters);
        SpinDirections.Add(Rotor.SpinDirection);
    }
}

void UUBotMultirotorDynamicsComponent::GetRotorSpeeds(TArray<double>& OutSpeeds) const
{
    OutSpeeds = Body.RotorSpeeds;
}

double UUBotMultirotorDynamicsComponent::GetVisualRotorSpeed(int32 RotorIndex) const
{
    return Body.RotorSpeeds.IsValidIndex(RotorIndex) ? Body.RotorSpeeds[RotorIndex] : 0.0;
}

double UUBotMultirotorDynamicsComponent::GetMaxAccelerationBound() const
{
    double MaxThrust = 0.0;
    for (const FUBotRotorSpec& Rotor : Airframe.Rotors)
    {
        MaxThrust += UBot::Multirotor::GetMaxRotorThrust(Rotor);
    }
    return MaxThrust / FMath::Max(Airframe.MassKilograms, 1e-3) + GetGravity().Size() + 10.0;
}

void UUBotMultirotorDynamicsComponent::ResetModel(bool bLanded)
{
    const int32 RotorCount = Airframe.Rotors.Num();
    const double HoverCommand = bLanded ? 0.0 : GetHoverMotorCommand();

    Body = UBot::Multirotor::FRigidBodyState();
    Body.Position = PositionMeters;
    Body.Orientation = Orientation;
    Body.RotorSpeeds.SetNumZeroed(RotorCount);
    MotorCommands.Init(HoverCommand, RotorCount);
    for (int32 Index = 0; Index < RotorCount; ++Index)
    {
        Body.RotorSpeeds[Index] = UBot::Multirotor::CommandToRotorSpeed(Airframe.Rotors[Index], Airframe.MotorCommandMapping, HoverCommand);
    }
    if (Backend)
    {
        Backend->Reset(Airframe, Body);
    }

    // Until a command arrives the drone hovers where it is (or stays landed).
    Setpoint = FUBotFlightSetpoint();
    Setpoint.Mode = EUBotFlightControlMode::Velocity;
    Setpoint.bHoldPosition = !bLanded;
    Setpoint.HoldPositionMeters = PositionMeters;
    VelocityCommandHeading = FVector::ZeroVector;
    DirectMotorCommands.Init(0.0, RotorCount);
    bBraking = false;
    bLanding = false;
    bCommandActive = false;
    bCrashedAtRest = false;
    LinearVelocity = FVector::ZeroVector;
    AngularVelocity = FVector::ZeroVector;
    ActiveController.Reset();
    StepsUntilController = 0;
}

void UUBotMultirotorDynamicsComponent::HandleExternalTeleport()
{
    Body.Position = PositionMeters;
    Body.Orientation = Orientation;
    if (Setpoint.bHoldPosition)
    {
        Setpoint.HoldPositionMeters = PositionMeters;
    }
}

UUBotFlightController* UUBotMultirotorDynamicsComponent::ResolveController(EUBotFlightControlMode Mode)
{
    if (FlightController && FlightController->SupportsControlMode(Mode))
    {
        return FlightController;
    }
    return FallbackController;
}

const FUBotMultirotorState& UUBotMultirotorDynamicsComponent::UpdateControllerState()
{
    ControllerState.PositionMeters = Body.Position;
    ControllerState.Orientation = Body.Orientation;
    ControllerState.LinearVelocityMetersPerSecond = Body.Velocity;
    ControllerState.AngularVelocityRadiansPerSecond = Body.AngularVelocity;
    ControllerState.LinearAccelerationMetersPerSecondSquared = LinearAcceleration;
    ControllerState.SpecificForceMetersPerSecondSquared = UBot::Multirotor::SpecificForce(Body.Orientation, LinearAcceleration, GetGravity());
    ControllerState.RotorSpeedsRadiansPerSecond = Body.RotorSpeeds;
    ControllerState.FlightState = FlightState;
    ControllerState.SimTimeSeconds = GetPhysicsTimeSeconds();
    return ControllerState;
}

void UUBotMultirotorDynamicsComponent::EnterSetpointMode(EUBotFlightControlMode Mode)
{
    if (Setpoint.Mode != Mode)
    {
        Setpoint.Mode = Mode;
        // The next controller step starts clean in the new mode.
        ActiveController.Reset();
        StepsUntilController = 0;
    }
    bBraking = false;
    bLanding = false;
}

void UUBotMultirotorDynamicsComponent::NoteCommand()
{
    LastCommandSimTime = UBot::Clock::GetSimTimeSeconds(this);
    bCommandActive = true;
}

bool UUBotMultirotorDynamicsComponent::AcceptVelocityCommand(const FVector& LinearHeading, double YawRate)
{
    if (!bAcceptVelocityCommands)
    {
        return false;
    }
    EnterSetpointMode(EUBotFlightControlMode::Velocity);
    VelocityCommandHeading = LinearHeading;
    Setpoint.YawRateRadiansPerSecond = YawRate;
    Setpoint.bHoldPosition = false;
    NoteCommand();
    return true;
}

void UUBotMultirotorDynamicsComponent::HandleStopMotion()
{
    EnterSetpointMode(EUBotFlightControlMode::Velocity);
    VelocityCommandHeading = FVector::ZeroVector;
    Setpoint.YawRateRadiansPerSecond = 0.0;
    Setpoint.bHoldPosition = false;
    bBraking = true;
    bCommandActive = false;
}

bool UUBotMultirotorDynamicsComponent::ApplyAttitudeCommand_Implementation(const FUBotAttitudeCommand& Command)
{
    if (!bAcceptAttitudeCommands || FlightState == EUBotMultirotorFlightState::Crashed || !IsFinite(Command.Orientation)
        || Command.Orientation.SizeSquared() < 0.01 || !FMath::IsFinite(Command.YawRateRadiansPerSecond)
        || !FMath::IsFinite(Command.NormalizedThrust) || !FMath::IsFinite(Command.VerticalVelocityMetersPerSecond))
    {
        return false;
    }
    EnterSetpointMode(EUBotFlightControlMode::Attitude);
    Setpoint.Orientation = Command.Orientation.GetNormalized();
    Setpoint.bUseYawRate = Command.bUseYawRate;
    Setpoint.YawRateRadiansPerSecond = FMath::Clamp(Command.YawRateRadiansPerSecond, -static_cast<double>(VelocityLimits.MaxYawRateRadiansPerSecond),
        static_cast<double>(VelocityLimits.MaxYawRateRadiansPerSecond));
    Setpoint.bUseThrust = Command.bUseThrust;
    Setpoint.NormalizedThrust = FMath::Clamp(Command.NormalizedThrust, 0.0, 1.0);
    Setpoint.VerticalVelocityMetersPerSecond = FMath::Clamp(Command.VerticalVelocityMetersPerSecond,
        -static_cast<double>(VelocityLimits.MaxDescentSpeedMetersPerSecond), static_cast<double>(VelocityLimits.MaxClimbSpeedMetersPerSecond));
    NoteCommand();
    return true;
}

bool UUBotMultirotorDynamicsComponent::ApplyBodyRateCommand_Implementation(const FUBotBodyRateCommand& Command)
{
    if (!bAcceptBodyRateCommands || FlightState == EUBotMultirotorFlightState::Crashed || Command.BodyRateRadiansPerSecond.ContainsNaN()
        || !FMath::IsFinite(Command.NormalizedThrust) || !FMath::IsFinite(Command.VerticalVelocityMetersPerSecond))
    {
        return false;
    }
    EnterSetpointMode(EUBotFlightControlMode::BodyRate);
    Setpoint.BodyRateRadiansPerSecond = Command.BodyRateRadiansPerSecond;
    Setpoint.bUseThrust = Command.bUseThrust;
    Setpoint.NormalizedThrust = FMath::Clamp(Command.NormalizedThrust, 0.0, 1.0);
    Setpoint.VerticalVelocityMetersPerSecond = FMath::Clamp(Command.VerticalVelocityMetersPerSecond,
        -static_cast<double>(VelocityLimits.MaxDescentSpeedMetersPerSecond), static_cast<double>(VelocityLimits.MaxClimbSpeedMetersPerSecond));
    NoteCommand();
    return true;
}

void UUBotMultirotorDynamicsComponent::ReleaseAttitudeCommand_Implementation()
{
    if (Setpoint.Mode == EUBotFlightControlMode::Attitude || Setpoint.Mode == EUBotFlightControlMode::BodyRate)
    {
        ApplyCommandLoss();
    }
}

bool UUBotMultirotorDynamicsComponent::ApplyMotorCommand_Implementation(const FUBotMotorCommand& Command)
{
    const int32 RotorCount = Airframe.Rotors.Num();
    if (!bAcceptMotorCommands || FlightState == EUBotMultirotorFlightState::Crashed || Command.Values.Num() != RotorCount)
    {
        return false;
    }
    for (const double Value : Command.Values)
    {
        if (!FMath::IsFinite(Value))
        {
            return false;
        }
    }
    DirectMotorCommands.SetNumZeroed(RotorCount);
    for (int32 Index = 0; Index < RotorCount; ++Index)
    {
        DirectMotorCommands[Index] = Command.Type == EUBotMotorCommandType::AngularVelocity
            ? UBot::Multirotor::RotorSpeedToCommand(Airframe.Rotors[Index], Airframe.MotorCommandMapping, Command.Values[Index])
            : FMath::Clamp(Command.Values[Index], 0.0, 1.0);
    }
    EnterSetpointMode(EUBotFlightControlMode::Motor);
    NoteCommand();
    return true;
}

int32 UUBotMultirotorDynamicsComponent::GetMotorCount_Implementation() const
{
    return Airframe.Rotors.Num();
}

void UUBotMultirotorDynamicsComponent::ReleaseMotorCommand_Implementation()
{
    if (Setpoint.Mode == EUBotFlightControlMode::Motor)
    {
        ApplyCommandLoss();
    }
}

void UUBotMultirotorDynamicsComponent::ApplyCommandLoss()
{
    switch (CommandLossAction)
    {
    case EUBotCommandLossAction::Land:
        EnterSetpointMode(EUBotFlightControlMode::Velocity);
        VelocityCommandHeading = FVector(0.0, 0.0, -LandingSpeedMetersPerSecond);
        Setpoint.YawRateRadiansPerSecond = 0.0;
        Setpoint.bHoldPosition = true;
        Setpoint.HoldPositionMeters = PositionMeters;
        bLanding = true;
        break;
    case EUBotCommandLossAction::CutMotors:
        EnterSetpointMode(EUBotFlightControlMode::Motor);
        DirectMotorCommands.Init(0.0, Airframe.Rotors.Num());
        break;
    case EUBotCommandLossAction::Hover:
    default:
        HandleStopMotion();
        break;
    }
    bCommandActive = false;
}

bool UUBotMultirotorDynamicsComponent::WantsTakeOff() const
{
    switch (Setpoint.Mode)
    {
    case EUBotFlightControlMode::Velocity:
        return !bBraking && !bLanding && VelocityCommandHeading.Z > 0.0;
    case EUBotFlightControlMode::Attitude:
    case EUBotFlightControlMode::BodyRate:
        return Setpoint.bUseThrust ? Setpoint.NormalizedThrust > 0.0 : Setpoint.VerticalVelocityMetersPerSecond > 0.0;
    case EUBotFlightControlMode::Motor:
    default:
        for (const double Command : DirectMotorCommands)
        {
            if (Command > 0.0)
            {
                return true;
            }
        }
        return false;
    }
}

void UUBotMultirotorDynamicsComponent::ComputeCommands(double StepSeconds)
{
    const int32 RotorCount = Airframe.Rotors.Num();
    if (FlightState == EUBotMultirotorFlightState::Crashed)
    {
        MotorCommands.Init(0.0, RotorCount);
        return;
    }
    if (Setpoint.Mode == EUBotFlightControlMode::Motor)
    {
        MotorCommands = DirectMotorCommands;
        MotorCommands.SetNumZeroed(RotorCount);
        return;
    }

    UUBotFlightController* Controller = ResolveController(Setpoint.Mode);
    if (!Controller)
    {
        MotorCommands.Init(0.0, RotorCount);
        return;
    }
    if (ActiveController.Get() != Controller)
    {
        Controller->Reset(UpdateControllerState(), Airframe);
        ActiveController = Controller;
        StepsUntilController = 0;
    }
    if (StepsUntilController <= 0)
    {
        const int32 Divider = FMath::Max(ControllerRateDivider, 1);
        Controller->Compute(UpdateControllerState(), Setpoint, StepSeconds * Divider, MotorCommands);
        StepsUntilController = Divider;
        if (MotorCommands.Num() != RotorCount)
        {
            if (!bControllerOutputWarned)
            {
                bControllerOutputWarned = true;
                UE_LOG(LogUBotAerial, Error, TEXT("%s: the flight controller returned %d motor commands for %d rotors; the motors are stopped."),
                    *GetPathNameSafe(this), MotorCommands.Num(), RotorCount);
            }
            MotorCommands.Init(0.0, RotorCount);
        }
        for (double& Command : MotorCommands)
        {
            Command = FMath::IsFinite(Command) ? FMath::Clamp(Command, 0.0, 1.0) : 0.0;
        }
    }
    --StepsUntilController;
}

void UUBotMultirotorDynamicsComponent::SimulateStep(double StepSeconds)
{
    if (!Backend)
    {
        return;
    }

    if (bCommandActive && CommandTimeoutSeconds > 0.0f && GetPhysicsTimeSeconds() - LastCommandSimTime > CommandTimeoutSeconds)
    {
        ApplyCommandLoss();
    }

    // After braking slows the drone down, it holds where it is.
    if (Setpoint.Mode == EUBotFlightControlMode::Velocity && bBraking && !Setpoint.bHoldPosition
        && FVector2D(Body.Velocity.X, Body.Velocity.Y).Size() < HoldLatchSpeed)
    {
        Setpoint.bHoldPosition = true;
        Setpoint.HoldPositionMeters = Body.Position;
    }
    if (Setpoint.Mode == EUBotFlightControlMode::Velocity)
    {
        Setpoint.VelocityMetersPerSecond = UBot::Multirotor::MakeYawQuat(UBot::Multirotor::GetYaw(Body.Orientation)).RotateVector(VelocityCommandHeading);
    }

    FUBotMultirotorEnvironment Environment;
    Environment.Gravity = GetGravity();
    Environment.Wind = bEnableWind ? SampleWind() : FVector::ZeroVector;

    const bool bAtRest = FlightState == EUBotMultirotorFlightState::Crashed && bCrashedAtRest;
    if (FlightState == EUBotMultirotorFlightState::Landed || bAtRest)
    {
        if (bAtRest || !WantsTakeOff())
        {
            // Resting: motors spin down, the body stays where it is.
            MotorCommands.Init(0.0, Airframe.Rotors.Num());
            for (int32 Index = 0; Index < Body.RotorSpeeds.Num(); ++Index)
            {
                const double TimeConstant = Airframe.Rotors[Index].MotorTimeConstantSeconds;
                Body.RotorSpeeds[Index] *= TimeConstant > 0.0 ? FMath::Exp(-StepSeconds / TimeConstant) : 0.0;
            }
            ActiveController.Reset();
            LinearVelocity = FVector::ZeroVector;
            AngularVelocity = FVector::ZeroVector;
            return;
        }
        ComputeCommands(StepSeconds);
        UBot::Multirotor::FRigidBodyState Trial = Body;
        Backend->Step(MotorCommands, Environment, StepSeconds, Trial);
        if (Trial.Velocity.Z <= 0.0)
        {
            // Not enough thrust yet: the ground holds the drone while the motors spin up.
            Body.RotorSpeeds = Trial.RotorSpeeds;
            LinearVelocity = FVector::ZeroVector;
            AngularVelocity = FVector::ZeroVector;
            return;
        }
        SetFlightState(EUBotMultirotorFlightState::Flying);
        Body = MoveTemp(Trial);
    }
    else
    {
        ComputeCommands(StepSeconds);
        Backend->Step(MotorCommands, Environment, StepSeconds, Body);
    }

    FUBotMultirotorContacts Contacts;
    MoveBody(Body.Position - PositionMeters, Body.Orientation.GetNormalized(), Contacts);
    Body.Position = PositionMeters;
    Body.Orientation = Orientation;

    if (Contacts.HasContact())
    {
        const double PushUp = Body.Acceleration.Z;
        ApplyContacts(Contacts, Body.Velocity);
        const FHitResult* Floor = Contacts.FindFloor();
        const bool bOnFloor = Floor && IsWalkable(*Floor);
        if (FlightState == EUBotMultirotorFlightState::Crashed)
        {
            MotorCommands.Init(0.0, Airframe.Rotors.Num());
            Body.AngularVelocity *= 0.5;
            if (bOnFloor && Body.Velocity.Size() < RestSpeed)
            {
                // Lying on the ground: stays there until Reset Flight.
                Body.Velocity = FVector::ZeroVector;
                Body.AngularVelocity = FVector::ZeroVector;
                bCrashedAtRest = true;
            }
        }
        else if (bOnFloor && Body.Velocity.Size() < LandingSpeed
            && (Setpoint.Mode == EUBotFlightControlMode::Velocity ? !WantsTakeOff() : PushUp <= 0.0))
        {
            SetFlightState(EUBotMultirotorFlightState::Landed);
            Body.Velocity = FVector::ZeroVector;
            Body.AngularVelocity = FVector::ZeroVector;
            Orientation = UBot::Multirotor::MakeYawQuat(UBot::Multirotor::GetYaw(Orientation));
            Body.Orientation = Orientation;
            if (bLanding || Setpoint.Mode == EUBotFlightControlMode::Velocity)
            {
                // Landed after a descent: wait for a climb command.
                VelocityCommandHeading = FVector::ZeroVector;
                Setpoint.bHoldPosition = false;
                bLanding = false;
                bBraking = false;
            }
            ActiveController.Reset();
        }
    }

    LinearVelocity = Body.Velocity;
    AngularVelocity = Body.AngularVelocity;
}
