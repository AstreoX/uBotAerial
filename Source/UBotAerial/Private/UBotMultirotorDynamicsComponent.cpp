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
    // A touched surface keeps blocking commands into it this long after the last contact, so resting on
    // it (touching in some steps, not in others) does not make the command jump, s.
    constexpr double ContactMemorySeconds = 0.05;

    bool IsFinite(const FQuat& Q)
    {
        return FMath::IsFinite(Q.X) && FMath::IsFinite(Q.Y) && FMath::IsFinite(Q.Z) && FMath::IsFinite(Q.W);
    }
}

UUBotMultirotorDynamicsComponent::UUBotMultirotorDynamicsComponent()
{
    PhysicsRateHz = 1000.0f;
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

void UUBotMultirotorDynamicsComponent::ResetModel()
{
    const int32 RotorCount = Airframe.Rotors.Num();
    const double HoverCommand = GetHoverMotorCommand();

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

    // Until a command arrives the drone hovers where it is.
    Setpoint = FUBotFlightSetpoint();
    Setpoint.Mode = EUBotFlightControlMode::Velocity;
    Setpoint.bHoldPosition = true;
    Setpoint.HoldPositionMeters = PositionMeters;
    VelocityCommandHeading = FVector::ZeroVector;
    CommandedVerticalVelocity = 0.0;
    DirectMotorCommands.Init(0.0, RotorCount);
    ContactNormals.Reset();
    bBraking = false;
    bCommandActive = false;
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
    if (!bAcceptAttitudeCommands || !IsFinite(Command.Orientation)
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
    CommandedVerticalVelocity = FMath::Clamp(Command.VerticalVelocityMetersPerSecond,
        -static_cast<double>(VelocityLimits.MaxDescentSpeedMetersPerSecond), static_cast<double>(VelocityLimits.MaxClimbSpeedMetersPerSecond));
    NoteCommand();
    if (!HasFlightStarted())
    {
        RunAfterFlightStart([this, Command] { ApplyAttitudeCommand_Implementation(Command); });
    }
    return true;
}

bool UUBotMultirotorDynamicsComponent::ApplyBodyRateCommand_Implementation(const FUBotBodyRateCommand& Command)
{
    if (!bAcceptBodyRateCommands || Command.BodyRateRadiansPerSecond.ContainsNaN()
        || !FMath::IsFinite(Command.NormalizedThrust) || !FMath::IsFinite(Command.VerticalVelocityMetersPerSecond))
    {
        return false;
    }
    EnterSetpointMode(EUBotFlightControlMode::BodyRate);
    Setpoint.BodyRateRadiansPerSecond = Command.BodyRateRadiansPerSecond;
    Setpoint.bUseThrust = Command.bUseThrust;
    Setpoint.NormalizedThrust = FMath::Clamp(Command.NormalizedThrust, 0.0, 1.0);
    CommandedVerticalVelocity = FMath::Clamp(Command.VerticalVelocityMetersPerSecond,
        -static_cast<double>(VelocityLimits.MaxDescentSpeedMetersPerSecond), static_cast<double>(VelocityLimits.MaxClimbSpeedMetersPerSecond));
    NoteCommand();
    if (!HasFlightStarted())
    {
        RunAfterFlightStart([this, Command] { ApplyBodyRateCommand_Implementation(Command); });
    }
    return true;
}

void UUBotMultirotorDynamicsComponent::ReleaseAttitudeCommand_Implementation()
{
    if (Setpoint.Mode == EUBotFlightControlMode::Attitude || Setpoint.Mode == EUBotFlightControlMode::BodyRate)
    {
        ApplyCommandLoss();
        if (!HasFlightStarted())
        {
            DropPendingStartCommand();
        }
    }
}

bool UUBotMultirotorDynamicsComponent::ApplyMotorCommand_Implementation(const FUBotMotorCommand& Command)
{
    const int32 RotorCount = Airframe.Rotors.Num();
    if (!bAcceptMotorCommands || Command.Values.Num() != RotorCount)
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
    if (!HasFlightStarted())
    {
        RunAfterFlightStart([this, Command] { ApplyMotorCommand_Implementation(Command); });
    }
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
        if (!HasFlightStarted())
        {
            DropPendingStartCommand();
        }
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

FVector UUBotMultirotorDynamicsComponent::ProjectOutOfContacts(const FVector& VelocityMetersPerSecond) const
{
    FVector Result = VelocityMetersPerSecond;
    for (const FVector& Normal : ContactNormals)
    {
        const double IntoSurface = FVector::DotProduct(Result, Normal);
        if (IntoSurface < 0.0)
        {
            Result -= Normal * IntoSurface;
        }
    }
    return Result;
}

void UUBotMultirotorDynamicsComponent::ComputeCommands(double StepSeconds)
{
    const int32 RotorCount = Airframe.Rotors.Num();
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
    // A surface the drone touches stops the motion into it: the commands do not push into it (so the
    // controller does not wind up), and the drone keeps following the rest of the command along it.
    if (Setpoint.Mode == EUBotFlightControlMode::Velocity)
    {
        const FVector World = UBot::Multirotor::MakeYawQuat(UBot::Multirotor::GetYaw(Body.Orientation)).RotateVector(VelocityCommandHeading);
        Setpoint.VelocityMetersPerSecond = ProjectOutOfContacts(World);
    }
    else
    {
        Setpoint.VerticalVelocityMetersPerSecond = ProjectOutOfContacts(FVector(0.0, 0.0, CommandedVerticalVelocity)).Z;
    }

    FUBotMultirotorEnvironment Environment;
    Environment.Gravity = GetGravity();
    Environment.Wind = bEnableWind ? SampleWind() : FVector::ZeroVector;

    ComputeCommands(StepSeconds);
    Backend->Step(MotorCommands, Environment, StepSeconds, Body);

    FUBotMultirotorContacts Contacts;
    MoveBody(Body.Position - PositionMeters, Body.Orientation.GetNormalized(), Contacts);
    Body.Position = PositionMeters;
    Body.Orientation = Orientation;

    if (Contacts.HasContact())
    {
        ApplyContacts(Contacts, Body.Velocity);
        ContactNormals.Reset();
        for (const FHitResult& Hit : Contacts.Hits)
        {
            ContactNormals.Add(Hit.Normal);
        }
        LastContactTimeSeconds = GetPhysicsTimeSeconds();
    }
    else if (GetPhysicsTimeSeconds() - LastContactTimeSeconds > ContactMemorySeconds)
    {
        ContactNormals.Reset();
    }

    LinearVelocity = Body.Velocity;
    AngularVelocity = Body.AngularVelocity;
}
