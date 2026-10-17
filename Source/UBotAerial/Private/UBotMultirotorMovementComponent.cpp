#include "UBotMultirotorMovementComponent.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "UBotAerial.h"
#include "UBotClock.h"
#include "UBotIdentityComponent.h"
#include "UBotMultirotorCollision.h"
#include "UBotRotorComponent.h"
#include "UBotWindSubsystem.h"

namespace
{
    // A presented pose that moved more than this was moved from outside.
    constexpr double TeleportToleranceCm = 1e-3;
    constexpr double TeleportToleranceQuat = 1e-6;
}

UUBotMultirotorMovementComponent::UUBotMultirotorMovementComponent()
{
    // Before physics, so sensors ticking after physics see this frame's pose.
    PrimaryComponentTick.TickGroup = TG_PrePhysics;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
    bUpdateOnlyIfRendered = false;
    bAutoUpdateTickRegistration = true;
}

UUBotMultirotorMovementComponent::~UUBotMultirotorMovementComponent() = default;

void UUBotMultirotorMovementComponent::BeginPlay()
{
    Super::BeginPlay();

    Rotors.Reset();
    if (const AActor* Owner = GetOwner())
    {
        TArray<FVector> Positions;
        TArray<EUBotRotorSpinDirection> Spins;
        GetRotorLayout(Positions, Spins);
        TArray<UUBotRotorComponent*> Found;
        Owner->GetComponents(Found);
        for (UUBotRotorComponent* Rotor : Found)
        {
            // The airframe decides the spin, so a rotor cannot spin against its motor.
            if (Spins.IsValidIndex(Rotor->RotorIndex))
            {
                Rotor->SpinDirection = Spins[Rotor->RotorIndex];
            }
            Rotors.Add(Rotor);
        }
    }

    if (UpdatedComponent)
    {
        StartFlight();
    }
}

void UUBotMultirotorMovementComponent::StartFlight()
{
    const FTransform Transform = UpdatedComponent->GetComponentTransform();
    PositionMeters = Transform.GetLocation() / 100.0;
    Orientation = Transform.GetRotation().GetNormalized();
    LinearVelocity = FVector::ZeroVector;
    AngularVelocity = FVector::ZeroVector;
    LinearAcceleration = FVector::ZeroVector;
    AngularAcceleration = FVector::ZeroVector;
    LastVelocityCommand = FUBotVelocityCommand();
    bContactThisStep = false;
    bContactLastStep = false;
    QueuedImpacts.Reset();
    if (const UWorld* World = GetWorld())
    {
        Gravity = FVector(0.0, 0.0, World->GetGravityZ() / 100.0);
    }

    Collision = MakeShared<FUBotMultirotorCollision>();
    if (!Collision->Initialize(Cast<UPrimitiveComponent>(UpdatedComponent)))
    {
        Collision.Reset();
    }

    PhysicsClock.Reset(UBot::Clock::GetSimTimeSeconds(this), PhysicsRateHz);
    bFlightStarted = true;
    bFirstFrame = true;
    ResetModel();
    // A command accepted before the flight started applies on top of the fresh model.
    TFunction<void()> Pending = MoveTemp(PendingStartCommand);
    PendingStartCommand = nullptr;
    if (Pending)
    {
        Pending();
    }

    UpdatedComponent->SetWorldLocationAndRotation(PositionMeters * 100.0, Orientation, false, nullptr, ETeleportType::TeleportPhysics);
    LastPresentedTransform = UpdatedComponent->GetComponentTransform();
    Velocity = FVector::ZeroVector;
    UpdateComponentVelocity();
}

void UUBotMultirotorMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (!UpdatedComponent || ShouldSkipUpdate(DeltaTime))
    {
        return;
    }
    const UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    Gravity = FVector(0.0, 0.0, World->GetGravityZ() / 100.0);
    WindSubsystem = World->GetSubsystem<UUBotWindSubsystem>();
    const double Now = UBot::Clock::GetSimTimeSeconds(this);
    if (!bFlightStarted)
    {
        StartFlight();
    }
    else
    {
        SyncExternalTeleport();
    }

    PhysicsClock.SetRate(PhysicsRateHz);
    const int64 DroppedBefore = PhysicsClock.GetDroppedSteps();
    const int32 Steps = PhysicsClock.GetStepsDue(Now, MaxStepsPerFrame);
    const bool bFrameAfterStart = bFirstFrame;
    bFirstFrame = false;
    if (PhysicsClock.GetDroppedSteps() > DroppedBefore && !bDroppedStepsWarned && !bFrameAfterStart)
    {
        bDroppedStepsWarned = true;
        UE_LOG(LogUBotAerial, Warning, TEXT("%s: a frame needed more than %d physics steps; the extra time was skipped. Raise Max Steps Per Frame or lower Physics Rate Hz."),
            *GetPathNameSafe(this), MaxStepsPerFrame);
    }

    if (Steps > 0)
    {
        const double StepSeconds = PhysicsClock.GetStepSeconds();
        const double Span = Steps * StepSeconds;
        // One overlap test per frame: when nothing is within reach, the per-step sweeps would find nothing either.
        bFreeSpaceThisFrame = true;
        if (Collision)
        {
            const double ReachCm = (LinearVelocity.Size() * Span + 0.5 * GetMaxAccelerationBound() * Span * Span) * 100.0 + 2.0;
            bFreeSpaceThisFrame = Collision->IsClear(PositionMeters * 100.0, Orientation, ReachCm);
        }

        for (int32 Step = 0; Step < Steps; ++Step)
        {
            const FVector VelocityBefore = LinearVelocity;
            const FVector WorldAngularVelocityBefore = Orientation.RotateVector(AngularVelocity);
            bContactThisStep = false;
            SimulateStep(StepSeconds);
            PhysicsClock.AdvanceStep();
            ++StepCount;
            LinearAcceleration = (LinearVelocity - VelocityBefore) / StepSeconds;
            AngularAcceleration = (Orientation.RotateVector(AngularVelocity) - WorldAngularVelocityBefore) / StepSeconds;
            bContactLastStep = bContactThisStep;
        }
    }

    PresentPose(Now, DeltaTime);
    UpdateVisuals(DeltaTime);
    BroadcastQueuedEvents();
}

void UUBotMultirotorMovementComponent::SyncExternalTeleport()
{
    const FTransform Current = UpdatedComponent->GetComponentTransform();
    if (Current.GetLocation().Equals(LastPresentedTransform.GetLocation(), TeleportToleranceCm)
        && Current.GetRotation().Equals(LastPresentedTransform.GetRotation(), TeleportToleranceQuat))
    {
        return;
    }
    PositionMeters = Current.GetLocation() / 100.0;
    Orientation = Current.GetRotation().GetNormalized();
    LastPresentedTransform = Current;
    HandleExternalTeleport();
}

FTransform UUBotMultirotorMovementComponent::ComputePresentedTransform(double ExtrapolationSeconds) const
{
    const FVector Location = (PositionMeters + LinearVelocity * ExtrapolationSeconds) * 100.0;
    const FQuat Rotation = UBot::Multirotor::IntegrateBodyRate(Orientation, AngularVelocity, ExtrapolationSeconds);
    return FTransform(Rotation, Location, UpdatedComponent->GetComponentScale());
}

void UUBotMultirotorMovementComponent::PresentPose(double NowSeconds, double FrameSeconds)
{
    // Physics runs up to one step behind sim time; show the pose at sim time so sensors that
    // differentiate poses frame by frame see smooth motion.
    const double Ahead = FMath::Clamp(NowSeconds - PhysicsClock.GetTimeSeconds(), 0.0, PhysicsClock.GetStepSeconds());
    // Not into a surface it is touching.
    const FTransform Presented = ComputePresentedTransform(bContactLastStep ? 0.0 : Ahead);
    UpdatedComponent->SetWorldLocationAndRotation(Presented.GetLocation(), Presented.GetRotation(), false, nullptr, ETeleportType::None);
    LastPresentedTransform = UpdatedComponent->GetComponentTransform();
    Velocity = LinearVelocity * 100.0;
    UpdateComponentVelocity();
}

void UUBotMultirotorMovementComponent::UpdateVisuals(double FrameSeconds)
{
    for (const TWeakObjectPtr<UUBotRotorComponent>& RotorPtr : Rotors)
    {
        if (UUBotRotorComponent* Rotor = RotorPtr.Get())
        {
            Rotor->SetRotorSpeed(static_cast<float>(GetVisualRotorSpeed(Rotor->RotorIndex)));
            Rotor->AdvanceSpin(FrameSeconds);
        }
    }
}

void UUBotMultirotorMovementComponent::BroadcastQueuedEvents()
{
    if (QueuedImpacts.IsEmpty())
    {
        return;
    }
    // Handlers may command the drone again; they see the impacts of this frame in order.
    TArray<FQueuedImpact> Impacts = MoveTemp(QueuedImpacts);
    QueuedImpacts.Reset();
    for (const FQueuedImpact& Impact : Impacts)
    {
        OnImpact.Broadcast(Impact.Hit, Impact.Speed);
    }
}

void UUBotMultirotorMovementComponent::MoveBody(const FVector& DeltaMeters, const FQuat& NewOrientation, FUBotMultirotorContacts& OutContacts)
{
    Orientation = NewOrientation;
    if (!Collision || bFreeSpaceThisFrame)
    {
        PositionMeters += DeltaMeters;
        return;
    }
    const FVector EndCm = Collision->Move(PositionMeters * 100.0, DeltaMeters * 100.0, Orientation, OutContacts);
    PositionMeters = EndCm / 100.0;
}

double UUBotMultirotorMovementComponent::ApplyContacts(const FUBotMultirotorContacts& Contacts, FVector& InOutVelocity)
{
    if (!Contacts.HasContact())
    {
        return 0.0;
    }
    bContactThisStep = true;

    double MaxImpact = 0.0;
    const FHitResult* Strongest = nullptr;
    for (const FHitResult& Hit : Contacts.Hits)
    {
        const double IntoSurface = FVector::DotProduct(InOutVelocity, Hit.Normal);
        if (IntoSurface < 0.0)
        {
            InOutVelocity -= Hit.Normal * IntoSurface;
            if (-IntoSurface > MaxImpact)
            {
                MaxImpact = -IntoSurface;
                Strongest = &Hit;
            }
        }
    }

    const double Sliding = InOutVelocity.Size();
    if (MaxImpact > 0.0 && ContactFriction > 0.0 && Sliding > UE_DOUBLE_SMALL_NUMBER)
    {
        InOutVelocity *= FMath::Max(0.0, 1.0 - ContactFriction * MaxImpact / Sliding);
    }

    if (Strongest && !bContactLastStep && MaxImpact >= ImpactEventMinSpeedMetersPerSecond)
    {
        FQueuedImpact& Impact = QueuedImpacts.AddDefaulted_GetRef();
        Impact.Hit = *Strongest;
        Impact.Speed = static_cast<float>(MaxImpact);
    }
    return MaxImpact;
}

void UUBotMultirotorMovementComponent::GetRotorLayout(TArray<FVector>& PositionsMeters, TArray<EUBotRotorSpinDirection>& SpinDirections) const
{
    UBot::Multirotor::GetQuadXRotorLayout(0.225, PositionsMeters, SpinDirections);
}

FVector UUBotMultirotorMovementComponent::SampleWind() const
{
    const UUBotWindSubsystem* Wind = WindSubsystem.Get();
    return Wind ? Wind->GetWindVelocity(PositionMeters * 100.0, GetPhysicsTimeSeconds()) : FVector::ZeroVector;
}

FUBotMultirotorState UUBotMultirotorMovementComponent::GetMultirotorState() const
{
    FUBotMultirotorState State;
    State.PositionMeters = PositionMeters;
    State.Orientation = Orientation;
    State.LinearVelocityMetersPerSecond = LinearVelocity;
    State.AngularVelocityRadiansPerSecond = AngularVelocity;
    State.LinearAccelerationMetersPerSecondSquared = LinearAcceleration;
    State.SpecificForceMetersPerSecondSquared = UBot::Multirotor::SpecificForce(Orientation, LinearAcceleration, Gravity);
    GetRotorSpeeds(State.RotorSpeedsRadiansPerSecond);
    State.SimTimeSeconds = PhysicsClock.GetTimeSeconds();
    return State;
}

bool UUBotMultirotorMovementComponent::ProvidesBodyState(const USceneComponent* Body) const
{
    // The claim holds from registration on (the updated component is set then), before the flight starts.
    return Body != nullptr && Body == UpdatedComponent.Get();
}

bool UUBotMultirotorMovementComponent::GetBodyState(const USceneComponent* Body, FUBotBodyState& OutState) const
{
    if (!ProvidesBodyState(Body) || !bFlightStarted)
    {
        return false;
    }
    // The integrator's state at the latest physics step, not the pose presented for the current sim time.
    FUBotBodyState State;
    State.PositionMeters = PositionMeters;
    State.Orientation = Orientation;
    State.LinearVelocityMetersPerSecond = LinearVelocity;
    // AngularVelocity is in the body frame; the state is in the world frame.
    State.AngularVelocityRadiansPerSecond = Orientation.RotateVector(AngularVelocity);
    State.LinearAccelerationMetersPerSecondSquared = LinearAcceleration;
    State.AngularAccelerationRadiansPerSecondSquared = AngularAcceleration;
    State.bHasLinearAcceleration = true;
    State.bHasAngularAcceleration = true;
    State.TimestampSeconds = PhysicsClock.GetTimeSeconds();
    State.Source = UBot::Aerial::PackageName;
    OutState = State;
    return true;
}

void UUBotMultirotorMovementComponent::ResetFlight()
{
    if (!UpdatedComponent)
    {
        return;
    }
    PendingStartCommand = nullptr;
    StartFlight();
}

void UUBotMultirotorMovementComponent::RunAfterFlightStart(TFunction<void()> Command)
{
    PendingStartCommand = MoveTemp(Command);
}

void UUBotMultirotorMovementComponent::TeleportMultirotor(FVector NewLocation, FRotator NewRotation, bool bStopMotion)
{
    if (!UpdatedComponent)
    {
        return;
    }
    UpdatedComponent->SetWorldLocationAndRotation(NewLocation, NewRotation.Quaternion(), false, nullptr, ETeleportType::TeleportPhysics);
    LastPresentedTransform = UpdatedComponent->GetComponentTransform();
    PositionMeters = LastPresentedTransform.GetLocation() / 100.0;
    Orientation = LastPresentedTransform.GetRotation().GetNormalized();
    if (bStopMotion)
    {
        PendingStartCommand = nullptr;
        LinearVelocity = FVector::ZeroVector;
        AngularVelocity = FVector::ZeroVector;
        LinearAcceleration = FVector::ZeroVector;
        AngularAcceleration = FVector::ZeroVector;
        ResetModel();
    }
    else
    {
        HandleExternalTeleport();
    }
}

FString UUBotMultirotorMovementComponent::GetActuatorId_Implementation() const
{
    return ActuatorId.IsEmpty() ? UBot::Identity::ResolveRobotId(GetOwner()) : ActuatorId;
}

FName UUBotMultirotorMovementComponent::GetActuatorType_Implementation() const
{
    return FName(TEXT("Multirotor"));
}

bool UUBotMultirotorMovementComponent::IsActuatorReady_Implementation() const
{
    return HasBegunPlay() && UpdatedComponent != nullptr;
}

bool UUBotMultirotorMovementComponent::ApplyVelocityCommand_Implementation(const FUBotVelocityCommand& Command)
{
    if (Command.LinearVelocityMetersPerSecond.ContainsNaN() || Command.AngularVelocityRadiansPerSecond.ContainsNaN())
    {
        return false;
    }
    FVector Linear = Command.LinearVelocityMetersPerSecond;
    double YawRate = Command.AngularVelocityRadiansPerSecond.Z;
    UBot::Multirotor::ClampVelocityCommand(VelocityLimits, Linear, YawRate);
    if (!AcceptVelocityCommand(Linear, YawRate))
    {
        return false;
    }
    LastVelocityCommand = Command;
    LastVelocityCommand.LinearVelocityMetersPerSecond = Linear;
    LastVelocityCommand.AngularVelocityRadiansPerSecond = FVector(0.0, 0.0, YawRate);
    if (!bFlightStarted)
    {
        RunAfterFlightStart([this, Command] { ApplyVelocityCommand_Implementation(Command); });
    }
    return true;
}

void UUBotMultirotorMovementComponent::StopMotion_Implementation()
{
    if (!bFlightStarted)
    {
        DropPendingStartCommand();
    }
    LastVelocityCommand.LinearVelocityMetersPerSecond = FVector::ZeroVector;
    LastVelocityCommand.AngularVelocityRadiansPerSecond = FVector::ZeroVector;
    HandleStopMotion();
}
