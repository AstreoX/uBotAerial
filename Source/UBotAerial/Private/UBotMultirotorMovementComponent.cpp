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
    // Auto initial state: walkable ground at most this far below counts as standing on it, cm.
    constexpr double GroundProbeCm = 5.0;
    // Gap kept above the ground when snapping down, cm.
    constexpr double GroundSkinCm = 0.1;
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
    LastVelocityCommand = FUBotVelocityCommand();
    bContactThisStep = false;
    bContactLastStep = false;
    QueuedEvents.Reset();
    if (const UWorld* World = GetWorld())
    {
        Gravity = FVector(0.0, 0.0, World->GetGravityZ() / 100.0);
    }

    Collision = MakeShared<FUBotMultirotorCollision>();
    if (!Collision->Initialize(Cast<UPrimitiveComponent>(UpdatedComponent)))
    {
        Collision.Reset();
    }

    bool bLanded = InitialState == EUBotMultirotorInitialState::Landed;
    if (InitialState == EUBotMultirotorInitialState::Auto && Collision)
    {
        FHitResult Floor;
        const double Distance = Collision->FindFloorDistance(PositionMeters * 100.0, Orientation, GroundProbeCm, &Floor);
        if (Distance >= 0.0 && IsWalkable(Floor))
        {
            bLanded = true;
            PositionMeters.Z -= FMath::Max(Distance - GroundSkinCm, 0.0) / 100.0;
        }
    }
    if (bLanded)
    {
        // Resting on the ground means level.
        Orientation = UBot::Multirotor::MakeYawQuat(UBot::Multirotor::GetYaw(Orientation));
    }
    FlightState = bLanded ? EUBotMultirotorFlightState::Landed : EUBotMultirotorFlightState::Flying;

    PhysicsClock.Reset(UBot::Clock::GetSimTimeSeconds(this), PhysicsRateHz);
    bFlightStarted = true;
    ResetModel(bLanded);

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
    if (PhysicsClock.GetDroppedSteps() > DroppedBefore && !bDroppedStepsWarned)
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
        if (Collision && FlightState != EUBotMultirotorFlightState::Landed)
        {
            const double ReachCm = (LinearVelocity.Size() * Span + 0.5 * GetMaxAccelerationBound() * Span * Span) * 100.0 + 2.0;
            bFreeSpaceThisFrame = Collision->IsClear(PositionMeters * 100.0, Orientation, ReachCm);
        }

        for (int32 Step = 0; Step < Steps; ++Step)
        {
            const FVector VelocityBefore = LinearVelocity;
            bContactThisStep = false;
            SimulateStep(StepSeconds);
            PhysicsClock.AdvanceStep();
            ++StepCount;
            LinearAcceleration = (LinearVelocity - VelocityBefore) / StepSeconds;
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
    const bool bExtrapolate = FlightState == EUBotMultirotorFlightState::Flying && !bContactLastStep;
    const FTransform Presented = ComputePresentedTransform(bExtrapolate ? Ahead : 0.0);
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
    if (QueuedEvents.IsEmpty())
    {
        return;
    }
    // Handlers may command the drone again; they see the events of this frame in order.
    TArray<FQueuedEvent> Events = MoveTemp(QueuedEvents);
    QueuedEvents.Reset();
    for (const FQueuedEvent& Event : Events)
    {
        switch (Event.Kind)
        {
        case FQueuedEvent::EKind::StateChanged:
            OnFlightStateChanged.Broadcast(Event.NewState, Event.PreviousState);
            break;
        case FQueuedEvent::EKind::Impact:
            OnImpact.Broadcast(Event.Hit, Event.Speed);
            break;
        case FQueuedEvent::EKind::Crashed:
            OnCrashed.Broadcast(Event.Hit, Event.Speed);
            break;
        }
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

    if (Strongest)
    {
        if (!bContactLastStep && MaxImpact >= ImpactEventMinSpeedMetersPerSecond)
        {
            FQueuedEvent& Event = QueuedEvents.AddDefaulted_GetRef();
            Event.Kind = FQueuedEvent::EKind::Impact;
            Event.Hit = *Strongest;
            Event.Speed = static_cast<float>(MaxImpact);
        }
        if (CrashImpactSpeedMetersPerSecond > 0.0f && MaxImpact >= CrashImpactSpeedMetersPerSecond && FlightState != EUBotMultirotorFlightState::Crashed)
        {
            SetFlightState(EUBotMultirotorFlightState::Crashed);
            FQueuedEvent& Event = QueuedEvents.AddDefaulted_GetRef();
            Event.Kind = FQueuedEvent::EKind::Crashed;
            Event.Hit = *Strongest;
            Event.Speed = static_cast<float>(MaxImpact);
        }
    }
    return MaxImpact;
}

bool UUBotMultirotorMovementComponent::IsWalkable(const FHitResult& Hit) const
{
    return Hit.Normal.Z >= FMath::Cos(FMath::DegreesToRadians(FMath::Clamp(MaxLandingSlopeDegrees, 0.0f, 89.0f)));
}

void UUBotMultirotorMovementComponent::SetFlightState(EUBotMultirotorFlightState NewState)
{
    if (NewState == FlightState)
    {
        return;
    }
    FQueuedEvent& Event = QueuedEvents.AddDefaulted_GetRef();
    Event.Kind = FQueuedEvent::EKind::StateChanged;
    Event.NewState = NewState;
    Event.PreviousState = FlightState;
    FlightState = NewState;
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
    State.FlightState = FlightState;
    State.SimTimeSeconds = PhysicsClock.GetTimeSeconds();
    return State;
}

void UUBotMultirotorMovementComponent::ResetFlight()
{
    if (!UpdatedComponent)
    {
        return;
    }
    const EUBotMultirotorFlightState Previous = FlightState;
    StartFlight();
    if (FlightState != Previous)
    {
        OnFlightStateChanged.Broadcast(FlightState, Previous);
    }
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
        LinearVelocity = FVector::ZeroVector;
        AngularVelocity = FVector::ZeroVector;
        LinearAcceleration = FVector::ZeroVector;
        if (FlightState == EUBotMultirotorFlightState::Landed)
        {
            SetFlightState(EUBotMultirotorFlightState::Flying);
        }
        ResetModel(false);
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
    return HasBegunPlay() && UpdatedComponent != nullptr && FlightState != EUBotMultirotorFlightState::Crashed;
}

bool UUBotMultirotorMovementComponent::ApplyVelocityCommand_Implementation(const FUBotVelocityCommand& Command)
{
    if (Command.LinearVelocityMetersPerSecond.ContainsNaN() || Command.AngularVelocityRadiansPerSecond.ContainsNaN()
        || FlightState == EUBotMultirotorFlightState::Crashed)
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
    return true;
}

void UUBotMultirotorMovementComponent::StopMotion_Implementation()
{
    LastVelocityCommand.LinearVelocityMetersPerSecond = FVector::ZeroVector;
    LastVelocityCommand.AngularVelocityRadiansPerSecond = FVector::ZeroVector;
    HandleStopMotion();
}
