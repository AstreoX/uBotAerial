#include "UBotSimpleFlightComponent.h"

#include "GameFramework/Actor.h"
#include "UBotMultirotorCollision.h"
#include "UBotMultirotorMath.h"

UUBotSimpleFlightComponent::UUBotSimpleFlightComponent()
{
    PhysicsRateHz = 500.0f;
}

void UUBotSimpleFlightComponent::BeginPlay()
{
    Visual.Reset();
    if (AActor* Owner = GetOwner())
    {
        if (USceneComponent* Found = Cast<USceneComponent>(VisualComponent.GetComponent(Owner)))
        {
            Visual = Found;
            VisualRestRotation = Found->GetRelativeRotation().Quaternion();
        }
    }
    Super::BeginPlay();
}

UBot::SimpleFlight::FParams UUBotSimpleFlightComponent::MakeParams() const
{
    UBot::SimpleFlight::FParams Params;
    Params.HorizontalTimeConstantSeconds = HorizontalTimeConstantSeconds;
    Params.VerticalTimeConstantSeconds = VerticalTimeConstantSeconds;
    Params.MaxHorizontalAcceleration = MaxHorizontalAccelerationMetersPerSecondSquared;
    Params.MaxVerticalAcceleration = MaxVerticalAccelerationMetersPerSecondSquared;
    Params.JerkLimit = JerkLimitMetersPerSecondCubed;
    Params.BrakeTimeConstantSeconds = BrakeTimeConstantSeconds;
    Params.BrakeHorizontalAcceleration = BrakeHorizontalAccelerationMetersPerSecondSquared;
    Params.BrakeVerticalAcceleration = BrakeVerticalAccelerationMetersPerSecondSquared;
    Params.YawTimeConstantSeconds = YawTimeConstantSeconds;
    Params.MaxYawAcceleration = MaxYawAccelerationRadiansPerSecondSquared;
    Params.WindResponse = WindResponse;
    Params.WindRejectionTimeConstantSeconds = WindRejectionTimeConstantSeconds;
    return Params;
}

FQuat UUBotSimpleFlightComponent::MakeBodyOrientation() const
{
    return TiltMode == EUBotSimpleFlightTiltMode::Body ? TiltedAttitude : UBot::Multirotor::MakeYawQuat(Model.Yaw);
}

void UUBotSimpleFlightComponent::ResetModel()
{
    Model = UBot::SimpleFlight::FState();
    Model.Yaw = UBot::Multirotor::GetYaw(Orientation);
    TargetVelocityHeading = FVector::ZeroVector;
    TargetYawRate = 0.0;
    bBraking = false;
    TiltedAttitude = UBot::Multirotor::MakeYawQuat(Model.Yaw);
    Orientation = MakeBodyOrientation();
    LinearVelocity = FVector::ZeroVector;
    AngularVelocity = FVector::ZeroVector;
}

void UUBotSimpleFlightComponent::HandleExternalTeleport()
{
    Model.Yaw = UBot::Multirotor::GetYaw(Orientation);
    TiltedAttitude = UBot::SimpleFlight::ComputeAttitude(Model.Acceleration, GetGravity(), Model.Yaw, FMath::DegreesToRadians(MaxTiltDegrees));
    Orientation = MakeBodyOrientation();
}

bool UUBotSimpleFlightComponent::AcceptVelocityCommand(const FVector& LinearHeading, double YawRate)
{
    TargetVelocityHeading = LinearHeading;
    TargetYawRate = YawRate;
    bBraking = false;
    return true;
}

void UUBotSimpleFlightComponent::HandleStopMotion()
{
    TargetVelocityHeading = FVector::ZeroVector;
    TargetYawRate = 0.0;
    bBraking = true;
}

void UUBotSimpleFlightComponent::SimulateStep(double StepSeconds)
{
    UBot::SimpleFlight::FInput Input;
    Input.TargetVelocity = UBot::Multirotor::MakeYawQuat(Model.Yaw).RotateVector(TargetVelocityHeading);
    Input.TargetYawRate = TargetYawRate;
    Input.bBraking = bBraking;
    Input.Wind = bEnableWind ? SampleWind() : FVector::ZeroVector;

    const FQuat Before = Orientation;
    const FVector Delta = UBot::SimpleFlight::Step(MakeParams(), Input, StepSeconds, Model);
    TiltedAttitude = UBot::SimpleFlight::ComputeAttitude(Model.Acceleration, GetGravity(), Model.Yaw, FMath::DegreesToRadians(MaxTiltDegrees));

    FUBotMultirotorContacts Contacts;
    MoveBody(Delta, MakeBodyOrientation(), Contacts);
    LinearVelocity = Model.Velocity + Model.WindDrift;

    if (Contacts.HasContact())
    {
        // The surface stops the motion into it; the drone slides along it and keeps following commands.
        // Wind contributes to the actual approach speed, including OnImpact and friction.
        FVector SlidingVelocity = LinearVelocity;
        ApplyContacts(Contacts, LinearVelocity);
        for (const FHitResult& Hit : Contacts.Hits)
        {
            const double IntoVelocity = FVector::DotProduct(Model.Velocity, Hit.Normal);
            if (IntoVelocity < 0.0)
            {
                Model.Velocity -= Hit.Normal * IntoVelocity;
            }
            const double IntoSliding = FVector::DotProduct(SlidingVelocity, Hit.Normal);
            if (IntoSliding < 0.0)
            {
                SlidingVelocity -= Hit.Normal * IntoSliding;
            }
            const double IntoSurface = FVector::DotProduct(Model.Acceleration, Hit.Normal);
            if (IntoSurface < 0.0)
            {
                Model.Acceleration -= Hit.Normal * IntoSurface;
            }
        }
        const double SlidingSpeed = SlidingVelocity.Size();
        if (SlidingSpeed > UE_DOUBLE_SMALL_NUMBER)
        {
            Model.Velocity *= LinearVelocity.Size() / SlidingSpeed;
        }
        // Keep tracking and wind separate: the next model step recomputes the wind drift.
        Model.WindDrift = LinearVelocity - Model.Velocity;
    }
    AngularVelocity = UBot::Multirotor::BodyRateBetween(Before, Orientation, StepSeconds);
}

void UUBotSimpleFlightComponent::UpdateVisuals(double FrameSeconds)
{
    Super::UpdateVisuals(FrameSeconds);
    USceneComponent* VisualRoot = Visual.Get();
    if (!VisualRoot)
    {
        return;
    }
    if (TiltMode == EUBotSimpleFlightTiltMode::Visual)
    {
        // The pure tilt in the heading frame; the root carries the heading.
        const FQuat Tilt = UBot::Multirotor::MakeYawQuat(Model.Yaw).Inverse() * TiltedAttitude;
        VisualRoot->SetRelativeRotation(Tilt * VisualRestRotation);
    }
    else
    {
        VisualRoot->SetRelativeRotation(VisualRestRotation);
    }
}

double UUBotSimpleFlightComponent::GetVisualRotorSpeed(int32 RotorIndex) const
{
    const double G = FMath::Max(GetGravity().Size(), 1e-3);
    return HoverRotorSpeedRadiansPerSecond * FMath::Sqrt((Model.Acceleration - GetGravity()).Size() / G);
}

double UUBotSimpleFlightComponent::GetMaxAccelerationBound() const
{
    const double Tracking = FMath::Max3(MaxHorizontalAccelerationMetersPerSecondSquared, MaxVerticalAccelerationMetersPerSecondSquared,
        FMath::Max(BrakeHorizontalAccelerationMetersPerSecondSquared, BrakeVerticalAccelerationMetersPerSecondSquared));
    // Heuristic probe size only: actual travel is checked before a sweep can be skipped.
    return Tracking * 1.5 + 10.0;
}
