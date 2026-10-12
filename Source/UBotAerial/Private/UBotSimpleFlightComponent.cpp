#include "UBotSimpleFlightComponent.h"

#include "GameFramework/Actor.h"
#include "UBotMultirotorCollision.h"
#include "UBotMultirotorMath.h"

namespace
{
    // Below this speed after touching walkable ground (with no climb commanded) the drone lands, m/s.
    constexpr double LandingSpeed = 0.3;
}

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

void UUBotSimpleFlightComponent::ResetModel(bool bLanded)
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
    if (FlightState == EUBotMultirotorFlightState::Crashed)
    {
        Model.Velocity = FVector::ZeroVector;
        Model.Acceleration = FVector::ZeroVector;
        Model.YawRate = 0.0;
        LinearVelocity = FVector::ZeroVector;
        AngularVelocity = FVector::ZeroVector;
        return;
    }
    if (FlightState == EUBotMultirotorFlightState::Landed)
    {
        if (bBraking || TargetVelocityHeading.Z <= 0.0)
        {
            LinearVelocity = FVector::ZeroVector;
            AngularVelocity = FVector::ZeroVector;
            return;
        }
        SetFlightState(EUBotMultirotorFlightState::Flying);
    }

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
        ApplyContacts(Contacts, Model.Velocity);
        for (const FHitResult& Hit : Contacts.Hits)
        {
            const double IntoSurface = FVector::DotProduct(Model.Acceleration, Hit.Normal);
            if (IntoSurface < 0.0)
            {
                Model.Acceleration -= Hit.Normal * IntoSurface;
            }
        }
        Model.WindDrift = FVector::ZeroVector;
        LinearVelocity = Model.Velocity;

        const FHitResult* Floor = Contacts.FindFloor();
        if (FlightState == EUBotMultirotorFlightState::Flying && Floor && IsWalkable(*Floor)
            && TargetVelocityHeading.Z <= 0.0 && Model.Velocity.Size() < LandingSpeed)
        {
            SetFlightState(EUBotMultirotorFlightState::Landed);
        }
        if (FlightState != EUBotMultirotorFlightState::Flying)
        {
            Model.Velocity = FVector::ZeroVector;
            Model.Acceleration = FVector::ZeroVector;
            Model.YawRate = 0.0;
            LinearVelocity = FVector::ZeroVector;
            TiltedAttitude = UBot::Multirotor::MakeYawQuat(Model.Yaw);
            Orientation = TiltedAttitude;
        }
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
    if (FlightState != EUBotMultirotorFlightState::Flying)
    {
        return 0.0;
    }
    const double G = FMath::Max(GetGravity().Size(), 1e-3);
    return HoverRotorSpeedRadiansPerSecond * FMath::Sqrt((Model.Acceleration - GetGravity()).Size() / G);
}

double UUBotSimpleFlightComponent::GetMaxAccelerationBound() const
{
    const double Tracking = FMath::Max3(MaxHorizontalAccelerationMetersPerSecondSquared, MaxVerticalAccelerationMetersPerSecondSquared,
        FMath::Max(BrakeHorizontalAccelerationMetersPerSecondSquared, BrakeVerticalAccelerationMetersPerSecondSquared));
    // Wind changes move the drone too; gusts stay well below this.
    return Tracking * 1.5 + 10.0;
}
