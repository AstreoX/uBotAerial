#include "UBotRotorComponent.h"

UUBotRotorComponent::UUBotRotorComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    SetCollisionEnabled(ECollisionEnabled::NoCollision);
    SetGenerateOverlapEvents(false);
    SetCanEverAffectNavigation(false);
    CastShadow = true;
}

void UUBotRotorComponent::OnRegister()
{
    Super::OnRegister();
    // The owner may still place the rotor (OnConstruction); the rest pose is taken at the first turn.
    bRestRotationCaptured = false;
    SpinAngle = 0.0;
}

void UUBotRotorComponent::SetRotorSpeed(float RadiansPerSecond)
{
    RotorSpeedRadiansPerSecond = FMath::Max(RadiansPerSecond, 0.0f);
}

void UUBotRotorComponent::AdvanceSpin(double FrameSeconds)
{
    if (RotorSpeedRadiansPerSecond <= 0.0f || FrameSeconds <= 0.0)
    {
        return;
    }
    if (!bRestRotationCaptured)
    {
        RestRotation = GetRelativeRotation().Quaternion();
        bRestRotationCaptured = true;
    }
    // Seen from above, a positive turn about +Z is clockwise (UE yaws right).
    const double Sign = SpinDirection == EUBotRotorSpinDirection::Clockwise ? 1.0 : -1.0;
    SpinAngle = FMath::Fmod(SpinAngle + Sign * RotorSpeedRadiansPerSecond * VisualSpinScale * FrameSeconds, UE_DOUBLE_TWO_PI);
    SetRelativeRotation(RestRotation * FQuat(FVector::UpVector, SpinAngle));
}
