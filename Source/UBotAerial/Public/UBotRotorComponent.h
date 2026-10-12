#pragma once

#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "UBotAerialTypes.h"
#include "UBotRotorComponent.generated.h"

/**
 * Visual rotor of a multirotor: a mesh spinning about its local Z axis. The multirotor movement
 * component sets the rotor speed every frame from its motor with the same Rotor Index.
 */
UCLASS(ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot Rotor"))
class UBOTAERIAL_API UUBotRotorComponent : public UStaticMeshComponent
{
    GENERATED_BODY()

public:
    UUBotRotorComponent();

    /** Index of the motor that drives this rotor (the order of the airframe's rotors). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor", meta = (ClampMin = "0"))
    int32 RotorIndex = 0;

    /** Spin direction seen from above. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor")
    EUBotRotorSpinDirection SpinDirection = EUBotRotorSpinDirection::Clockwise;

    /** Display speed as a share of the real rotor speed: real speeds alias at frame rate, so the blades are shown slower. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rotor", meta = (ClampMin = "0.0"))
    float VisualSpinScale = 0.04f;

    /** Sets the rotor speed shown from now on, rad/s (the magnitude; the direction comes from Spin Direction). */
    UFUNCTION(BlueprintCallable, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Set Rotor Speed"))
    void SetRotorSpeed(float RadiansPerSecond);

    /** The rotor speed last set, rad/s. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Rotor Speed"))
    float GetRotorSpeed() const { return RotorSpeedRadiansPerSecond; }

    /** Turns the mesh by the current speed for FrameSeconds. */
    void AdvanceSpin(double FrameSeconds);

protected:
    virtual void OnRegister() override;

private:
    float RotorSpeedRadiansPerSecond = 0.0f;
    double SpinAngle = 0.0;
    FQuat RestRotation = FQuat::Identity;
    bool bRestRotationCaptured = false;
};
