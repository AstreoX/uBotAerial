#pragma once

#include "CoreMinimal.h"
#include "Engine/HitResult.h"
#include "GameFramework/MovementComponent.h"
#include "UBotActuatorInterface.h"
#include "UBotAerialTypes.h"
#include "UBotMultirotorMath.h"
#include "UBotMultirotorMovementComponent.generated.h"

class FUBotMultirotorCollision;
class UUBotRotorComponent;
class UUBotWindSubsystem;
struct FUBotMultirotorContacts;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FUBotMultirotorFlightStateChangedEvent, EUBotMultirotorFlightState, NewState, EUBotMultirotorFlightState, PreviousState);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FUBotMultirotorImpactEvent, const FHitResult&, Hit, float, ImpactSpeedMetersPerSecond);

/**
 * Base of the multirotor flight models. Moves the root component (the updated component) in fixed
 * physics steps on uBot sim time, sweeps its collision shape through the world, lands and crashes,
 * and presents the pose extrapolated to the current sim time every frame. Accepts velocity commands
 * (IUBotVelocityCommandable) in the heading frame: X forward and Y right along the current heading,
 * Z up; only the Z angular velocity (yaw rate) is used.
 */
UCLASS(Abstract, ClassGroup = (uBot), meta = (DisplayName = "uBot Multirotor Movement"))
class UBOTAERIAL_API UUBotMultirotorMovementComponent : public UMovementComponent, public IUBotActuator, public IUBotVelocityCommandable
{
    GENERATED_BODY()

public:
    UUBotMultirotorMovementComponent();
    virtual ~UUBotMultirotorMovementComponent() override;

    /** Physics steps per second of sim time. More steps are more accurate and cost more. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Multirotor", meta = (ClampMin = "100.0", ClampMax = "4000.0", Units = "Hertz"))
    float PhysicsRateHz = 500.0f;

    /** At most this many physics steps per frame. When a frame is longer, the extra time is skipped and a warning is logged once. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Multirotor", AdvancedDisplay, meta = (ClampMin = "1"))
    int32 MaxStepsPerFrame = 250;

    /** Velocity commands above these limits are clamped. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor|Limits")
    FUBotMultirotorVelocityLimits VelocityLimits;

    /** How the drone starts when play begins (also after Reset Flight). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor")
    EUBotMultirotorInitialState InitialState = EUBotMultirotorInitialState::Auto;

    /** Impact speed along the surface normal at which the drone crashes, m/s. 0 never crashes. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor|Contact", meta = (ClampMin = "0.0", Units = "MetersPerSecond"))
    float CrashImpactSpeedMetersPerSecond = 0.0f;

    /** Impacts slower than this (along the surface normal) do not fire On Impact, m/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor|Contact", meta = (ClampMin = "0.0", Units = "MetersPerSecond"))
    float ImpactEventMinSpeedMetersPerSecond = 0.3f;

    /** Sliding friction on contact: the sliding speed drops by this share of the impact speed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor|Contact", meta = (ClampMin = "0.0"))
    float ContactFriction = 0.3f;

    /** Steepest surface the drone lands on, degrees from level. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor|Contact", meta = (ClampMin = "0.0", ClampMax = "89.0", Units = "Degrees"))
    float MaxLandingSlopeDegrees = 30.0f;

    /** Whether wind from uBot Wind actors affects this drone. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor")
    bool bEnableWind = true;

    /** Actuator id reported to uBot. Empty uses the robot id of the owner (its uBot Identity, else its name). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Multirotor")
    FString ActuatorId;

    /** Fires when the flight state changes (take-off, landing, crash, reset). */
    UPROPERTY(BlueprintAssignable, Category = "uBot|Aerial|Multirotor")
    FUBotMultirotorFlightStateChangedEvent OnFlightStateChanged;

    /** Fires when the drone starts touching something at least Impact Event Min Speed fast. */
    UPROPERTY(BlueprintAssignable, Category = "uBot|Aerial|Multirotor")
    FUBotMultirotorImpactEvent OnImpact;

    /** Fires when an impact reaches Crash Impact Speed. */
    UPROPERTY(BlueprintAssignable, Category = "uBot|Aerial|Multirotor")
    FUBotMultirotorImpactEvent OnCrashed;

    /** The current flight state. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Flight State"))
    EUBotMultirotorFlightState GetFlightState() const { return FlightState; }

    /** True while the drone rests on the ground. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Is Landed"))
    bool IsLanded() const { return FlightState == EUBotMultirotorFlightState::Landed; }

    /** Ground-truth state at the latest physics step (SI units, UE world axes). */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Multirotor State"))
    FUBotMultirotorState GetMultirotorState() const;

    /** The latest accepted velocity command, after clamping. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Last Velocity Command"))
    FUBotVelocityCommand GetLastVelocityCommand() const { return LastVelocityCommand; }

    /** Physics steps run since play began. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Physics Step Count"))
    int64 GetPhysicsStepCount() const { return StepCount; }

    /** Clears a crash, stops all motion and starts again as Initial State says, at the current pose. */
    UFUNCTION(BlueprintCallable, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Reset Flight"))
    void ResetFlight();

    /** Moves the drone to a new pose without sweeping. With Stop Motion, velocities are zeroed and the drone hovers there. */
    UFUNCTION(BlueprintCallable, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Teleport Multirotor"))
    void TeleportMultirotor(FVector NewLocation, FRotator NewRotation, bool bStopMotion = true);

    // IUBotActuator
    virtual FString GetActuatorId_Implementation() const override;
    virtual FName GetActuatorType_Implementation() const override;
    virtual bool IsActuatorReady_Implementation() const override;

    // IUBotVelocityCommandable
    virtual bool ApplyVelocityCommand_Implementation(const FUBotVelocityCommand& Command) override;
    virtual void StopMotion_Implementation() override;

    // UActorComponent
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /** The physics step clock (for tests and tools). */
    const UBot::Multirotor::FFixedStepClock& GetPhysicsClock() const { return PhysicsClock; }

    /** Gravity sampled this frame, m/s². */
    const FVector& GetGravity() const { return Gravity; }

    /**
     * Where the rotors are (body frame, m, UE axes) and how they spin, in motor order. The default is a
     * quadrotor X with 0.225 m arms in PX4 order: front right, rear left, front left, rear right.
     */
    virtual void GetRotorLayout(TArray<FVector>& OutPositionsMeters, TArray<EUBotRotorSpinDirection>& OutSpinDirections) const;

protected:
    /** Resets the model of the subclass to the current base state. bLanded says how it starts. */
    virtual void ResetModel(bool bLanded) {}

    /** Advances the model by one physics step. Implementations call MoveBody and ApplyContacts. */
    virtual void SimulateStep(double StepSeconds) PURE_VIRTUAL(UUBotMultirotorMovementComponent::SimulateStep, );

    /** Takes a clamped velocity command (heading frame, m/s; yaw rate rad/s). Returns false to reject it. */
    virtual bool AcceptVelocityCommand(const FVector& LinearHeading, double YawRate) { return false; }

    /** Stops the motion commanded so far (brake to a hover). */
    virtual void HandleStopMotion() {}

    /** The pose was moved from outside (teleport); the model takes over the new pose. */
    virtual void HandleExternalTeleport() {}

    /** Called once per frame after the pose was presented, with the frame time. */
    virtual void UpdateVisuals(double FrameSeconds);

    /** Rotor speed to show for the rotor with this index, rad/s. */
    virtual double GetVisualRotorSpeed(int32 RotorIndex) const { return 0.0; }

    /** Rotor speeds reported in the state (motor order, rad/s); empty when the model has no rotors. */
    virtual void GetRotorSpeeds(TArray<double>& OutSpeeds) const { OutSpeeds.Reset(); }

    /** Upper bound of the acceleration this model can reach, m/s² (sizes the free-space check per frame). */
    virtual double GetMaxAccelerationBound() const { return 50.0; }

    /**
     * Moves the body by DeltaMeters and turns it to NewOrientation, sweeping its collision shape and
     * sliding along what it hits. Updates PositionMeters and Orientation.
     */
    void MoveBody(const FVector& DeltaMeters, const FQuat& NewOrientation, FUBotMultirotorContacts& OutContacts);

    /**
     * Removes the part of InOutVelocity that points into the contacts, applies friction, fires impact
     * and crash events. Returns the largest impact speed along a normal, m/s.
     */
    double ApplyContacts(const FUBotMultirotorContacts& Contacts, FVector& InOutVelocity);

    /** True when the drone may land on this surface. */
    bool IsWalkable(const FHitResult& Hit) const;

    /** Changes the flight state and queues On Flight State Changed. */
    void SetFlightState(EUBotMultirotorFlightState NewState);

    /** Wind velocity at the drone now, m/s (zero when wind is disabled). */
    FVector SampleWind() const;

    /** Physics time of the step being simulated (or of the latest step), s. */
    double GetPhysicsTimeSeconds() const { return PhysicsClock.GetTimeSeconds(); }

    // State at the latest physics step: SI units on UE world axes.
    FVector PositionMeters = FVector::ZeroVector;
    FQuat Orientation = FQuat::Identity;
    FVector LinearVelocity = FVector::ZeroVector;
    /** Body frame, rad/s. */
    FVector AngularVelocity = FVector::ZeroVector;
    FVector LinearAcceleration = FVector::ZeroVector;
    EUBotMultirotorFlightState FlightState = EUBotMultirotorFlightState::Flying;
    FUBotVelocityCommand LastVelocityCommand;
    /** The latest step touched something. */
    bool bContactThisStep = false;

private:
    void StartFlight();
    void SyncExternalTeleport();
    void PresentPose(double NowSeconds, double FrameSeconds);
    void BroadcastQueuedEvents();
    FTransform ComputePresentedTransform(double ExtrapolationSeconds) const;

    struct FQueuedEvent
    {
        enum class EKind : uint8 { StateChanged, Impact, Crashed };
        EKind Kind = EKind::StateChanged;
        EUBotMultirotorFlightState NewState = EUBotMultirotorFlightState::Flying;
        EUBotMultirotorFlightState PreviousState = EUBotMultirotorFlightState::Flying;
        FHitResult Hit;
        float Speed = 0.0f;
    };

    UBot::Multirotor::FFixedStepClock PhysicsClock;
    TSharedPtr<FUBotMultirotorCollision> Collision;
    TArray<FQueuedEvent> QueuedEvents;
    TArray<TWeakObjectPtr<UUBotRotorComponent>> Rotors;
    TWeakObjectPtr<const UUBotWindSubsystem> WindSubsystem;
    FTransform LastPresentedTransform = FTransform::Identity;
    FVector Gravity = FVector(0.0, 0.0, -9.80665);
    int64 StepCount = 0;
    bool bFlightStarted = false;
    bool bFreeSpaceThisFrame = false;
    bool bContactLastStep = false;
    bool bDroppedStepsWarned = false;
};
