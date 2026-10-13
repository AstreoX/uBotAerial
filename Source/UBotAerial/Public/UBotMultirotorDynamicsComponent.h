#pragma once

#include "CoreMinimal.h"
#include "UBotActuatorInterface.h"
#include "UBotMultirotorDynamics.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotMultirotorDynamicsComponent.generated.h"

class IUBotMultirotorDynamicsBackend;
class UUBotFlightController;

/**
 * Rigid-body multirotor for flight-control research: rotors with motor lag, thrust and reaction
 * torque, air-relative drag and gravity, integrated in fixed steps on uBot sim time. A flight
 * controller turns the active command into motor commands, or motor commands drive the rotors
 * directly. Accepts velocity (heading frame), attitude, body-rate and motor commands.
 */
UCLASS(ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot Multirotor Dynamics"))
class UBOTAERIAL_API UUBotMultirotorDynamicsComponent : public UUBotMultirotorMovementComponent, public IUBotAttitudeCommandable, public IUBotMotorCommandable
{
    GENERATED_BODY()

public:
    UUBotMultirotorDynamicsComponent();

    /** Mass, inertia, rotors and drag. Changes take effect at the next Reset Flight. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamics|Airframe")
    FUBotMultirotorAirframe Airframe;

    /** Flight controller for velocity, attitude and body-rate commands. Replace it with your own subclass. */
    UPROPERTY(EditAnywhere, Instanced, BlueprintReadOnly, Category = "Dynamics|Control")
    TObjectPtr<UUBotFlightController> FlightController;

    /** The controller runs every this many physics steps and its commands are held in between. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamics|Control", meta = (ClampMin = "1"))
    int32 ControllerRateDivider = 1;

    /** Whether velocity commands are accepted. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamics|Control")
    bool bAcceptVelocityCommands = true;

    /** Whether attitude commands are accepted. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamics|Control")
    bool bAcceptAttitudeCommands = true;

    /** Whether body-rate commands are accepted. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamics|Control")
    bool bAcceptBodyRateCommands = true;

    /** Whether motor commands are accepted. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamics|Control")
    bool bAcceptMotorCommands = true;

    /** What the drone does when its commands stop: released by the sender, or older than Command Timeout Seconds. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamics|Control")
    EUBotCommandLossAction CommandLossAction = EUBotCommandLossAction::Hover;

    /** Commands older than this (sim time) count as lost, s. 0 leaves the timeout to the sender (uBot ROS components have their own). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamics|Control", meta = (ClampMin = "0.0", Units = "Seconds"))
    float CommandTimeoutSeconds = 0.0f;

    /** Descent speed of the Land command-loss action, m/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamics|Control", meta = (ClampMin = "0.05", Units = "MetersPerSecond"))
    float LandingSpeedMetersPerSecond = 0.7f;

    /** Dynamics backend by name, from the UBotAerial.DynamicsBackend extension point. Experimental; BuiltIn is the plugin's own integrator. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamics", AdvancedDisplay)
    FName DynamicsBackendName = TEXT("BuiltIn");

    /** The control mode of the latest accepted command (Velocity while hovering on its own). */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Active Control Mode"))
    EUBotFlightControlMode GetActiveControlMode() const { return Setpoint.Mode; }

    /** The setpoint the controller flies now. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Flight Setpoint"))
    FUBotFlightSetpoint GetFlightSetpoint() const { return Setpoint; }

    /** Replaces the flight controller; it is reset at once. Null falls back to the built-in cascaded controller. */
    UFUNCTION(BlueprintCallable, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Set Flight Controller"))
    void SetFlightController(UUBotFlightController* NewController);

    /** Motor commands applied in the latest step (normalized, motor order). Copies an array. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Last Motor Commands"))
    TArray<double> GetLastMotorCommands() const { return MotorCommands; }

    /** Normalized command that holds a level hover with this airframe and the world's gravity. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Hover Motor Command"))
    double GetHoverMotorCommand() const;

    // IUBotAttitudeCommandable
    virtual bool ApplyAttitudeCommand_Implementation(const FUBotAttitudeCommand& Command) override;
    virtual bool ApplyBodyRateCommand_Implementation(const FUBotBodyRateCommand& Command) override;
    virtual void ReleaseAttitudeCommand_Implementation() override;

    // IUBotMotorCommandable
    virtual bool ApplyMotorCommand_Implementation(const FUBotMotorCommand& Command) override;
    virtual int32 GetMotorCount_Implementation() const override;
    virtual void ReleaseMotorCommand_Implementation() override;

    virtual void GetRotorLayout(TArray<FVector>& PositionsMeters, TArray<EUBotRotorSpinDirection>& SpinDirections) const override;
    virtual void BeginPlay() override;

protected:
    virtual void ResetModel() override;
    virtual void SimulateStep(double StepSeconds) override;
    virtual bool AcceptVelocityCommand(const FVector& LinearHeading, double YawRate) override;
    virtual void HandleStopMotion() override;
    virtual void HandleExternalTeleport() override;
    virtual double GetVisualRotorSpeed(int32 RotorIndex) const override;
    virtual void GetRotorSpeeds(TArray<double>& OutSpeeds) const override;
    virtual double GetMaxAccelerationBound() const override;

private:
    /** The controller in use: the configured one, or the built-in one when it is missing or cannot fly Mode. */
    UUBotFlightController* ResolveController(EUBotFlightControlMode Mode);
    void ComputeCommands(double StepSeconds);
    void EnterSetpointMode(EUBotFlightControlMode Mode);
    void ApplyCommandLoss();
    void NoteCommand();
    const FUBotMultirotorState& UpdateControllerState();
    /** Velocity without the part pointing into the surfaces touched in the latest step. */
    FVector ProjectOutOfContacts(const FVector& VelocityMetersPerSecond) const;

    UPROPERTY(Transient)
    TObjectPtr<UUBotFlightController> FallbackController;

    TSharedPtr<IUBotMultirotorDynamicsBackend> Backend;
    UBot::Multirotor::FRigidBodyState Body;
    FUBotFlightSetpoint Setpoint;
    /** Heading-frame velocity of the latest velocity command; turned into the world frame every step. */
    FVector VelocityCommandHeading = FVector::ZeroVector;
    /** Vertical velocity of the latest attitude or body-rate command, m/s (used when it has no thrust). */
    double CommandedVerticalVelocity = 0.0;
    TArray<double> MotorCommands;
    TArray<double> DirectMotorCommands;
    FUBotMultirotorState ControllerState;
    TWeakObjectPtr<UUBotFlightController> ActiveController;
    double LastCommandSimTime = 0.0;
    int32 StepsUntilController = 0;
    bool bBraking = false;
    bool bCommandActive = false;
    bool bAirframeValid = false;
    bool bControllerOutputWarned = false;
    /** Normals of the surfaces touched lately: commands do not push into them. */
    TArray<FVector> ContactNormals;
    double LastContactTimeSeconds = 0.0;
};
