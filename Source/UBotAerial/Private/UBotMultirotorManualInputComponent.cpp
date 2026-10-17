#include "UBotMultirotorManualInputComponent.h"

#include "EnhancedInputSubsystems.h"
#include "EnhancedPlayerInput.h"
#include "Engine/LocalPlayer.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "UBotAerial.h"
#include "UBotClock.h"
#include "UBotMultirotorMath.h"
#include "UBotMultirotorMovementComponent.h"

namespace UBot::ManualFlight
{
    TArray<FKeyBinding> GetDefaultBindings()
    {
        return {
            { EKeys::W, 0, 1.0 },
            { EKeys::S, 0, -1.0 },
            { EKeys::D, 1, 1.0 },
            { EKeys::A, 1, -1.0 },
            { EKeys::SpaceBar, 2, 1.0 },
            { EKeys::LeftControl, 2, -1.0 },
            { EKeys::E, 3, 1.0 },
            { EKeys::Q, 3, -1.0 },
            { EKeys::Gamepad_RightY, 0, 1.0 },
            { EKeys::Gamepad_RightX, 1, 1.0 },
            { EKeys::Gamepad_LeftY, 2, 1.0 },
            { EKeys::Gamepad_LeftX, 3, 1.0 },
        };
    }

    double ApplyDeadZone(double Value, double DeadZone)
    {
        const double Zone = FMath::Clamp(DeadZone, 0.0, 0.99);
        const double Magnitude = FMath::Abs(Value);
        if (Magnitude <= Zone)
        {
            return 0.0;
        }
        return FMath::Sign(Value) * FMath::Min((Magnitude - Zone) / (1.0 - Zone), 1.0);
    }

    FUBotManualFlightAxes ConditionAxes(const FUBotManualFlightAxes& Raw, double DeadZone)
    {
        FUBotManualFlightAxes Out;
        FVector2D Horizontal(ApplyDeadZone(Raw.Forward, DeadZone), ApplyDeadZone(Raw.Right, DeadZone));
        if (Horizontal.SizeSquared() > 1.0)
        {
            Horizontal.Normalize();
        }
        Out.Forward = static_cast<float>(Horizontal.X);
        Out.Right = static_cast<float>(Horizontal.Y);
        Out.Up = static_cast<float>(ApplyDeadZone(Raw.Up, DeadZone));
        Out.Yaw = static_cast<float>(ApplyDeadZone(Raw.Yaw, DeadZone));
        return Out;
    }

    FUBotVelocityCommand MakeVelocityCommand(const FUBotManualFlightAxes& Axes, const FSettings& Settings)
    {
        FUBotVelocityCommand Command;
        Command.LinearVelocityMetersPerSecond = FVector(Axes.Forward * Settings.MaxHorizontalSpeed, Axes.Right * Settings.MaxHorizontalSpeed,
            Axes.Up * Settings.MaxVerticalSpeed);
        Command.AngularVelocityRadiansPerSecond = FVector(0.0, 0.0, Axes.Yaw * Settings.MaxYawRate);
        Command.Source = TEXT("Manual");
        return Command;
    }

    FUBotAttitudeCommand MakeAttitudeCommand(const FUBotManualFlightAxes& Axes, const FSettings& Settings, double& InOutYaw, double DeltaSeconds)
    {
        InOutYaw = UBot::Multirotor::WrapAngle(InOutYaw + Axes.Yaw * Settings.MaxYawRate * DeltaSeconds);
        // Thrust tilts toward the stick: forward tilts the nose down, right rolls right.
        const FVector Heading(FMath::Tan(Axes.Forward * Settings.MaxTilt), FMath::Tan(Axes.Right * Settings.MaxTilt), 1.0);
        const FVector Thrust = UBot::Multirotor::MakeYawQuat(InOutYaw).RotateVector(Heading);

        FUBotAttitudeCommand Command;
        Command.Orientation = UBot::Multirotor::AttitudeFromThrustDirectionAndYaw(Thrust, InOutYaw);
        Command.VerticalVelocityMetersPerSecond = Axes.Up * Settings.MaxVerticalSpeed;
        Command.Source = TEXT("Manual");
        return Command;
    }

    FUBotBodyRateCommand MakeBodyRateCommand(const FUBotManualFlightAxes& Axes, const FSettings& Settings)
    {
        FUBotBodyRateCommand Command;
        // UE rotation sense: +X raises the right side (roll left), +Y lowers the nose, +Z yaws right.
        Command.BodyRateRadiansPerSecond = FVector(-Axes.Right * Settings.MaxBodyRate, Axes.Forward * Settings.MaxBodyRate, Axes.Yaw * Settings.MaxYawRate);
        Command.VerticalVelocityMetersPerSecond = Axes.Up * Settings.MaxVerticalSpeed;
        Command.Source = TEXT("Manual");
        return Command;
    }
}

UUBotMultirotorManualInputComponent::UUBotMultirotorManualInputComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

UBot::ManualFlight::FSettings UUBotMultirotorManualInputComponent::MakeSettings() const
{
    UBot::ManualFlight::FSettings Settings;
    Settings.MaxHorizontalSpeed = MaxHorizontalSpeedMetersPerSecond;
    Settings.MaxVerticalSpeed = MaxVerticalSpeedMetersPerSecond;
    Settings.MaxYawRate = FMath::DegreesToRadians(MaxYawRateDegreesPerSecond);
    Settings.MaxTilt = FMath::DegreesToRadians(MaxTiltDegrees);
    Settings.MaxBodyRate = FMath::DegreesToRadians(MaxBodyRateDegreesPerSecond);
    Settings.DeadZone = DeadZone;
    return Settings;
}

void UUBotMultirotorManualInputComponent::BeginPlay()
{
    Super::BeginPlay();
    APawn* Pawn = Cast<APawn>(GetOwner());
    if (!Pawn)
    {
        return;
    }
    Pawn->ReceiveControllerChangedDelegate.AddDynamic(this, &UUBotMultirotorManualInputComponent::HandleControllerChanged);
    // Commands reach the flight model in the same frame.
    if (UUBotMultirotorMovementComponent* Movement = Pawn->FindComponentByClass<UUBotMultirotorMovementComponent>())
    {
        Movement->PrimaryComponentTick.AddPrerequisite(this, PrimaryComponentTick);
    }
    HandleControllerChanged(Pawn, nullptr, Pawn->GetController());
}

void UUBotMultirotorManualInputComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    ReleaseControl();
    if (APawn* Pawn = Cast<APawn>(GetOwner()))
    {
        Pawn->ReceiveControllerChangedDelegate.RemoveDynamic(this, &UUBotMultirotorManualInputComponent::HandleControllerChanged);
    }
    Super::EndPlay(EndPlayReason);
}

void UUBotMultirotorManualInputComponent::HandleControllerChanged(APawn* Pawn, AController* OldController, AController* NewController)
{
    APlayerController* Player = Cast<APlayerController>(NewController);
    if (bTakeControlWhenPossessed && Player && Player->IsLocalController())
    {
        TakeControl(Player);
    }
    else
    {
        ReleaseControl();
    }
}

void UUBotMultirotorManualInputComponent::BuildDefaultInput()
{
    if (DefaultMappingContext)
    {
        return;
    }
    DefaultMoveAction = NewObject<UInputAction>(this, TEXT("IA_UBotMove"), RF_Transient);
    DefaultMoveAction->ValueType = EInputActionValueType::Axis3D;
    DefaultYawAction = NewObject<UInputAction>(this, TEXT("IA_UBotYaw"), RF_Transient);
    DefaultYawAction->ValueType = EInputActionValueType::Axis1D;
    DefaultMappingContext = NewObject<UInputMappingContext>(this, TEXT("IMC_UBotMultirotor"), RF_Transient);

    for (const UBot::ManualFlight::FKeyBinding& Binding : UBot::ManualFlight::GetDefaultBindings())
    {
        const bool bYaw = Binding.Axis == 3;
        FEnhancedActionKeyMapping& Mapping = DefaultMappingContext->MapKey(bYaw ? DefaultYawAction : DefaultMoveAction, Binding.Key);
        if (Binding.Scale < 0.0)
        {
            Mapping.Modifiers.Add(NewObject<UInputModifierNegate>(DefaultMappingContext));
        }
        // A key or stick gives its value on X; move it to the action's axis.
        if (!bYaw && Binding.Axis > 0)
        {
            UInputModifierSwizzleAxis* Swizzle = NewObject<UInputModifierSwizzleAxis>(DefaultMappingContext);
            Swizzle->Order = Binding.Axis == 1 ? EInputAxisSwizzle::YXZ : EInputAxisSwizzle::ZYX;
            Mapping.Modifiers.Add(Swizzle);
        }
    }
}

const UInputMappingContext* UUBotMultirotorManualInputComponent::GetMappingContext() const
{
    return CustomMappingContext ? CustomMappingContext.Get() : DefaultMappingContext.Get();
}

const UInputAction* UUBotMultirotorManualInputComponent::GetMoveAction() const
{
    return CustomMappingContext && CustomMoveAction ? CustomMoveAction.Get() : DefaultMoveAction.Get();
}

const UInputAction* UUBotMultirotorManualInputComponent::GetYawAction() const
{
    return CustomMappingContext && CustomYawAction ? CustomYawAction.Get() : DefaultYawAction.Get();
}

void UUBotMultirotorManualInputComponent::TakeControl(APlayerController* PlayerController)
{
    if (bControlling && ControllingPlayer.Get() == PlayerController)
    {
        return;
    }
    ReleaseControl();
    BuildDefaultInput();
    if (ULocalPlayer* LocalPlayer = PlayerController->GetLocalPlayer())
    {
        if (UEnhancedInputLocalPlayerSubsystem* Input = LocalPlayer->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
        {
            Input->AddMappingContext(GetMappingContext(), MappingPriority);
        }
    }
    ControllingPlayer = PlayerController;
    bControlling = true;
    bYawSetpointValid = false;
    SetComponentTickEnabled(true);
}

void UUBotMultirotorManualInputComponent::ReleaseControl()
{
    if (!bControlling)
    {
        return;
    }
    if (APlayerController* Player = ControllingPlayer.Get())
    {
        if (ULocalPlayer* LocalPlayer = Player->GetLocalPlayer())
        {
            if (UEnhancedInputLocalPlayerSubsystem* Input = LocalPlayer->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
            {
                Input->RemoveMappingContext(GetMappingContext());
            }
        }
    }
    // The drone keeps flying on its own. Velocity mode stops the motion (brake and hover); Angle and Rate
    // mode release the attitude command, and the drone's Command Loss Action decides what follows.
    const AActor* Owner = GetOwner();
    if (FlightMode == EUBotManualFlightMode::Velocity)
    {
        if (UObject* Target = UBot::Actuation::FindVelocityCommandable(Owner))
        {
            IUBotVelocityCommandable::Execute_StopMotion(Target);
        }
    }
    else if (UObject* Target = UBot::Actuation::FindAttitudeCommandable(Owner))
    {
        IUBotAttitudeCommandable::Execute_ReleaseAttitudeCommand(Target);
    }
    ControllingPlayer.Reset();
    bControlling = false;
    Axes = FUBotManualFlightAxes();
    SetComponentTickEnabled(false);
}

void UUBotMultirotorManualInputComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    APlayerController* Player = ControllingPlayer.Get();
    if (!bControlling || !Player)
    {
        return;
    }
    const UEnhancedPlayerInput* PlayerInput = Cast<UEnhancedPlayerInput>(Player->PlayerInput);
    if (!PlayerInput)
    {
        if (!bWarnedNoEnhancedInput)
        {
            bWarnedNoEnhancedInput = true;
            UE_LOG(LogUBotAerial, Error, TEXT("%s: manual flight needs Enhanced Input (Project Settings > Input > Default Player Input Class)."),
                *GetPathNameSafe(this));
        }
        return;
    }

    FUBotManualFlightAxes Raw;
    if (const UInputAction* Move = GetMoveAction())
    {
        const FVector Value = PlayerInput->GetActionValue(Move).Get<FVector>();
        Raw.Forward = static_cast<float>(Value.X);
        Raw.Right = static_cast<float>(Value.Y);
        Raw.Up = static_cast<float>(Value.Z);
    }
    if (const UInputAction* Yaw = GetYawAction())
    {
        Raw.Yaw = PlayerInput->GetActionValue(Yaw).Get<float>();
    }
    Axes = UBot::ManualFlight::ConditionAxes(Raw, DeadZone);

    const AActor* Owner = GetOwner();
    const UBot::ManualFlight::FSettings Settings = MakeSettings();
    const double Now = UBot::Clock::GetSimTimeSeconds(this);
    bool bSent = false;
    switch (FlightMode)
    {
    case EUBotManualFlightMode::Velocity:
        if (UObject* Target = UBot::Actuation::FindVelocityCommandable(Owner))
        {
            FUBotVelocityCommand Command = UBot::ManualFlight::MakeVelocityCommand(Axes, Settings);
            Command.TimestampSeconds = Now;
            bSent = IUBotVelocityCommandable::Execute_ApplyVelocityCommand(Target, Command);
        }
        break;
    case EUBotManualFlightMode::Angle:
        if (UObject* Target = UBot::Actuation::FindAttitudeCommandable(Owner))
        {
            if (!bYawSetpointValid)
            {
                YawSetpoint = UBot::Multirotor::GetYaw(Owner->GetActorQuat());
                bYawSetpointValid = true;
            }
            FUBotAttitudeCommand Command = UBot::ManualFlight::MakeAttitudeCommand(Axes, Settings, YawSetpoint, DeltaTime);
            Command.TimestampSeconds = Now;
            bSent = IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Target, Command);
        }
        break;
    case EUBotManualFlightMode::Rate:
        if (UObject* Target = UBot::Actuation::FindAttitudeCommandable(Owner))
        {
            FUBotBodyRateCommand Command = UBot::ManualFlight::MakeBodyRateCommand(Axes, Settings);
            Command.TimestampSeconds = Now;
            bSent = IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Target, Command);
        }
        break;
    }
    if (!bSent && !bWarnedNoTarget)
    {
        bWarnedNoTarget = true;
        UE_LOG(LogUBotAerial, Warning, TEXT("%s: nothing on %s accepts %s commands; pick another Flight Mode."), *GetPathNameSafe(this),
            *GetNameSafe(Owner), *UEnum::GetValueAsString(FlightMode));
    }
}
