#include "UBotWind.h"

#include "Components/ArrowComponent.h"
#include "Engine/World.h"
#include "UBotWindSubsystem.h"

AUBotWind::AUBotWind()
{
    PrimaryActorTick.bCanEverTick = false;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    Arrow = CreateDefaultSubobject<UArrowComponent>(TEXT("Arrow"));
    Arrow->SetupAttachment(RootComponent);
    Arrow->ArrowColor = FColor(80, 170, 255);
    Arrow->ArrowSize = 2.0f;
    Arrow->SetHiddenInGame(true);
    Arrow->SetUsingAbsoluteRotation(true);
}

void AUBotWind::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    PointArrow();
}

void AUBotWind::PointArrow()
{
    if (Arrow && !Settings.ConstantVelocityMetersPerSecond.IsNearlyZero())
    {
        Arrow->SetWorldRotation(Settings.ConstantVelocityMetersPerSecond.Rotation());
    }
}

void AUBotWind::BeginPlay()
{
    Super::BeginPlay();
    if (UUBotWindSubsystem* Wind = GetWorld() ? GetWorld()->GetSubsystem<UUBotWindSubsystem>() : nullptr)
    {
        Wind->RegisterWindSource(this);
    }
}

void AUBotWind::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (UUBotWindSubsystem* Wind = GetWorld() ? GetWorld()->GetSubsystem<UUBotWindSubsystem>() : nullptr)
    {
        Wind->UnregisterWindSource(this);
    }
    Super::EndPlay(EndPlayReason);
}

void AUBotWind::SetWindSettings(const FUBotWindSettings& NewSettings)
{
    Settings = NewSettings;
    PointArrow();
}

void AUBotWind::SetWindEnabled(bool bNewEnabled)
{
    bEnabled = bNewEnabled;
}
