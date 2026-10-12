#include "UBotMultirotorPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Controller.h"
#include "GameFramework/SpringArmComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UBotIdentityComponent.h"
#include "UBotMultirotorManualInputComponent.h"
#include "UBotMultirotorMovementComponent.h"
#include "UBotRotorComponent.h"
#include "UObject/ConstructorHelpers.h"

const FName AUBotMultirotorPawn::MovementComponentName(TEXT("Movement"));

namespace
{
    // Sizes of the built-in body, cm. The engine's basic shapes are 100 cm across.
    constexpr double BodySize = 18.0;
    constexpr double BodyHeight = 6.0;
    constexpr double ArmWidth = 2.5;
    constexpr double ArmHeight = 1.5;
    constexpr double MotorDiameter = 3.5;
    constexpr double MotorHeight = 3.0;
    constexpr double MotorZ = 2.0;
    constexpr double BladeLength = 25.4;
    constexpr double BladeWidth = 2.2;
    constexpr double BladeThickness = 0.4;
    constexpr double RotorZ = 4.0;

    const FName ColorParameter(TEXT("Color"));
    const FLinearColor FrameColor(0.06f, 0.06f, 0.07f);
    const FLinearColor MotorColor(0.25f, 0.25f, 0.27f);
    const FLinearColor BladeColor(0.85f, 0.85f, 0.85f);
    const FLinearColor MarkerColor(1.0f, 0.35f, 0.05f);

    UStaticMeshComponent* MakeVisualMesh(AUBotMultirotorPawn& Pawn, const FName Name, UStaticMesh* Mesh, USceneComponent* Parent)
    {
        UStaticMeshComponent* Component = Pawn.CreateDefaultSubobject<UStaticMeshComponent>(Name);
        Component->SetupAttachment(Parent);
        Component->SetStaticMesh(Mesh);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component->SetGenerateOverlapEvents(false);
        Component->SetCanEverAffectNavigation(false);
        return Component;
    }
}

AUBotMultirotorPawn::AUBotMultirotorPawn(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    struct FShapes
    {
        ConstructorHelpers::FObjectFinder<UStaticMesh> Cube;
        ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder;
        ConstructorHelpers::FObjectFinder<UStaticMesh> Cone;
        ConstructorHelpers::FObjectFinder<UMaterialInterface> Material;
        FShapes()
            : Cube(TEXT("/Engine/BasicShapes/Cube.Cube"))
            , Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"))
            , Cone(TEXT("/Engine/BasicShapes/Cone.Cone"))
            , Material(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"))
        {
        }
    };
    static FShapes Shapes;

    PrimaryActorTick.bCanEverTick = false;
    AutoPossessAI = EAutoPossessAI::Disabled;
    bReplicates = false;
    ShapeMaterial = Shapes.Material.Object;

    CollisionBox = CreateDefaultSubobject<UBoxComponent>(TEXT("CollisionBox"));
    CollisionBox->InitBoxExtent(FVector(29.0, 29.0, 9.0));
    CollisionBox->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);
    CollisionBox->SetSimulatePhysics(false);
    CollisionBox->SetCanEverAffectNavigation(false);
    CollisionBox->SetMobility(EComponentMobility::Movable);
    RootComponent = CollisionBox;

    VisualRoot = CreateDefaultSubobject<USceneComponent>(TEXT("VisualRoot"));
    VisualRoot->SetupAttachment(CollisionBox);

    Body = MakeVisualMesh(*this, TEXT("Body"), Shapes.Cube.Object, VisualRoot);
    Body->SetRelativeScale3D(FVector(BodySize, BodySize, BodyHeight) / 100.0);

    FrontMarker = MakeVisualMesh(*this, TEXT("FrontMarker"), Shapes.Cone.Object, VisualRoot);
    FrontMarker->SetRelativeLocation(FVector(BodySize * 0.5 + 2.0, 0.0, 0.0));
    // The cone points along +Z; pitch -90 turns it forward.
    FrontMarker->SetRelativeRotation(FRotator(-90.0, 0.0, 0.0));
    FrontMarker->SetRelativeScale3D(FVector(0.04, 0.04, 0.05));

    for (int32 Index = 0; Index < RotorCount; ++Index)
    {
        Arms.Add(MakeVisualMesh(*this, *FString::Printf(TEXT("Arm%d"), Index), Shapes.Cube.Object, VisualRoot));
        Motors.Add(MakeVisualMesh(*this, *FString::Printf(TEXT("Motor%d"), Index), Shapes.Cylinder.Object, VisualRoot));
        UUBotRotorComponent* Rotor = CreateDefaultSubobject<UUBotRotorComponent>(*FString::Printf(TEXT("Rotor%d"), Index));
        Rotor->SetupAttachment(VisualRoot);
        Rotor->SetStaticMesh(Shapes.Cube.Object);
        Rotor->RotorIndex = Index;
        Rotors.Add(Rotor);
    }

    CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
    CameraBoom->SetupAttachment(CollisionBox);
    CameraBoom->TargetArmLength = 120.0f;
    CameraBoom->SetRelativeRotation(FRotator(-15.0, 0.0, 0.0));
    CameraBoom->bUsePawnControlRotation = false;
    CameraBoom->bInheritPitch = false;
    CameraBoom->bInheritRoll = false;
    CameraBoom->bInheritYaw = true;
    CameraBoom->bEnableCameraLag = true;
    CameraBoom->CameraLagSpeed = 5.0f;

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);
    Camera->SetAutoActivate(false);
    Camera->FieldOfView = 80.0f;

    Identity = CreateDefaultSubobject<UUBotIdentityComponent>(TEXT("Identity"));
    ManualInput = CreateDefaultSubobject<UUBotMultirotorManualInputComponent>(TEXT("ManualInput"));

    // Abstract here: each drone picks the class with SetDefaultSubobjectClass, so this is null only on the abstract base.
    Movement = CreateDefaultSubobject<UUBotMultirotorMovementComponent>(MovementComponentName);
    if (Movement)
    {
        Movement->SetUpdatedComponent(CollisionBox);
    }

    PlaceRotors();
}

void AUBotMultirotorPawn::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    PlaceRotors();
    ApplyColors();
}

void AUBotMultirotorPawn::PlaceRotors()
{
    TArray<FVector> Positions;
    TArray<EUBotRotorSpinDirection> Spins;
    if (Movement)
    {
        Movement->GetRotorLayout(Positions, Spins);
    }
    else
    {
        UBot::Multirotor::GetQuadXRotorLayout(0.225, Positions, Spins);
    }

    for (int32 Index = 0; Index < RotorCount; ++Index)
    {
        const bool bUsed = Positions.IsValidIndex(Index);
        const FVector Position = bUsed ? Positions[Index] * 100.0 : FVector::ZeroVector;
        const double Reach = FVector2D(Position.X, Position.Y).Size();
        const double Heading = FMath::RadiansToDegrees(FMath::Atan2(Position.Y, Position.X));

        if (UStaticMeshComponent* Arm = Arms.IsValidIndex(Index) ? Arms[Index].Get() : nullptr)
        {
            Arm->SetRelativeLocation(FVector(Position.X * 0.5, Position.Y * 0.5, 0.0));
            Arm->SetRelativeRotation(FRotator(0.0, Heading, 0.0));
            Arm->SetRelativeScale3D(FVector(Reach, ArmWidth, ArmHeight) / 100.0);
            Arm->SetVisibility(bUsed);
        }
        if (UStaticMeshComponent* Motor = Motors.IsValidIndex(Index) ? Motors[Index].Get() : nullptr)
        {
            Motor->SetRelativeLocation(FVector(Position.X, Position.Y, MotorZ));
            Motor->SetRelativeScale3D(FVector(MotorDiameter, MotorDiameter, MotorHeight) / 100.0);
            Motor->SetVisibility(bUsed);
        }
        if (UUBotRotorComponent* Rotor = Rotors.IsValidIndex(Index) ? Rotors[Index].Get() : nullptr)
        {
            Rotor->RotorIndex = Index;
            Rotor->SpinDirection = Spins.IsValidIndex(Index) ? Spins[Index] : EUBotRotorSpinDirection::Clockwise;
            Rotor->SetRelativeLocation(FVector(Position.X, Position.Y, RotorZ));
            Rotor->SetRelativeRotation(FRotator(0.0, Heading + 90.0, 0.0));
            Rotor->SetRelativeScale3D(FVector(BladeLength, BladeWidth, BladeThickness) / 100.0);
            Rotor->SetVisibility(bUsed);
        }
    }
}

void AUBotMultirotorPawn::ApplyColors()
{
    if (!ShapeMaterial)
    {
        return;
    }
    const auto Paint = [this](UStaticMeshComponent* Component, const FLinearColor& Color)
    {
        if (Component)
        {
            if (UMaterialInstanceDynamic* Instance = Component->CreateDynamicMaterialInstance(0, ShapeMaterial))
            {
                Instance->SetVectorParameterValue(ColorParameter, Color);
            }
        }
    };
    Paint(Body, FrameColor);
    Paint(FrontMarker, MarkerColor);
    for (UStaticMeshComponent* Arm : Arms)
    {
        Paint(Arm, FrameColor);
    }
    for (UStaticMeshComponent* Motor : Motors)
    {
        Paint(Motor, MotorColor);
    }
    for (UUBotRotorComponent* Rotor : Rotors)
    {
        Paint(Rotor, BladeColor);
    }
}

void AUBotMultirotorPawn::NotifyControllerChanged()
{
    Super::NotifyControllerChanged();
    // The follow camera is for a local player only; AI and remote controllers leave it off.
    if (Camera)
    {
        Camera->SetActive(IsLocallyControlled() && IsPlayerControlled());
    }
}
