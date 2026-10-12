#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "UBotMultirotorPawn.generated.h"

class UBoxComponent;
class UCameraComponent;
class UMaterialInterface;
class USpringArmComponent;
class UStaticMesh;
class UStaticMeshComponent;
class UUBotIdentityComponent;
class UUBotMultirotorManualInputComponent;
class UUBotMultirotorMovementComponent;
class UUBotRotorComponent;

/**
 * A ready-made quadrotor X: a collision box as root, a body built from engine basic shapes whose
 * rotors spin with the motors, a follow camera used only while a local player controls the drone,
 * a uBot Identity, and a multirotor movement component chosen by the subclass.
 */
UCLASS(Abstract, meta = (DisplayName = "uBot Multirotor Pawn"))
class UBOTAERIAL_API AUBotMultirotorPawn : public APawn
{
    GENERATED_BODY()

public:
    AUBotMultirotorPawn(const FObjectInitializer& ObjectInitializer);

    /** Name of the movement component subobject; subclasses choose its class with SetDefaultSubobjectClass. */
    static const FName MovementComponentName;

    /** Number of rotors of the built-in body. */
    static constexpr int32 RotorCount = 4;

    /** The flight model. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Multirotor", meta = (DisplayName = "Get Multirotor Movement"))
    UUBotMultirotorMovementComponent* GetMultirotorMovement() const { return Movement; }

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void NotifyControllerChanged() override;

    /** Collision shape the drone flies with; it covers the rotor disks. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor")
    TObjectPtr<UBoxComponent> CollisionBox;

    /** Parent of every mesh of the drone (tilted by the navigation drone's visual tilt). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor")
    TObjectPtr<USceneComponent> VisualRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|Visual")
    TObjectPtr<UStaticMeshComponent> Body;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|Visual")
    TObjectPtr<UStaticMeshComponent> FrontMarker;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|Visual")
    TArray<TObjectPtr<UStaticMeshComponent>> Arms;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|Visual")
    TArray<TObjectPtr<UStaticMeshComponent>> Motors;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|Visual")
    TArray<TObjectPtr<UUBotRotorComponent>> Rotors;

    /** Follow camera boom (yaw only). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|Camera")
    TObjectPtr<USpringArmComponent> CameraBoom;

    /** Follow camera, active only while a local player controls the drone. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor|Camera")
    TObjectPtr<UCameraComponent> Camera;

    /** Robot id, ROS namespace and base frame of the drone. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor")
    TObjectPtr<UUBotIdentityComponent> Identity;

    /** Keyboard and gamepad flight while a local player controls the drone. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor")
    TObjectPtr<UUBotMultirotorManualInputComponent> ManualInput;

    /** The flight model (its class depends on the drone). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Multirotor")
    TObjectPtr<UUBotMultirotorMovementComponent> Movement;

private:
    void PlaceRotors();
    void ApplyColors();

    UPROPERTY(Transient)
    TObjectPtr<UMaterialInterface> ShapeMaterial;
};
