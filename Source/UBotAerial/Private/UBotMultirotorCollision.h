#pragma once

#include "CoreMinimal.h"
#include "CollisionQueryParams.h"
#include "CollisionShape.h"
#include "Engine/EngineTypes.h"
#include "Engine/HitResult.h"

class UPrimitiveComponent;
class UWorld;

/** Contacts found while moving a multirotor through one physics step. */
struct FUBotMultirotorContacts
{
    /** Blocking hits, in the order they were met (at most a few per step). */
    TArray<FHitResult, TInlineAllocator<4>> Hits;

    bool HasContact() const { return Hits.Num() > 0; }

    /** The hit whose normal points most upward, or nullptr. */
    const FHitResult* FindFloor() const;
};

/**
 * Swept movement of a multirotor's collision shape through the world, with sliding. Uses the
 * shape, object channel and responses of the root primitive and ignores its owner. Rotation is not
 * swept (UE sweeps translate the shape only); a shape that starts inside geometry is pushed out.
 */
class FUBotMultirotorCollision
{
public:
    /** Takes the shape and responses of Primitive. Returns false when it has no usable collision. */
    bool Initialize(const UPrimitiveComponent* Primitive);

    bool IsInitialized() const { return World.IsValid(); }

    /** True when no blocking geometry lies within MarginCm of the shape at LocationCm. */
    bool IsClear(const FVector& LocationCm, const FQuat& Rotation, double MarginCm) const;

    /**
     * Moves the shape from StartCm by DeltaCm, sliding along at most a few surfaces. Returns the end
     * location; the blocking hits go to OutContacts.
     */
    FVector Move(const FVector& StartCm, const FVector& DeltaCm, const FQuat& Rotation, FUBotMultirotorContacts& OutContacts) const;

    /** Distance from LocationCm down to blocking geometry within MaxDistanceCm, or a negative number. */
    double FindFloorDistance(const FVector& LocationCm, const FQuat& Rotation, double MaxDistanceCm, FHitResult* OutHit = nullptr) const;

    /** Radius of a sphere around the shape, cm. */
    double GetBoundingRadius() const { return BoundingRadiusCm; }

private:
    TWeakObjectPtr<UWorld> World;
    FCollisionShape Shape;
    ECollisionChannel Channel = ECC_Pawn;
    FCollisionQueryParams QueryParams;
    FCollisionResponseParams ResponseParams;
    double BoundingRadiusCm = 0.0;
};
