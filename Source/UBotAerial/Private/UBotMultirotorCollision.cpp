#include "UBotMultirotorCollision.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

namespace
{
    // Gap kept between the shape and a surface it stops at, cm.
    constexpr double ContactSkinCm = 0.1;
    // Extra push when leaving a penetration, cm.
    constexpr double PushOutSkinCm = 0.125;
    // Sweeps per move: a first contact, then slides along up to two more surfaces.
    constexpr int32 MaxSweeps = 4;
}

const FHitResult* FUBotMultirotorContacts::FindFloor() const
{
    const FHitResult* Floor = nullptr;
    for (const FHitResult& Hit : Hits)
    {
        if (!Floor || Hit.Normal.Z > Floor->Normal.Z)
        {
            Floor = &Hit;
        }
    }
    return Floor;
}

bool FUBotMultirotorCollision::Initialize(const UPrimitiveComponent* Primitive)
{
    World.Reset();
    if (!Primitive || !Primitive->GetWorld() || !Primitive->IsCollisionEnabled())
    {
        return false;
    }
    Shape = Primitive->GetCollisionShape();
    if (Shape.IsNearlyZero())
    {
        return false;
    }
    Channel = Primitive->GetCollisionObjectType();
    QueryParams = FCollisionQueryParams(SCENE_QUERY_STAT(UBotMultirotorMove), false, Primitive->GetOwner());
    ResponseParams = FCollisionResponseParams();
    Primitive->InitSweepCollisionParams(QueryParams, ResponseParams);
    if (const AActor* Owner = Primitive->GetOwner())
    {
        QueryParams.AddIgnoredActor(Owner);
    }
    BoundingRadiusCm = Shape.GetExtent().Size();
    World = Primitive->GetWorld();
    return true;
}

bool FUBotMultirotorCollision::IsClear(const FVector& LocationCm, const FQuat& Rotation, double MarginCm) const
{
    UWorld* QueryWorld = World.Get();
    if (!QueryWorld)
    {
        return true;
    }
    // A sphere around the shape grown by the margin contains every position the shape can reach.
    const FCollisionShape Probe = FCollisionShape::MakeSphere(static_cast<float>(BoundingRadiusCm + FMath::Max(MarginCm, 0.0)));
    return !QueryWorld->OverlapBlockingTestByChannel(LocationCm, Rotation, Channel, Probe, QueryParams, ResponseParams);
}

FVector FUBotMultirotorCollision::Move(const FVector& StartCm, const FVector& DeltaCm, const FQuat& Rotation, FUBotMultirotorContacts& OutContacts) const
{
    UWorld* QueryWorld = World.Get();
    if (!QueryWorld)
    {
        return StartCm + DeltaCm;
    }

    FVector Location = StartCm;
    FVector Remaining = DeltaCm;
    for (int32 Sweep = 0; Sweep < MaxSweeps && !Remaining.IsNearlyZero(1e-6); ++Sweep)
    {
        FHitResult Hit;
        const FVector End = Location + Remaining;
        if (!QueryWorld->SweepSingleByChannel(Hit, Location, End, Rotation, Channel, Shape, QueryParams, ResponseParams) || !Hit.bBlockingHit)
        {
            Location = End;
            break;
        }

        OutContacts.Hits.Add(Hit);
        if (Hit.bStartPenetrating)
        {
            Location += Hit.Normal * (Hit.PenetrationDepth + PushOutSkinCm);
            continue;
        }
        Location = Hit.Location + Hit.Normal * ContactSkinCm;
        Remaining *= 1.0 - Hit.Time;
        const double IntoSurface = FVector::DotProduct(Remaining, Hit.Normal);
        if (IntoSurface < 0.0)
        {
            Remaining -= Hit.Normal * IntoSurface;
        }
    }
    return Location;
}

double FUBotMultirotorCollision::FindFloorDistance(const FVector& LocationCm, const FQuat& Rotation, double MaxDistanceCm, FHitResult* OutHit) const
{
    UWorld* QueryWorld = World.Get();
    if (!QueryWorld)
    {
        return -1.0;
    }
    FHitResult Hit;
    const FVector End = LocationCm - FVector(0.0, 0.0, FMath::Max(MaxDistanceCm, 0.0));
    if (!QueryWorld->SweepSingleByChannel(Hit, LocationCm, End, Rotation, Channel, Shape, QueryParams, ResponseParams) || !Hit.bBlockingHit)
    {
        return -1.0;
    }
    if (OutHit)
    {
        *OutHit = Hit;
    }
    return Hit.bStartPenetrating ? 0.0 : FMath::Max(LocationCm.Z - Hit.Location.Z, 0.0);
}
