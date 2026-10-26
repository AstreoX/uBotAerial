#pragma once

#include "Components/SceneComponent.h"
#include "UBotBodyStateHistory.h"

/** Provider-private bounded ring. All reads and writes occur on the game thread. */
class FUBotAerialBodyStateHistory final : public IUBotBodyStateHistory
{
public:
    explicit FUBotAerialBodyStateHistory(const USceneComponent* InBody, int32 InCapacity)
        : Body(InBody)
    {
        SetCapacity(InCapacity);
    }

    void SetCapacity(int32 Capacity)
    {
        Capacity = FMath::Max(4, Capacity);
        if (Entries.Num() != Capacity)
        {
            Entries.SetNum(Capacity);
            Clear();
        }
    }

    void Clear()
    {
        Head = Count = 0;
        ++Generation;
        ++Segment;
    }

    void Invalidate() { Clear(); bValid = false; }
    bool IsForBody(const USceneComponent* Other) const { return bValid && Body.Get() == Other; }
    void BreakSegment() { ++Segment; }

    void Append(const FUBotBodyState& State)
    {
        check(IsInGameThread());
        if (!bValid || !Body.IsValid()) { return; }
        if (Count && State.TimestampSeconds <= At(Count - 1).State.TimestampSeconds)
        {
            Clear();
        }
        const int32 Index = (Head + Count) % Entries.Num();
        Entries[Index] = {State, Segment};
        if (Count == Entries.Num()) { Head = (Head + 1) % Entries.Num(); }
        else { ++Count; }
    }

    virtual bool GetRange(FUBotBodyStateHistoryRange& OutRange) const override
    {
        check(IsInGameThread());
        if (!bValid || !Body.IsValid() || !Count) { return false; }
        OutRange = {At(0).State.TimestampSeconds, At(Count - 1).State.TimestampSeconds, Generation};
        return true;
    }

    virtual EUBotBodyStateQueryResult Query(double Time, FUBotBodyState& Out, uint64& OutSegment) const override
    {
        FUBotBodyStateHistoryRange Range;
        if (!FMath::IsFinite(Time) || !GetRange(Range)) { return EUBotBodyStateQueryResult::Unavailable; }
        if (Time > Range.NewestSeconds + 1.e-9) { return EUBotBodyStateQueryResult::NotReady; }
        if (Time < Range.OldestSeconds - 1.e-9) { return EUBotBodyStateQueryResult::Unavailable; }
        int32 Low = 0, High = Count - 1;
        while (Low < High)
        {
            const int32 Mid = Low + (High - Low) / 2;
            if (At(Mid).State.TimestampSeconds < Time) { Low = Mid + 1; }
            else { High = Mid; }
        }
        const FEntry& B = At(Low);
        if (FMath::Abs(B.State.TimestampSeconds - Time) <= 1.e-9)
        {
            Out = B.State;
            Out.TimestampSeconds = Time;
            OutSegment = B.Segment;
            return EUBotBodyStateQueryResult::Exact;
        }
        if (!Low) { return EUBotBodyStateQueryResult::Unavailable; }
        const FEntry& A = At(Low - 1);
        if (A.Segment != B.Segment) { return EUBotBodyStateQueryResult::Unavailable; }
        const double Alpha = (Time - A.State.TimestampSeconds) / (B.State.TimestampSeconds - A.State.TimestampSeconds);
        Out = A.State;
        Out.PositionMeters = FMath::Lerp(A.State.PositionMeters, B.State.PositionMeters, Alpha);
        Out.Orientation = FQuat::Slerp(A.State.Orientation, B.State.Orientation, Alpha).GetNormalized();
        Out.LinearVelocityMetersPerSecond = FMath::Lerp(A.State.LinearVelocityMetersPerSecond, B.State.LinearVelocityMetersPerSecond, Alpha);
        Out.AngularVelocityRadiansPerSecond = FMath::Lerp(A.State.AngularVelocityRadiansPerSecond, B.State.AngularVelocityRadiansPerSecond, Alpha);
        Out.LinearAccelerationMetersPerSecondSquared = FMath::Lerp(A.State.LinearAccelerationMetersPerSecondSquared, B.State.LinearAccelerationMetersPerSecondSquared, Alpha);
        Out.AngularAccelerationRadiansPerSecondSquared = FMath::Lerp(A.State.AngularAccelerationRadiansPerSecondSquared, B.State.AngularAccelerationRadiansPerSecondSquared, Alpha);
        Out.bHasLinearAcceleration = A.State.bHasLinearAcceleration && B.State.bHasLinearAcceleration;
        Out.bHasAngularAcceleration = A.State.bHasAngularAcceleration && B.State.bHasAngularAcceleration;
        Out.TimestampSeconds = Time;
        OutSegment = A.Segment;
        return EUBotBodyStateQueryResult::Interpolated;
    }

private:
    struct FEntry { FUBotBodyState State; uint64 Segment = 0; };
    const FEntry& At(int32 Index) const { return Entries[(Head + Index) % Entries.Num()]; }
    TWeakObjectPtr<const USceneComponent> Body;
    TArray<FEntry> Entries;
    int32 Head = 0, Count = 0;
    uint64 Generation = 0, Segment = 0;
    bool bValid = true;
};
