#include "UBotWindSubsystem.h"

#include "Engine/World.h"
#include "Math/RandomStream.h"
#include "UBotClock.h"
#include "UBotWind.h"

namespace UBot::Wind
{
    void FGustModel::Build(const FUBotWindSettings& Settings)
    {
        bHasGusts = false;
        const double T = FMath::Max(static_cast<double>(Settings.GustCorrelationTimeSeconds), 0.05);
        // From 1/20 to 2 times the inverse correlation time, evenly on a log scale.
        const double LowHz = 1.0 / (20.0 * T);
        const double HighHz = 2.0 / T;
        const double LogStep = FMath::Loge(HighHz / LowHz) / (ComponentsPerAxis - 1);
        FRandomStream Stream(Settings.Seed);
        for (int32 Axis = 0; Axis < 3; ++Axis)
        {
            double Power = 0.0;
            for (int32 Index = 0; Index < ComponentsPerAxis; ++Index)
            {
                const double Jitter = (Stream.FRand() - 0.5) * 0.4 * LogStep;
                const double Hz = LowHz * FMath::Exp(Index * LogStep + Jitter);
                const double Shape = 1.0 / (1.0 + FMath::Square(UE_DOUBLE_TWO_PI * Hz * T));
                // A component stands for the band around it, whose width grows with its frequency.
                Amplitude[Axis][Index] = FMath::Sqrt(Shape * Hz);
                AngularFrequency[Axis][Index] = UE_DOUBLE_TWO_PI * Hz;
                Phase[Axis][Index] = Stream.FRand() * UE_DOUBLE_TWO_PI;
                Power += 0.5 * FMath::Square(Amplitude[Axis][Index]);
            }
            // A sine of amplitude A has a mean square of A^2/2; scale the sum to the requested deviation.
            const double Sigma = FMath::Max(Settings.GustStandardDeviationMetersPerSecond[Axis], 0.0);
            const double Scale = Power > 0.0 ? Sigma / FMath::Sqrt(Power) : 0.0;
            for (int32 Index = 0; Index < ComponentsPerAxis; ++Index)
            {
                Amplitude[Axis][Index] *= Scale;
            }
            bHasGusts |= Sigma > 0.0;
        }
    }

    FVector FGustModel::Evaluate(double TimeSeconds) const
    {
        if (!bHasGusts)
        {
            return FVector::ZeroVector;
        }
        FVector Gust = FVector::ZeroVector;
        for (int32 Axis = 0; Axis < 3; ++Axis)
        {
            double Sum = 0.0;
            for (int32 Index = 0; Index < ComponentsPerAxis; ++Index)
            {
                Sum += Amplitude[Axis][Index] * FMath::Sin(AngularFrequency[Axis][Index] * TimeSeconds + Phase[Axis][Index]);
            }
            Gust[Axis] = Sum;
        }
        return Gust;
    }

    bool SameSettings(const FUBotWindSettings& A, const FUBotWindSettings& B)
    {
        return A.ConstantVelocityMetersPerSecond == B.ConstantVelocityMetersPerSecond
            && A.GustStandardDeviationMetersPerSecond == B.GustStandardDeviationMetersPerSecond
            && A.GustCorrelationTimeSeconds == B.GustCorrelationTimeSeconds && A.Seed == B.Seed;
    }
}

bool UUBotWindSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
    const UWorld* World = Cast<UWorld>(Outer);
    return World && (World->WorldType == EWorldType::Game || World->WorldType == EWorldType::PIE);
}

void UUBotWindSubsystem::SetScriptedWind(const FUBotWindSettings& Settings, bool bEnabled)
{
    bScripted = bEnabled;
    ScriptedSettings = Settings;
    ScriptedGusts.Build(Settings);
}

FVector UUBotWindSubsystem::GetWindVelocityAt(FVector LocationCm) const
{
    return GetWindVelocity(LocationCm, UBot::Clock::GetSimTimeSeconds(this));
}

FVector UUBotWindSubsystem::GetWindVelocity(const FVector& LocationCm, double TimeSeconds) const
{
    FVector Wind = FVector::ZeroVector;
    for (FSource& Source : Sources)
    {
        const AUBotWind* Actor = Source.Actor.Get();
        if (!Actor || !Actor->bEnabled)
        {
            continue;
        }
        // Settings may change at run time (Set Wind Settings, or edits while simulating).
        if (!UBot::Wind::SameSettings(Actor->Settings, Source.Settings))
        {
            Source.Settings = Actor->Settings;
            Source.Gusts.Build(Source.Settings);
        }
        Wind += Source.Settings.ConstantVelocityMetersPerSecond + Source.Gusts.Evaluate(TimeSeconds);
    }
    if (bScripted)
    {
        Wind += ScriptedSettings.ConstantVelocityMetersPerSecond + ScriptedGusts.Evaluate(TimeSeconds);
    }
    return Wind;
}

void UUBotWindSubsystem::RegisterWindSource(AUBotWind* Source)
{
    if (!Source || Sources.ContainsByPredicate([Source](const FSource& Entry) { return Entry.Actor.Get() == Source; }))
    {
        return;
    }
    FSource& Entry = Sources.AddDefaulted_GetRef();
    Entry.Actor = Source;
    Entry.Settings = Source->Settings;
    Entry.Gusts.Build(Entry.Settings);
}

void UUBotWindSubsystem::UnregisterWindSource(AUBotWind* Source)
{
    Sources.RemoveAll([Source](const FSource& Entry) { return !Entry.Actor.IsValid() || Entry.Actor.Get() == Source; });
}
