#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "UBotAerialTypes.h"
#include "UBotWindSubsystem.generated.h"

class AUBotWind;

namespace UBot::Wind
{
    /**
     * Gusts as a sum of sines per world axis: frequencies spread logarithmically around the inverse
     * correlation time, amplitudes shaped like a first-order (Dryden-like) spectrum and scaled so the
     * gusts have the requested standard deviation, phases from a stream seeded by the settings. The
     * result depends only on the settings and the time.
     */
    struct UBOTAERIAL_API FGustModel
    {
        static constexpr int32 ComponentsPerAxis = 8;

        void Build(const FUBotWindSettings& Settings);

        /** Gust velocity (without the constant part) at TimeSeconds, m/s. */
        FVector Evaluate(double TimeSeconds) const;

    private:
        double Amplitude[3][ComponentsPerAxis] = {};
        double AngularFrequency[3][ComponentsPerAxis] = {};
        double Phase[3][ComponentsPerAxis] = {};
        bool bHasGusts = false;
    };

    /** True when the settings describe the same wind. */
    UBOTAERIAL_API bool SameSettings(const FUBotWindSettings& A, const FUBotWindSettings& B);
}

/**
 * The wind of a world: the sum of every enabled uBot Wind actor and the scripted wind set from
 * Blueprint. The field is uniform in space; the location argument is there for future use.
 */
UCLASS(meta = (DisplayName = "uBot Wind Subsystem"))
class UBOTAERIAL_API UUBotWindSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()

public:
    virtual bool ShouldCreateSubsystem(UObject* Outer) const override;

    /** Sets (or with Enabled off, clears) a wind that adds to the uBot Wind actors of the level. */
    UFUNCTION(BlueprintCallable, Category = "uBot|Aerial|Wind", meta = (DisplayName = "Set Scripted Wind"))
    void SetScriptedWind(const FUBotWindSettings& Settings, bool bEnabled = true);

    /** Wind velocity now at a location (cm, world), m/s. */
    UFUNCTION(BlueprintPure, Category = "uBot|Aerial|Wind", meta = (DisplayName = "Get Wind Velocity At"))
    FVector GetWindVelocityAt(FVector LocationCm) const;

    /** Wind velocity at a location (cm, world) and a sim time (s), m/s. */
    FVector GetWindVelocity(const FVector& LocationCm, double TimeSeconds) const;

    void RegisterWindSource(AUBotWind* Source);
    void UnregisterWindSource(AUBotWind* Source);

private:
    struct FSource
    {
        TWeakObjectPtr<AUBotWind> Actor;
        FUBotWindSettings Settings;
        UBot::Wind::FGustModel Gusts;
    };

    mutable TArray<FSource> Sources;
    FUBotWindSettings ScriptedSettings;
    UBot::Wind::FGustModel ScriptedGusts;
    bool bScripted = false;
};
