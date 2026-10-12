#pragma once

#if WITH_DEV_AUTOMATION_TESTS

#include "CoreMinimal.h"
#include "Components/BoxComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerState.h"
#include "GameFramework/WorldSettings.h"
#include "UBotActuatorInterface.h"
#include "UBotMultirotorMovementComponent.h"

namespace UBotAerialTests
{
    constexpr float FrameSeconds = 1.0f / 60.0f;

    // Transient game world, initialized for play and ticked by hand; torn down when the scope ends.
    // Same pattern as the uBotSensor tests.
    class FScopedAerialTestWorld
    {
    public:
        UE_NONCOPYABLE(FScopedAerialTestWorld);

        FScopedAerialTestWorld()
            : World(UWorld::CreateWorld(EWorldType::Game, false))
            , CachedFrameCounter(GFrameCounter)
        {
            if (World && GEngine)
            {
                World->AddToRoot();
                GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
                World->InitializeActorsForPlay(FURL());
            }
        }

        ~FScopedAerialTestWorld()
        {
            if (!World)
            {
                return;
            }
            if (Pauser)
            {
                SetPaused(false);
            }
            if (World->AreActorsInitialized())
            {
                for (TActorIterator<AActor> It(World); It; ++It)
                {
                    It->RouteEndPlay(EEndPlayReason::LevelTransition);
                }
            }
            if (GEngine)
            {
                GEngine->DestroyWorldContext(World);
            }
            World->DestroyWorld(false);
            World->RemoveFromRoot();
            GFrameCounter = CachedFrameCounter;
        }

        UWorld* Get() const { return World; }

        void Tick(float DeltaSeconds) const
        {
            World->Tick(LEVELTICK_All, DeltaSeconds);
            // A tick function runs at most once per GFrameCounter value.
            ++GFrameCounter;
        }

        /** Ticks frames of DeltaSeconds until Seconds of sim time passed; calls PerFrame after each. */
        void RunFor(double Seconds, float DeltaSeconds, TFunctionRef<void()> PerFrame) const
        {
            const int32 Frames = FMath::RoundToInt(Seconds / DeltaSeconds);
            for (int32 Frame = 0; Frame < Frames; ++Frame)
            {
                Tick(DeltaSeconds);
                PerFrame();
            }
        }

        void RunFor(double Seconds, float DeltaSeconds = FrameSeconds) const
        {
            RunFor(Seconds, DeltaSeconds, [] {});
        }

        double Now() const { return World->GetTimeSeconds(); }

        /** Gravity of the world, m/s². */
        FVector Gravity() const { return FVector(0.0, 0.0, World->GetGravityZ() / 100.0); }

        /** A static blocking box with its center at CenterCm. */
        AActor* SpawnBox(const FVector& CenterCm, const FVector& HalfExtentCm, const FRotator& Rotation = FRotator::ZeroRotator) const
        {
            AActor* Owner = World->SpawnActor<AActor>();
            UBoxComponent* Box = NewObject<UBoxComponent>(Owner, TEXT("Box"));
            Box->SetBoxExtent(HalfExtentCm, false);
            Box->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
            Owner->SetRootComponent(Box);
            Box->SetWorldLocationAndRotation(CenterCm, Rotation);
            Box->RegisterComponent();
            return Owner;
        }

        /** A large floor whose top is at height TopCm. */
        AActor* SpawnFloor(double TopCm = 0.0) const
        {
            return SpawnBox(FVector(0.0, 0.0, TopCm - 50.0), FVector(100000.0, 100000.0, 50.0));
        }

        /** Spawns an actor of class T without beginning play (set properties, then call DispatchBeginPlay). */
        template <typename T>
        T* Spawn(const FVector& LocationCm, const FRotator& Rotation = FRotator::ZeroRotator) const
        {
            FActorSpawnParameters Parameters;
            Parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            return World->SpawnActor<T>(LocationCm, Rotation, Parameters);
        }

        /** Spawns an actor of class T and begins play on it. */
        template <typename T>
        T* SpawnAndBegin(const FVector& LocationCm, const FRotator& Rotation = FRotator::ZeroRotator) const
        {
            T* Actor = Spawn<T>(LocationCm, Rotation);
            if (Actor)
            {
                Actor->DispatchBeginPlay();
            }
            return Actor;
        }

        bool SetPaused(bool bPaused)
        {
            AWorldSettings* Settings = World ? World->GetWorldSettings(false, false) : nullptr;
            if (!Settings)
            {
                return false;
            }
            if (bPaused && !Pauser)
            {
                Pauser = World->SpawnActor<APlayerState>();
            }
            Settings->SetPauserPlayerState(bPaused ? Pauser : nullptr);
            return World->IsPaused() == bPaused;
        }

    private:
        UWorld* World = nullptr;
        APlayerState* Pauser = nullptr;
        uint64 CachedFrameCounter = 0;
    };

    // What uBotSensor's IMU computes from poses sampled once per frame: velocity and acceleration by
    // finite differences, specific force in the component frame.
    struct FFrameImuProbe
    {
        /** False until two velocities are known. */
        bool Sample(const USceneComponent& Component, double TimeSeconds, const FVector& Gravity)
        {
            const FVector Position = Component.GetComponentLocation() / 100.0;
            if (!bHasPosition)
            {
                PreviousPosition = Position;
                PreviousTime = TimeSeconds;
                bHasPosition = true;
                return false;
            }
            const double Dt = TimeSeconds - PreviousTime;
            if (Dt <= 0.0)
            {
                return false;
            }
            const FVector CurrentVelocity = (Position - PreviousPosition) / Dt;
            PreviousPosition = Position;
            PreviousTime = TimeSeconds;
            if (!bHasVelocity)
            {
                Velocity = CurrentVelocity;
                bHasVelocity = true;
                return false;
            }
            Acceleration = (CurrentVelocity - Velocity) / Dt;
            Velocity = CurrentVelocity;
            SpecificForce = Component.GetComponentQuat().UnrotateVector(Acceleration - Gravity);
            return true;
        }

        FVector Velocity = FVector::ZeroVector;
        FVector Acceleration = FVector::ZeroVector;
        FVector SpecificForce = FVector::ZeroVector;

    private:
        FVector PreviousPosition = FVector::ZeroVector;
        double PreviousTime = 0.0;
        bool bHasPosition = false;
        bool bHasVelocity = false;
    };

    inline FUBotVelocityCommand MakeVelocityCommand(const FVector& LinearMetersPerSecond, double YawRate = 0.0)
    {
        FUBotVelocityCommand Command;
        Command.LinearVelocityMetersPerSecond = LinearMetersPerSecond;
        Command.AngularVelocityRadiansPerSecond = FVector(0.0, 0.0, YawRate);
        Command.Source = TEXT("Test");
        return Command;
    }

    inline bool SendVelocity(UUBotMultirotorMovementComponent& Movement, const FVector& LinearMetersPerSecond, double YawRate = 0.0)
    {
        return IUBotVelocityCommandable::Execute_ApplyVelocityCommand(&Movement, MakeVelocityCommand(LinearMetersPerSecond, YawRate));
    }
}

#endif
