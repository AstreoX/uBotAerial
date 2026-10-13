#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "AssetRegistry/IAssetRegistry.h"
#include "Camera/CameraComponent.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Misc/PackageName.h"
#include "UBotAerialTestWorld.h"
#include "UBotIdentityComponent.h"
#include "UBotMultirotorDynamicsComponent.h"
#include "UBotMultirotorManualInputComponent.h"
#include "UBotMultirotorMath.h"
#include "UBotRotorComponent.h"
#include "UBotSimpleFlightComponent.h"

namespace UBotAerialExamplesTests
{
    using namespace UBotAerialTests;

    const TCHAR* const NavigationDronePath = TEXT("/UBotAerial/Examples/BP_UBotNavigationDrone.BP_UBotNavigationDrone_C");
    const TCHAR* const PhysicsDronePath = TEXT("/UBotAerial/Examples/BP_UBotPhysicsDrone.BP_UBotPhysicsDrone_C");
    const TCHAR* const BareAirframeDronePath = TEXT("/UBotAerial/Examples/BP_UBotBareAirframeDrone.BP_UBotBareAirframeDrone_C");
    const TCHAR* const DemoMapPackage = TEXT("/UBotAerial/Maps/L_UBotAerialDemo");

    UClass* LoadExample(const TCHAR* Path)
    {
        return LoadClass<APawn>(nullptr, Path);
    }

    APawn* SpawnExample(const FScopedAerialTestWorld& Scope, UClass* Class, const FVector& LocationCm)
    {
        FActorSpawnParameters Parameters;
        Parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        APawn* Pawn = Class ? Scope.Get()->SpawnActor<APawn>(Class, LocationCm, FRotator::ZeroRotator, Parameters) : nullptr;
        if (Pawn)
        {
            Pawn->DispatchBeginPlay();
        }
        return Pawn;
    }

    template <typename T>
    T* FindNamed(const AActor& Actor, const TCHAR* Name)
    {
        TArray<T*> Components;
        Actor.GetComponents(Components);
        T* const* Found = Components.FindByPredicate([Name](const T* Component) { return Component->GetFName() == FName(Name); });
        return Found ? *Found : nullptr;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialExamplesComponentsTest, "UBotAerial.Examples.Components", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialExamplesComponentsTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialExamplesTests;

    struct FExpected
    {
        const TCHAR* Path;
        UClass* MovementClass;
        bool bBareAirframe;
    };
    const FExpected Examples[] = {
        { NavigationDronePath, UUBotSimpleFlightComponent::StaticClass(), false },
        { PhysicsDronePath, UUBotMultirotorDynamicsComponent::StaticClass(), false },
        { BareAirframeDronePath, UUBotMultirotorDynamicsComponent::StaticClass(), true },
    };

    FScopedAerialTestWorld Scope;
    double Offset = 0.0;
    for (const FExpected& Expected : Examples)
    {
        Offset += 500.0;
        UClass* Class = LoadExample(Expected.Path);
        if (!TestNotNull(FString::Printf(TEXT("%s loads as a Pawn class"), Expected.Path), Class))
        {
            continue;
        }
        APawn* Drone = SpawnExample(Scope, Class, FVector(Offset, 0.0, 500.0));
        if (!TestNotNull(FString::Printf(TEXT("%s spawns"), Expected.Path), Drone))
        {
            continue;
        }
        const FString Name = Class->GetName();

        UBoxComponent* Box = Cast<UBoxComponent>(Drone->GetRootComponent());
        if (!TestNotNull(*FString::Printf(TEXT("%s: root is a collision box"), *Name), Box))
        {
            continue;
        }
        TestEqual(*FString::Printf(TEXT("%s: root name"), *Name), Box->GetFName(), FName(TEXT("CollisionBox")));
        TestEqual(*FString::Printf(TEXT("%s: Pawn collision profile"), *Name), Box->GetCollisionProfileName(), FName(TEXT("Pawn")));
        TestTrue(*FString::Printf(TEXT("%s: box is not simulated"), *Name), !Box->IsSimulatingPhysics());

        UUBotMultirotorMovementComponent* Movement = Drone->FindComponentByClass<UUBotMultirotorMovementComponent>();
        if (!TestNotNull(*FString::Printf(TEXT("%s: movement"), *Name), Movement))
        {
            continue;
        }
        TestTrue(*FString::Printf(TEXT("%s: movement class"), *Name), Movement->IsA(Expected.MovementClass));
        TestTrue(*FString::Printf(TEXT("%s: movement moves the root"), *Name), Movement->UpdatedComponent == Box);
        TestTrue(*FString::Printf(TEXT("%s: actuator ready"), *Name), IUBotActuator::Execute_IsActuatorReady(Movement));
        TestNotNull(*FString::Printf(TEXT("%s: identity"), *Name), Drone->FindComponentByClass<UUBotIdentityComponent>());

        const UStaticMeshComponent* Body = FindNamed<UStaticMeshComponent>(*Drone, TEXT("Body"));
        TestTrue(*FString::Printf(TEXT("%s: body mesh"), *Name), Body && Body->GetStaticMesh() && Body->GetCollisionEnabled() == ECollisionEnabled::NoCollision);

        const UCameraComponent* Camera = Drone->FindComponentByClass<UCameraComponent>();
        TestTrue(*FString::Printf(TEXT("%s: camera at the end of the boom"), *Name),
            Camera && Cast<USpringArmComponent>(Camera->GetAttachParent()) && Camera->GetAttachSocketName() == USpringArmComponent::SocketName);

        TArray<FVector> Positions;
        TArray<EUBotRotorSpinDirection> Spins;
        Movement->GetRotorLayout(Positions, Spins);
        TArray<UUBotRotorComponent*> Rotors;
        Drone->GetComponents(Rotors);
        if (!TestEqual(*FString::Printf(TEXT("%s: four rotors"), *Name), Rotors.Num(), 4))
        {
            continue;
        }
        TSet<int32> Indices;
        const bool bDynamics = Movement->IsA<UUBotMultirotorDynamicsComponent>();
        for (const UUBotRotorComponent* Rotor : Rotors)
        {
            Indices.Add(Rotor->RotorIndex);
            if (!TestTrue(*FString::Printf(TEXT("%s: rotor index %d in the layout"), *Name, Rotor->RotorIndex), Spins.IsValidIndex(Rotor->RotorIndex)))
            {
                continue;
            }
            TestTrue(*FString::Printf(TEXT("%s: rotor %d mesh"), *Name, Rotor->RotorIndex), Rotor->GetStaticMesh() != nullptr);
            TestTrue(*FString::Printf(TEXT("%s: rotor %d has no collision"), *Name, Rotor->RotorIndex), Rotor->GetCollisionEnabled() == ECollisionEnabled::NoCollision);
            TestEqual(*FString::Printf(TEXT("%s: rotor %d spin from the layout"), *Name, Rotor->RotorIndex), Rotor->SpinDirection, Spins[Rotor->RotorIndex]);
            if (bDynamics)
            {
                // The physics airframe must match the model: each rotor sits on its motor.
                const FVector InBody = Box->GetComponentTransform().InverseTransformPosition(Rotor->GetComponentLocation());
                const FVector2D Expected2D(Positions[Rotor->RotorIndex] * 100.0);
                TestTrue(*FString::Printf(TEXT("%s: rotor %d on its airframe position (%s vs %s)"), *Name, Rotor->RotorIndex, *FVector2D(InBody).ToString(), *Expected2D.ToString()),
                    FVector2D(InBody).Equals(Expected2D, 0.5));
            }
        }
        TestEqual(*FString::Printf(TEXT("%s: rotor indices 0..3"), *Name), Indices.Num(), 4);

        const UUBotMultirotorManualInputComponent* ManualInput = Drone->FindComponentByClass<UUBotMultirotorManualInputComponent>();
        if (TestNotNull(*FString::Printf(TEXT("%s: manual input"), *Name), ManualInput))
        {
            TestEqual(*FString::Printf(TEXT("%s: manual flight mode"), *Name), ManualInput->FlightMode,
                Expected.bBareAirframe ? EUBotManualFlightMode::Rate : EUBotManualFlightMode::Velocity);
        }

        if (const UUBotSimpleFlightComponent* SimpleFlight = Cast<UUBotSimpleFlightComponent>(Movement))
        {
            const UActorComponent* Visual = SimpleFlight->VisualComponent.GetComponent(Drone);
            TestTrue(*FString::Printf(TEXT("%s: visual tilt on VisualRoot"), *Name), Visual && Visual->GetFName() == FName(TEXT("VisualRoot")) && Visual != Box);
        }
        if (const UUBotMultirotorDynamicsComponent* Dynamics = Cast<UUBotMultirotorDynamicsComponent>(Movement))
        {
            TestEqual(*FString::Printf(TEXT("%s: accepts velocity"), *Name), Dynamics->bAcceptVelocityCommands, !Expected.bBareAirframe);
            TestEqual(*FString::Printf(TEXT("%s: accepts attitude"), *Name), Dynamics->bAcceptAttitudeCommands, !Expected.bBareAirframe);
            TestTrue(*FString::Printf(TEXT("%s: accepts body rates and motors"), *Name), Dynamics->bAcceptBodyRateCommands && Dynamics->bAcceptMotorCommands);
        }
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialExamplesPhysicsHoverTest, "UBotAerial.Examples.PhysicsHover", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialExamplesPhysicsHoverTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialExamplesTests;
    FScopedAerialTestWorld Scope;
    APawn* Drone = SpawnExample(Scope, LoadExample(PhysicsDronePath), FVector(0.0, 0.0, 200.0));
    UUBotMultirotorDynamicsComponent* Dynamics = Drone ? Drone->FindComponentByClass<UUBotMultirotorDynamicsComponent>() : nullptr;
    if (!TestNotNull(TEXT("Physics drone example"), Dynamics))
    {
        return false;
    }
    const FVector Start = Dynamics->GetMultirotorState().PositionMeters;
    double MaxDz = 0.0;
    double MaxTilt = 0.0;
    Scope.RunFor(5.0, FrameSeconds, [&] {
        const FUBotMultirotorState State = Dynamics->GetMultirotorState();
        MaxDz = FMath::Max(MaxDz, FMath::Abs(State.PositionMeters.Z - Start.Z));
        MaxTilt = FMath::Max(MaxTilt, UBot::Multirotor::GetTilt(State.Orientation));
    });
    TestTrue(FString::Printf(TEXT("Altitude within 1 cm (%.4f cm)"), MaxDz * 100.0), MaxDz <= 0.01);
    TestTrue(FString::Printf(TEXT("Level within 0.1 deg (%.5f)"), FMath::RadiansToDegrees(MaxTilt)), FMath::RadiansToDegrees(MaxTilt) <= 0.1);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialExamplesPhysicsVelocityStepTest, "UBotAerial.Examples.PhysicsVelocityStep", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialExamplesPhysicsVelocityStepTest::RunTest(const FString& Parameters)
{
    using namespace UBotAerialExamplesTests;
    FScopedAerialTestWorld Scope;
    APawn* Drone = SpawnExample(Scope, LoadExample(PhysicsDronePath), FVector(0.0, 0.0, 1000.0));
    UUBotMultirotorDynamicsComponent* Dynamics = Drone ? Drone->FindComponentByClass<UUBotMultirotorDynamicsComponent>() : nullptr;
    if (!TestNotNull(TEXT("Physics drone example"), Dynamics))
    {
        return false;
    }
    TestTrue(TEXT("Velocity command accepted"), SendVelocity(*Dynamics, FVector(2.0, 0.0, 0.0)));
    double MaxSpeed = 0.0;
    double MaxSideways = 0.0;
    Scope.RunFor(2.0, FrameSeconds, [&] {
        const FVector Velocity = Dynamics->GetMultirotorState().LinearVelocityMetersPerSecond;
        MaxSpeed = FMath::Max(MaxSpeed, Velocity.X);
        MaxSideways = FMath::Max(MaxSideways, FMath::Abs(Velocity.Y));
    });
    const double SpeedAt2s = Dynamics->GetMultirotorState().LinearVelocityMetersPerSecond.X;
    Scope.RunFor(2.0, FrameSeconds, [&] {
        MaxSpeed = FMath::Max(MaxSpeed, Dynamics->GetMultirotorState().LinearVelocityMetersPerSecond.X);
    });
    TestTrue(FString::Printf(TEXT("At least 1.8 m/s after 2 s (%.4f)"), SpeedAt2s), SpeedAt2s >= 1.8);
    TestTrue(FString::Printf(TEXT("Peak at most 2.2 m/s (%.4f)"), MaxSpeed), MaxSpeed <= 2.2);
    TestTrue(FString::Printf(TEXT("No sideways drift (%.4f)"), MaxSideways), MaxSideways <= 0.05);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerialExamplesDependenciesTest, "UBotAerial.Examples.Dependencies", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotAerialExamplesDependenciesTest::RunTest(const FString& Parameters)
{
    IAssetRegistry& Registry = IAssetRegistry::GetChecked();
    Registry.ScanPathsSynchronous({ TEXT("/UBotAerial") }, true);
    TArray<FAssetData> Assets;
    Registry.GetAssetsByPath(TEXT("/UBotAerial"), Assets, true);
    TestTrue(FString::Printf(TEXT("Plugin content found (%d assets)"), Assets.Num()), Assets.Num() > 0);

    TSet<FName> Packages;
    for (const FAssetData& Asset : Assets)
    {
        Packages.Add(Asset.PackageName);
    }
    for (const FName Package : Packages)
    {
        TArray<FName> Dependencies;
        Registry.GetDependencies(Package, Dependencies, UE::AssetRegistry::EDependencyCategory::Package);
        for (const FName Dependency : Dependencies)
        {
            const FString Path = Dependency.ToString();
            const bool bAllowed = Path.StartsWith(TEXT("/UBotAerial/")) || Path.StartsWith(TEXT("/Engine/")) || Path.StartsWith(TEXT("/Script/"));
            TestTrue(FString::Printf(TEXT("%s depends only on plugin, engine and script packages (%s)"), *Package.ToString(), *Path), bAllowed);
        }
    }
    TestTrue(TEXT("Demo map exists"), FPackageName::DoesPackageExist(UBotAerialExamplesTests::DemoMapPackage));
    return true;
}

#endif
