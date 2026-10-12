# CLAUDE.md

Guide for coding agents and contributors working in this repository (AstreoX/uBotAerial).

## What this is

UBotAerial is the multirotor plugin of the uBot family for Unreal Engine 5.5. It has three ready-made quadrotors for different research needs: a navigation drone with simplified flight (velocity and yaw rate), a physics drone (rigid-body dynamics with a built-in cascaded flight controller, velocity and attitude commands) and a bare airframe drone (the same dynamics, body-rate and motor commands only, the user writes the controller). It also has wind and keyboard/gamepad flight. It depends on UBotCore (the attitude and motor command interfaces arrive in UBotCore 0.1.2) and the engine plugin `EnhancedInput`; never on UBotSensor or UBotROS. UBotROS drives the drones through the uBotCore actuation interfaces.

`README.md` is the user manual, written in Chinese; keep its language. Code, comments and commit messages are English.

## Layout

| Path | Contents |
| --- | --- |
| `UBotAerial.uplugin` | Descriptor: `VersionName`, modules, `UBot` block (`Layer`, `Requires`, `EnginePlugins`, `Provides`, `Tags`) |
| `Source/UBotAerial/` | The only module (Runtime, `Default`): `Public/` API, `Private/` implementation, `Private/Tests/` tests |
| `docs/` | `api.json` (generated, see below), `brand/` logo SVGs used by `README.md` |
| `Resources/`, `Config/FilterPlugin.ini` | Plugin icon; extra files staged when the plugin is packaged |

The plugin has no content: the drones are built from `/Engine/BasicShapes` in C++. Do not add `.uasset` files or third-party meshes.

## Key types

- `UBotAerialTypes.h`: enums (`EUBotMultirotorFlightState`, `EUBotMultirotorInitialState`, `EUBotSimpleFlightTiltMode`, `EUBotRotorSpinDirection`) and structs (`FUBotMultirotorVelocityLimits`, `FUBotMultirotorState`).
- `UBotMultirotorMath.h`: `UBot::Multirotor` (fixed-step clock `FFixedStepClock`, yaw and attitude helpers, velocity clamp, body-rate integration, specific force, quad X layout). `UBotSimpleFlightModel.h`: `UBot::SimpleFlight` (the navigation drone's model).
- `UUBotMultirotorMovementComponent` (`UBotMultirotorMovementComponent.h`): abstract base. Steps the subclass model on the fixed-step clock, sweeps the root's collision shape (`Private/UBotMultirotorCollision.h`), handles contacts, landing and crashes, presents the extrapolated pose, implements `IUBotActuator` and `IUBotVelocityCommandable`. Subclasses override `SimulateStep`, `ResetModel`, `AcceptVelocityCommand`, `HandleStopMotion`.
- `UUBotSimpleFlightComponent`: the navigation drone's flight model (velocity in the heading frame, bounded braking, tilt from acceleration, Visual/Body tilt).
- `UBotMultirotorDynamics.h`: `UBot::Multirotor` rigid body (`FRigidBodyState`, `StepRigidBody`: motor lag, rotor wrench, air-relative drag, semi-implicit Euler), rotor helpers and `FThrustAllocator` (pseudo-inverse with yaw-first desaturation). Airframe, setpoint and control-mode types are in `UBotAerialTypes.h`.
- `UUBotMultirotorDynamicsComponent`: the physics drones' model. Control modes Velocity, Attitude, BodyRate and Motor (latest accepted command wins), command-loss actions, landing, take-off and crashes. Implements `IUBotAttitudeCommandable` and `IUBotMotorCommandable` from UBotCore.
- `UUBotFlightController` (abstract, instanced) and `UUBotCascadedFlightController`: controllers return normalized motor commands per step; `Compute`/`Reset` call native implementations directly and Blueprint ones through the VM.
- `UBotMultirotorDynamicsBackend.h`: experimental backend seam. `IUBotMultirotorDynamicsBackend` steps the body; factories (`IUBotMultirotorBackendFactory`) register at `UBot::Aerial::DynamicsBackendExtensionPoint`; the module registers `BuiltIn` (`Private/UBotBuiltInMultirotorBackend.h`).
- `AUBotMultirotorPawn` (abstract), `AUBotNavigationDrone`, `AUBotPhysicsDrone`, `AUBotBareAirframeDrone`: the ready-made drones. Subclasses pick the movement class with `SetDefaultSubobjectClass(AUBotMultirotorPawn::MovementComponentName)`. `UUBotRotorComponent` spins the rotor meshes. `UUBotMultirotorLibrary` has Blueprint helpers.

## Design rules

- **Sim time, fixed steps.** Flight models advance in fixed steps on uBot sim time (`UBot::Clock::GetSimTimeSeconds`), counted with an integer step index, never by summing frame deltas. Pause stops them, `UUBotClockSubsystem::StepSimulation` advances them exactly, and the same frame deltas give bit-identical results. No wall-clock time, no `FMath::Rand` (use a seeded `FRandomStream`), no looping timers.
- **Presented pose.** Movement components tick in `TG_PrePhysics` and write the pose extrapolated to the current sim time, so uBotSensor sensors (which tick in `TG_PostPhysics` and differentiate poses) see smooth motion in the same frame.
- **Units and frames.** Internally SI on UE world axes (X forward, Y right, Z up, meters, radians, `FQuat` body to world, angular rates in the UE sense: +Z yaws right). Convert centimeters only at the component boundary. ROS conversions belong to UBotROS, not here.
- **Pure math stays plain C++** in namespaces (`UBot::Multirotor`, `UBot::SimpleFlight`, `UBot::Wind`, `UBot::ManualFlight`) so it can be tested without a world. Components and Blueprint libraries are thin wrappers.
- **Commands go through interfaces.** Drones implement `IUBotVelocityCommandable` and `IUBotActuator` from UBotCore (and the attitude and motor command interfaces once UBotCore provides them). No reflection on function names, no view-target or camera changes from command code.
- **Public text never mentions roadmap plans.** The dynamics backend extension point is documented as experimental, without naming future backends.

## Build and test

The plugin has no project of its own. Build it inside a host UE 5.5 project that loads it: place or clone it at `<Host>/Plugins/uBot/uBotAerial` (next to uBotCore), or add its parent folder to `AdditionalPluginDirectories` in `<Host>.uproject`, and enable `UBotAerial`. Close the host editor before building C++.

```powershell
& '<UE_5.5>\Engine\Build\BatchFiles\Build.bat' <Host>Editor Win64 Development '-Project=<Host>.uproject' -WaitMutex -NoHotReloadFromIDE
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Host>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotAerial; Quit' '-TestExit=Automation Test Queue Empty'
```

- Every test name starts with `UBotAerial.`; narrow the run with a longer prefix, e.g. `UBotAerial.SimpleFlight`.
- While the package is unreleased, the uBotCore index lists its version as planned; `UBotCore.Packages.BuiltInIndex` accepts that and still compares the `UBot` block with the index entry. Change both together.
- Cloud sessions have no Unreal Engine and cannot compile or run tests. Do not claim a build or tests passed. State in the PR that the change is uncompiled and untested, and name the test prefix a maintainer should run.

## Conventions

- Unreal C++: four spaces, Allman braces, PascalCase, `A`/`U`/`F`/`E`/`I` type prefixes, `b`-prefixed booleans, `#pragma once`. Public headers in `Public/` with `UBOTAERIAL_API`, implementation in `Private/`. Log with `LogUBotAerial`.
- Tests: `IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotAerial<Area><Case>Test, "UBotAerial.<Area>.<Case>", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)` in `Private/Tests/UBotAerial<Area>Tests.cpp`, wrapped in `#if WITH_DEV_AUTOMATION_TESTS`. Reflected test-only types go in `Private/Tests/*TestTypes.h` (compiled in every configuration; the docs export skips them). Add or extend a test with each behavior change or fix; acceptance numbers live in the tests.
- Blueprint-facing API: functions use `Category = "uBot|Aerial|<Area>"` with `meta = (DisplayName = "Title Case Name", Keywords = "...")`; properties use readable groups such as `"Multirotor|Limits"`. Components use `ClassGroup = (uBot)` and `meta = (BlueprintSpawnableComponent, DisplayName = "uBot ...")`.
- Write a `/** ... */` comment on every Blueprint-visible class, function and property: it becomes the tooltip in `docs/api.json`. A plain `//` comment goes above `GENERATED_BODY()`, never below it.
- Blueprint-visible names (classes, structs, enums and enumerators, functions and parameters, properties, component subobject names of the drones) are what user Blueprints bind to. Renaming or removing one breaks assets in user projects. Add new API next to the old; to retire one, mark it `meta = (DeprecatedFunction)` and call it out in the PR. Never reorder enumerators.

## docs/api.json

The Blueprint API description of this plugin, generated by UBotCore's `UBotDocExport` commandlet. Never edit it by hand. Regenerate after any change to reflected API and before tagging a release (needs a local UE install). Chinese first, then English merged with it:

```powershell
$Editor = '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
& $Editor '<Host>.uproject' -run=UBotDocExport -Plugin=UBotAerial -Out=Saved/Docs/UBotAerial.zh.json -culture=zh-Hans -NullRHI
& $Editor '<Host>.uproject' -run=UBotDocExport -Plugin=UBotAerial -Out='<this repo>/docs/api.json' -MergeZh=Saved/Docs/UBotAerial.zh.json -culture=en -NullRHI
```

## Versioning and commits

- `VersionName` in `UBotAerial.uplugin` is the plugin version; release tags are `vX.Y.Z` and equal it. The index entry (`Index/index.json` in uBotCore) must match the `UBot` block in order: `requires`, `enginePlugins`, `provides`, `tags`, `layer`.
- Maintainers tag and release; do not create tags or releases unless asked.
- Never commit `Binaries/`, `Intermediate/`, `Saved/` or `DerivedDataCache/`.
- Conventional Commits: `feat(dynamics): ...`, `fix(simple-flight): ...`, `test(wind): ...`, `docs: ...`.
- The repo is public: no local paths, credentials, tokens or server addresses in code, docs, tests or logs.
