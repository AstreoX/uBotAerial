<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
    <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
    <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="88">
  </picture>
</p>
<h1 align="center">uBot Aerial</h1>
<p align="center"><b>Multirotor drones for Unreal Engine 5.5, from simple navigation flight to rigid-body dynamics.</b></p>
<p align="center">
  <a href="https://github.com/AstreoX/uBotAerial/releases"><img alt="release" src="https://img.shields.io/github/v/release/AstreoX/uBotAerial?label=release"></a>
  <a href="LICENSE"><img alt="license" src="https://img.shields.io/github/license/AstreoX/uBotAerial"></a>
  <img alt="Unreal Engine 5.5" src="https://img.shields.io/badge/Unreal%20Engine-5.5-313131">
</p>
<p align="center">
  <a href="https://astreox.github.io/uBotDocs/aerial/">Documentation</a> ·
  <a href="https://github.com/AstreoX/uBotAerial/releases">Releases</a> ·
  <a href="https://github.com/users/AstreoX/projects/2">Roadmap</a> ·
  <a href="https://github.com/AstreoX/uBotAerial/issues">Issues</a>
</p>

**English** | [简体中文](README.zh-CN.md)

## What it is

uBot Aerial adds multirotor drones to Unreal Engine 5.5 as a uBot plugin. It includes three example drone Blueprints, from simplified flight for navigation research to rigid-body dynamics with a built-in flight controller, and a demo map. The plugin has no ROS dependency; drones are commanded from ROS 2 through [uBotROS](https://github.com/AstreoX/uBotROS).

- **Simplified flight:** velocity commands tracked with a first-order response and acceleration limits.
- **Rigid-body dynamics:** rotor thrust, motor lag, drag and wind; velocity, attitude, body-rate and per-motor commands.
- **Custom flight controllers:** subclass `uBot Flight Controller` in Blueprint or C++ to replace the built-in one.
- **Manual flight:** keyboard or gamepad in Velocity, Angle or Rate mode.
- **Wind and collision:** constant wind with seeded gusts; drones slide along walls and the ground.

| Example drone | Use | Commands |
| --- | --- | --- |
| `BP_UBotNavigationDrone` | Navigation, planning, perception | Velocity, yaw rate |
| `BP_UBotPhysicsDrone` | Realistic dynamics with the built-in controller | Velocity, attitude, body rate, single motor |
| `BP_UBotBareAirframeDrone` | Flight control research with your own controller | Body rate, single motor |

## Install

**uBot Manager (recommended).** [uBot Manager](https://github.com/AstreoX/uBotManager) is a Windows desktop app that installs uBot packages and their dependencies.

1. Download and run the installer from the [uBot Manager releases](https://github.com/AstreoX/uBotManager/releases).
2. Open your `.uproject` in uBot Manager, select **UBotAerial** in the Package view, then click **Install** and **Apply…**.
3. Open the project in UE 5.5. `Window > uBot` lists UBotAerial under Loaded Packages.

**Manual.** Install [uBotCore](https://github.com/AstreoX/uBotCore) first, clone this repository next to it, enable **uBot Aerial** in `Edit > Plugins` and restart the editor:

```powershell
git clone --branch v0.1.0 https://github.com/AstreoX/uBotAerial.git <Project>/Plugins/uBot/uBotAerial
```

Requirements: Unreal Engine 5.5, uBotCore `^0.1.2`, Enhanced Input (also as the project's `Default Player Input Class`), and Visual Studio 2022 for source builds.

## Quick start

Fly the physics drone in the demo map:

1. In the Content Browser, enable `Settings > Show Plugin Content`.
2. Open `/UBotAerial/Maps/L_UBotAerialDemo` and click Play. The physics drone is controlled by Player 0.
3. Keyboard: Space up, Left Ctrl down, W / A / S / D move, E / Q yaw. Gamepad: right stick moves, left stick climbs and yaws.
4. To switch drones, set `Auto Possess Player` to Player 0 on the other one and back to `Disabled` on the first.

## Documentation

The manual is on the [docs site](https://astreox.github.io/uBotDocs/aerial/), which also covers collision, the FAQ and the API reference.

| I want to ... | Go to |
| --- | --- |
| Fly the example drones in the demo map | [Example drones and demo map](https://astreox.github.io/uBotDocs/aerial/examples/) |
| Send velocity commands to the navigation drone | [Simple flight](https://astreox.github.io/uBotDocs/aerial/simple-flight/) |
| Send attitude, body-rate or motor commands | [Physics drone](https://astreox.github.io/uBotDocs/aerial/dynamics/) |
| Write my own flight controller | [Flight controller](https://astreox.github.io/uBotDocs/aerial/flight-controller/) |
| Fly with a keyboard or gamepad | [Manual flight](https://astreox.github.io/uBotDocs/aerial/manual-flight/) |
| Add wind | [Wind](https://astreox.github.io/uBotDocs/aerial/wind/) |
| Build a drone from components | [Build a drone](https://astreox.github.io/uBotDocs/aerial/build-a-drone/) |
| Command a drone from ROS 2 | [Flight commands from ROS 2](https://astreox.github.io/uBotDocs/ros/flight-commands/) |
| Read the C++ examples | [docs/CPP.md](docs/CPP.md) |
| See the flight model details and acceptance criteria | [docs/FLIGHT-MODELS.md](docs/FLIGHT-MODELS.md) |
| Build the plugin and run the tests | [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) |

## uBot series

| Repository | What it is |
| --- | --- |
| [uBotCore](https://github.com/AstreoX/uBotCore) | Simulation clock, frame header, shared interfaces, package management |
| [uBotSensor](https://github.com/AstreoX/uBotSensor) | Pose, IMU, odometry, 2D/3D lidar, RGB and depth cameras |
| [uBotROS](https://github.com/AstreoX/uBotROS) | ROS 2 over a TCP bridge: sensors, TF, /clock, cmd_vel |
| **[uBotAerial](https://github.com/AstreoX/uBotAerial)** | **Multirotor drones: simplified flight, rigid-body dynamics, wind** |
| [uBotManager](https://github.com/AstreoX/uBotManager) | Desktop app and CLI to install, update and build uBot packages |
| [uBotDocs](https://github.com/AstreoX/uBotDocs) | Documentation site: https://astreox.github.io/uBotDocs/ |

## Development

- Build: place the repository under `Plugins/uBot/uBotAerial` in a UE 5.5 C++ project, next to uBotCore, and build the project.
- Test: Unreal automation tests named `UBotAerial.*`, run from the Session Frontend (Automation tab) or the command line; see [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).
- Issues: report bugs and questions in [Issues](https://github.com/AstreoX/uBotAerial/issues).

## License

Copyright 2026 AstreoX. The code is licensed under [Apache-2.0](LICENSE); see also [NOTICE](NOTICE).

The drone model in `Content/Models` (`SM_UBotDrone_*`, `M_UBotDrone`, `T_UBotDrone_*`) is based on "Animated Drone" by HARTWEL JUNIOR KISAKA, from [Fab](https://www.fab.com/listings/c1f5f392-330a-418c-b0b0-e3c89639347f), licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The Apache License does not apply to it.

Changes made: converted from a skinned glTF model to static meshes (one body mesh and clockwise and counter-clockwise propeller meshes centered on their hubs), scaled by 2, re-oriented so the nose faces +X, all materials merged into one, textures resized to 1024x1024 and converted from specular-glossiness to metallic-roughness, normal map green channel flipped. The animations are not included.
