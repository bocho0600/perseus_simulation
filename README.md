# Running the Gazebo Simulation

This readme file explains how to launch the Gazebo Simulation environment and control the rover.

The environment (ROS 2 Jazzy + Gazebo) is fully managed by [pixi](https://pixi.sh),
using the [RoboStack](https://robostack.github.io/) conda channels. No system ROS
installation is required, and there is no longer a nix devshell.

> **Note:** avoid `source /opt/ros/*/setup.bash` (e.g. the `a` alias) in the
> shell you run pixi from — a system ROS leaking in conflicts with the RoboStack
> packages and crashes Gazebo. The pixi tasks defend against this automatically
> via `scripts/pixi-run.sh` (which strips any `/opt/ros/*` paths), but running
> from a clean shell is still the cleanest option.

## One-time setup

Install pixi (if you don't have it):

```
curl -fsSL https://pixi.sh/install.sh | bash
```

The first `pixi run` will download the ROS 2 / Gazebo packages and build the
workspace automatically. The lunar moonscape model is also downloaded from the
Gazebo Fuel server the first time you launch — see the note at the bottom.

## Terminal 1: Launch Simulation Environment

From the repository root:

```
pixi run sim
```

This builds the colcon workspace (incrementally) and then runs
`ros2 launch perseus_simulation perseus_sim.launch.py`.

## Terminal 2: Control Perseus via keyboard

```
pixi run teleop
```

## Other pixi tasks

| Command | Description |
| --- | --- |
| `pixi run build` | Build the colcon workspace only |
| `pixi run sim` | Build + launch the Gazebo simulation |
| `pixi run teleop` | Drive the rover with the keyboard |
| `pixi run clean` | Remove `build/`, `install/`, and `log/` |
| `pixi shell` | Drop into an activated ROS 2 environment |

## Note on start first time

The lunar moonscape model is downloaded the first time you launch the simulation.

It can take a while to download from the gazebo server and there are
only warnings about missing world files to indicate this process is happening (no clear indication of progress of the download).

Therefore when you first launch, please have an internet connection and let gazebo operate for up to 20 minutes to complete the download. It can be useful to monitor your network connection whilst this is happening (e.g. with btop).
