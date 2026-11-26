# Project Pegasus Simulator in Isaac Sim

This project aims at installing and running a photorealistic simulation of a drone using [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator), [Nvidia Isaac Sim](https://developer.nvidia.com/isaac/sim) and [Aerostack 2](https://github.com/aerostack2/aerostack2). Here a short video of the resulting simulation:

https://github.com/user-attachments/assets/a465b64a-a23c-4e7f-9d47-6908acafc153

https://github.com/user-attachments/assets/dde015c6-1d9d-4e26-9ad8-0b7a1ecae015

Credits to https://github.com/gdaudt, as the creator of the indoor ship environment shown in this video.

## World and swarm configuration

See [WORLD_CONFIG.md](WORLD_CONFIG.md) for the current config-based workflow.
`config/world.yaml` and `config/world_swarm.yaml` now drive Isaac Sim, AS2 and
the ground station. `launch_as2.bash` starts the simulator by default (like the
Gazebo project); use `-s` when it is already running. The default single drone is
now **drone0**. The older single-drone instructions below are retained as history.

The optional `--video-relay` flag starts ROS video relays for cameras declared in
the selected world. See [WORLD_CONFIG.md](WORLD_CONFIG.md) for the one-time package
build, automatic per-drone UDP port allocation and console routing warnings.

## CUSTOM Isaac Sim 6.x (Ubuntu 24.04 / ROS 2 Jazzy)

The standalone entrypoint now targets the `dev_6.0.1_custom` Pegasus port.
It uses `PegasusInterface.initialize_world()`, Kit updates and timeline callbacks;
there is no dependency on the removed `omni.isaac.core.World` API. Enablement of
Pegasus and the ROS bridge is explicit, so no saved Isaac Sim GUI settings are needed.
The entrypoint delegates to `scripts/isaacsim_app.py`, which loads
`scripts/isaacsim_scene.py` after Kit and extension startup; a following camera selected
through YAML or `--follow-camera` tracks live Pegasus physics state.

Prerequisites:

- Isaac Sim 6.0.1 with a working NVIDIA driver.
- The migrated Pegasus checkout at `~/IsaacSim-Extensions/PegasusSimulator`.
  The project does not patch or modify that installation.
- Built PX4 SITL at `~/PX4-Autopilot` (the binary is checked before starting Kit).
- ROS 2 Jazzy, built Aerostack2, `as2_platform_pixhawk` and `px4_msgs` matching
  the PX4 firmware's DDS message definitions. These are separate installations;
  this example does not install them or change the PX4 checkout.
- Micro XRCE-DDS Agent compatible with that PX4 firmware, plus tmux/tmuxinator
  when using the AS2 launcher. The historical v1.14 instructions below are not a
  recommendation to use v1.14 message definitions with newer firmware.

Start the simulator (Pegasus autostarts PX4; do not also run `launch_sitl.bash`):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/workspaces/aerostack2_ws/src/project_pegasus_isaac_sim
bash launch_isaacsim.bash
```

Set spawn positions, world assets, camera behavior and backends in the world YAML,
then select that file:

```bash
bash launch_isaacsim.bash --config config/world.yaml
bash launch_isaacsim.bash -m
```

For example, `follow_camera: true` enables tracking; `follow_drone` selects the
drone. `world_usd` opens a custom Z-up stage in meters and each drone's `xyz`
sets its spawn position. The supplied worlds default to an empty ground plane.
See [WORLD_CONFIG.md](WORLD_CONFIG.md) for the complete schema.

Optional installation overrides:

```bash
export ISAACSIM_PATH="$HOME/sim/IsaacSim-6.0.1"
export PEGASUS_SIMULATOR_DIR="$HOME/sim/IsaacSim-Extensions/PegasusSimulator"
export PX4_DIR="$HOME/sim/PX4-Autopilot"
```

All options: `python3 scripts/isaac_sim_run.py --help` (does not start Kit).
The script also accepts the existing `--px4_dir` argument.

In a second terminal, source your built AS2 and Pixhawk platform workspaces, then:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/aerostack2_ws/install/setup.bash
# Source a separate as2_platform_pixhawk workspace here if applicable.
cd ~/workspaces/aerostack2_ws/src/project_pegasus_isaac_sim
bash launch_as2.bash -s -x
```

`-x` now runs `MicroXRCEAgent udp4 -p 8888`, rather than opening a physical
serial device. Alternatively run that command manually and omit `-x`.
The selected world config supplies drone names, PX4 instances and namespaces.
Use `px4.autolaunch: false` for external SITL or `px4.enabled: false` to omit
the MAVLink backend. Configure the external PX4 process's DDS namespace to match.

The ROS bridge publishes `/clock`. Configure `use_sim_time:=true` on AS2 nodes
that should follow simulation time. Camera frequency now uses the actual Pegasus
`frequency` option. LiDAR creation uses the Isaac Sim 6 experimental RTX API,
while Pegasus retains sensor scheduling. Project-local ROS helper graphs publish
RGB, depth, camera info, point clouds and laser scans using the current bridge
API, avoiding the legacy backend's CameraInfo list/reshape incompatibility.

Bounded runtime check (requires a working GPU):

```bash
bash launch_isaacsim.bash --config config/world.yaml --headless --max-steps 120
```

For a physics-only check, use a separate world file with `px4.enabled: false`,
`ros2: false`, `world_name: empty`, and no camera/LiDAR payloads. A
`PEGASUS_RUN_COMPLETE` marker confirms completion of the update loop, not flight
or telemetry. Kit closes in a `finally` block on failure.

Validation of this migration was limited to offline API checks, syntax checks and
mocked lifecycle regression checks against the installed 6.0.1 source. Rendering,
sensor contents, PX4 DDS telemetry and AS2 flight still need a working GPU session.

## Legacy installation reference (Isaac Sim 4.2 / Humble)

The remaining instructions below describe the original upstream setup. Use the
6.x instructions above with the migrated entrypoints.

## Installation

This project has been tested in Ubuntu 22.04, Isaac Sim 4.2.0 and ROS 2 (Humble). It is recommended to have an Nvidia GPU with CUDA enabled capabilities and enough VRAM (tested on Nvidia RTX4090).

Note: Check Troubleshooting section if you have installation problems and remember to deactivate conda for all the building steps performed in the following sections (how to deactivate is also in Troubleshooting section)

### Clone this repository
```
mkdir ~/workspace
cd ~/workspace
https://github.com/Lanot/aerostack2-project_pegasus_isaac_sim.git
git checkout dev/isaac-sim-6

# Or origin:
# git clone https://github.com/aerostack2/project_pegasus_isaac_sim
```

### Install dependencies and remove conda activation from .bashrc
```
sudo apt-get install tmux tmuxinator
```

If conda is installed already in your system, remove everything related to conda from .bashrc and run this command:
```
conda config --set auto_activate_base false
```


## Origin Isaac Sim 4.x (Ubuntu 22.04 / ROS 2 Humble)
### Install Pegasus Simulator and Isaac Sim 4.2.0

Follow this instructions to install Isaac Sim 4.2.0 (https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)

You should be able now to run:
```
# Run the bundled python interpreter and see if it prints on the terminal "Hello World."
$ISAACSIM_PYTHON -c "print('Hello World.')"

# Run the python interpreter and check if we can run a script that starts the simulator and adds cubes to the world
$ISAACSIM_PYTHON ${ISAACSIM_PATH}/standalone_examples/api/omni.isaac.core/add_cubes.py
```
Install Pegasus Simulator (https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html#installing-the-pegasus-simulator)

### Build PX4
Important note about PX4: set parameter `COM_OF_LOSS_T` to a high value (e.g. 15 seconds) (more info [here](https://docs.px4.io/v1.14/en/flight_modes/offboard.html#offboard-parameters)). Sometimes simulation can be slowed down (not noticeable) but this small de-sync can trigger a failsafe of PX4. One way of doing it is modifying `<root_to_px4>/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/px4-rc.params` and include:

```
#!/bin/sh
# shellcheck disable=SC2154

param set-default COM_OF_LOSS_T 15
```

After, build and install PX4 (release 1.14.3) as explained in the Pegasus Docs (https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html#installing-px4-autopilot)


### Install ROS 2 (Humble)

Install full version of ROS 2 (Humble) as in https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Note: remember to upgrade the libraries explained in the ROS 2 docs if you don't want ROS 2 to mess with your ubuntu basic libraries.

### [Optional] Installation check
Download QGroundControl (https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu)

To check the installation is successful until this point:
```
source /opt/ros/humble/setup.bash
cd ~/workspace/project_pegasus_isaac_sim
$ISAACSIM_PYTHON ./scripts/isaac_sim_run.py
```
Now the simulation with Pegasus Simulator and Isaac Sim will be running. Run now QGroundControl:
```
cd <qgroundcontrol_path>
./QGroundControl.AppImage
```
In QGroundControl you can now press "Take Off" and after, set navgation points for the drone to move. If this is working, you have everything until this point properly installed and working.

### Build Micro XRCE-DDS Agent & Client
Here it is being build from source but it can be installed in other ways (check: https://docs.px4.io/main/en/ros2/user_guide.html).

Note1: Remember to have conda deactivated properly.

Note2: If build error, fix CMakeList.txt as in https://github.com/PX4/PX4-Autopilot/issues/24477#issuecomment-2710838732
```
cd ~/workspace
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Install Aerostack2
Instal Aerostack2 base:
```
sudo apt install ros-humble-aerostack2
```

Clone and build `ros-humble-as2-platform-pixhawk` (if you have conda, remove libtiff `conda uninstall libtiff` and deactivate environment previous to build):
```
mkdir -p ~/workspace/as2_platform_pixhawk_ws/src
cd ~/workspace/as2_platform_pixhawk_ws/src
git clone https://github.com/aerostack2/as2_platform_pixhawk
git clone https://github.com/PX4/px4_msgs
cd px4_msgs
git checkout remotes/origin/release/1.14
cd ../..
rosdep init
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Config [WIP]
Currently, the environment can be changed in `scripts/isaac_sim_run.py`, in the line:
```
# Launch one of the worlds provided by NVIDIA
self.pg.load_environment(SIMULATION_ENVIRONMENTS["Warehouse with Shelves"])
```

There are a few possible environments (https://pegasussimulator.github.io/PegasusSimulator/source/features/environments.html). We are working currently in enabling custom environments.

## Execution

### Launch Simulation with Teleoperation

Open a terminal and launch Isaac Sim simulation:
```
source /opt/ros/humble/setup.bash
cd ~/workspace/project_pegasus_isaac_sim
$ISAACSIM_PYTHON scripts/isaac_sim_run.py --px4_dir <root_to_px4>/PX4-Autopilot
```

Open another terminal and launch Micro XRCE-DDS Client:
```
source /opt/ros/humble/setup.bash
cd ~/workspace/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
```

Open another terminal and launch Aerostack 2 (to detach from tmux you can do CTRL-B + D, or to move in the different windows CTRL-B + 0,1,2..):

```
source /opt/ros/humble/setup.bash
source ~/workspace/as2_platform_pixhawk_ws/install/setup.bash
cd ~/workspace/project_pegasus_isaac_sim
./launch_as2.bash -n drone1
OR using swarm and custom world config, with auto started MicroXRCEAgent:
./launch_as2.bash -x -m --config ./config/custom_world_swarm.yaml
```

Now, you can teleoperate manually the drone in Isaac Sim, follow the instructions in the graphical interface that appeared. There are many topics being published, including RGB camera and LIDAR.

To finalize the execution, close Isaac Sim, exit Micro XRCE-DDS Client and close tmux sessions with the following command:

```bash
./stop_tmuxinator_as2.bash
```

### Launch a mission [WIP]
There are several missions that can be executed:

- **AS2 keyboard teleoperation control**: You can use the keyboard teleoperation launched with the ground station, using the flag `-t`:
  ```bash
  ./launch_ground_station.bash -n drone1 -t
  ```
- **AS2 Python API single drone mission**: You can execute a mission that used AS2 Python API, launching the mission with:
  ```bash
  python3 mission.py -n drone1
  ```
- **AS2 Python API multi-drone mission**: You can execute a mission that used AS2 Python API, launching the mission with:
  ```bash
  python3 mission_swarm.py
  ```
- **AS2 Python API single drone mission using GPS**: You can execute a mission that used AS2 Python API with GPS, launching the mission with:
  ```bash
  python3 mission_gps.py -n drone1
  ```
- **AS2 Mission Interpreter single drone mission**: You can execute a mission that used AS2 Mission Interpreter, launching the mission with:
  ```bash
  python3 mission_interpreter.py -n drone1
  ```
- **AS2 Behavior Trees single drone mission**: You can execute a mission that used AS2 Behavior Trees, launching the mission with:
  ```bash
  python3 mission_behavior_tree.py -n drone1
  ```

You can force the end of all tmux sessions with the command:
```bash
tmux kill-server
```

If you are using gnome-terminal, you can end the execution by closing the terminal.

## Developers guide

All projects in aerostack2 are structured in the same way. The project is divided into the following directories:

- **tmuxinator**: Contains the tmuxinator launch file, which is used to launch all aerostack2 nodes.
  - **aerostack2.yaml**: Tmuxinator launch file for each drone. The list of nodes to be launched is defined here.
  - **ground_station.yaml**: Tmuxinator launch file for the ground station. The list of nodes to be launched is defined here.
- **config**: Contains the configuration files for the launchers of the nodes in the drones.
- **config_ground_station**: Contains the configuration files for the launchers of the nodes in the ground station.
- **launch_as2.bash**: Script to launch nodes defined in *tmuxinator/aerostack2.yaml*.
- **launch_ground_station.bash**: Script to launch nodes defined in *tmuxinator/ground_station.yaml*.
- **mission_\*.py**: Differents python mission files that can be executed.
- **stop_tmuxinator_as2.bash**: Script to stop all nodes launched by *launch_as2.bash*.
- **stop_tmuxinator_ground_station.bash**: Script to stop all nodes launched by *launch_ground_station.bash*.
- **stop_tmuxinator.bash**: Script to stop all nodes launched by *launch_as2.bash* and *launch_ground_station.bash*.
- **rosbag/record_rosbag.bash**: Script to record a rosbag. Can be modified to record only the topics that are needed.
- **trees\***: Contains the behavior trees that can be executed. They can be selected in the *aerostack2.yaml* file.
- **utils**: Contains utils scripts for launchers.

Both python and bash scripts have a help message that can be displayed by running the script with the `-h` option. For example, `./launch_as2.bash -h` will display the help message for the `launch_as2.bash` script.

**Note**: For knowing all parameters for each launch, you can execute the following command:

```bash
ros2 launch my_package my_launch.py -s
```

Also, you can see them in the default config file of the package, in the *config* folder. If you want to modify the default parameters, you can add the parameter to the config file.

## Troubleshooting
### Conda installation and compilation issues
Remove from `.bashrc` the conda init for the base environment. Source again `.bashrc` and run `sudo ldconfig`.

### PX4 process still running after closing simulation
```
killall px4
```


### Video Relay from ROS2 to QGC via UDP
```
python3 scripts/stream_video_udp.py \
  --topic /drone0/camera/color/image_raw \
  --host 127.0.0.1 \
  --port 5600
```
