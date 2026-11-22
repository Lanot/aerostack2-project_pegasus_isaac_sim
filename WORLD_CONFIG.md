# Pegasus world configuration

The Isaac Sim launchers now share the Gazebo project's world-file layout:

| Mode | File | Default drone names |
| --- | --- | --- |
| Single | config/world.yaml | drone0 |
| Swarm (`-m`) | config/world_swarm.yaml | drone0, drone1, drone2 |

The supplied files are copied from `project_gazebo`. Edit `xyz` in these files
instead of passing `--position`. `rpy` is optional and uses radians.

```yaml
world_name: empty
# world_usd: /absolute/path/to/world.usda
origin:
  latitude: 50.55
  longitude: 30.22
  altitude: 100.0
scene_camera_view:               # optional; follow_camera takes precedence
  eye: [8.0, 8.0, 6.0]
  target: [0.0, 0.0, 0.0]
follow_camera: true
follow_drone: drone0              # optional; otherwise first drone
ros2: true
px4:
  enabled: true
  autolaunch: true                # false when PX4 is started externally
  model: gazebo-classic_iris

drones:
  - model_type: quadrotor_base
    model_name: drone0
    xyz: [3.0, 0.0, 0.3]
    rpy: [0.0, 0.0, 0.0]
    # instance: 0                # optional; defaults to index in drones list
    payload:
      - model_type: gps
        model_name: gps
      - model_type: hd_camera
        model_name: camera
        config: {frequency: 30, resolution: [1280, 720]}
      - model_type: lidar
        model_name: lidar
        config: {sensor_configuration: Example_Rotary}
```

`world_name: empty` creates a ground plane and light. Other values select a
Pegasus environment by catalogue name. `world_usd` opens a custom stage instead;
relative asset paths resolve against the YAML directory. Stages must be Z-up
and in meters. `quadrotor_base` maps to Pegasus's Iris asset/dynamics, not an
identical Gazebo model. Use `pegasus_model` for another Pegasus robot name or
`usd_path` for a compatible custom multirotor USD.

Each drone has an independent PX4 process, USD prim and ROS/DDS namespace.
The launchers generate a per-drone platform configuration from the world file:

| Name | Instance | PX4 system ID | ROS / DDS namespace |
| --- | --- | --- | --- |
| drone0 | 0 | 1 | /drone0 |
| drone1 | 1 | 2 | /drone1 |
| drone2 | 2 | 3 | /drone2 |

The system ID is instance + 1, independent of the name suffix. Arbitrary names
such as `leader` work too. One DDS agent serves the whole swarm. Each PX4 process
gets an independent environment snapshot so another drone cannot overwrite its
DDS namespace. All processes inherit the same `ROS_DOMAIN_ID`.

## Launch the complete system, like Gazebo

From the project directory, with ROS and your workspace sourced:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/aerostack2_ws/install/setup.bash
export ROS_DOMAIN_ID=10

# Single drone: AS2 + Isaac Sim/PX4 + one DDS agent.
bash launch_as2.bash -x

# Alternatively, three drones from world_swarm.yaml.
bash launch_as2.bash -m -x
```

In a second terminal with the same ROS environment/domain:

```bash
bash launch_ground_station.bash -t       # reads world.yaml
# Or for swarm:
bash launch_ground_station.bash -m -t

# Only after odometry and TF are ready:
python3 mission.py -n drone0 -s
```

Do not run the single and swarm commands concurrently. `-n` selects the AS2
namespaces to launch; Isaac Sim still loads all drones in the selected file.
`-c pid|df`, `-r` and `-g` remain available. In the ground-station launcher `-m`
now means swarm, matching Gazebo; motion capture uses `--mocap`.

If a tmux server already exists, update its environment before relaunching nodes:
`tmux set-environment -g ROS_DOMAIN_ID 10`. Existing nodes must be restarted.

## Launch Isaac Sim separately

```bash
bash launch_isaacsim.bash
bash launch_isaacsim.bash -m
bash launch_isaacsim.bash --config config/world.yaml

# -s prevents AS2 from launching a second simulator.
bash launch_as2.bash -s -x
bash launch_as2.bash -m -s -x
```

`--config FILE` works on all three launchers. When AS2 starts Isaac Sim itself,
it passes the selected file through automatically. Keep only one DDS agent:
omit `-x` if an agent is already running manually.

World geometry, drones, sensors and backends are configured in YAML. Set `follow_camera: true`
to follow `follow_drone` (defaults to the first drone). Position/orientation and
instance belong to each drone; payload declarations select camera/LiDAR sensors.
Omit a camera or LiDAR payload to disable that sensor. LiDAR's profile belongs in
its `config.sensor_configuration`. `sensor_namespace` and `dds_namespace` are
optional per-drone overrides for external integration; normally keep both equal
to `model_name` so generated AS2 routing matches.

Use `px4.enabled: false` to omit the MAVLink backend, `px4.autolaunch: false`
to connect to external PX4, or `ros2: false` to disable publishers and `/clock`.
These values must be YAML booleans. Do not also run legacy `launch_sitl.bash`
with Pegasus autolaunch enabled. Camera controls remain available in both YAML
and CLI. `--follow-camera`, `--no-follow-camera` and `--follow-drone NAME` work
with both `launch_isaacsim.bash` and `launch_as2.bash`; an explicit CLI value
wins, while omitted options preserve YAML. Camera overrides also appear in
`--print-config` output. Use `scripts/isaac_sim_run.py` for both
static and following cameras; the redundant `_following.py` entrypoint is removed.

```bash
bash launch_isaacsim.bash --follow-camera --follow-drone drone0
bash launch_isaacsim.bash --no-follow-camera
bash launch_as2.bash -m -x --follow-camera --follow-drone drone2
```

Gazebo-style launcher controls (`-m`, `-n`, `-s`, `-g`), config-file selection
and simulator `--headless` remain available.

The remaining simulator CLI options are `--config` (`--simulation-config-file`),
`-m`/`--swarm`, `--print-config`, `--px4-dir` (alias `--px4_dir`),
`--pegasus-dir`, `--follow-camera`/`--no-follow-camera`, `--follow-drone`,
`--headless`, `--max-steps`, and `--help`. Installation paths
remain machine-specific overrides; headless mode and the update limit are
execution controls. An explicit `--config` takes precedence over `-m`.

Default installation paths are `~/sim/IsaacSim-6.0.1`,
`~/sim/IsaacSim-Extensions/PegasusSimulator`, and `~/sim/PX4-Autopilot`.
Override using `ISAACSIM_PATH`/`ISAACSIM_PYTHON`, `PEGASUS_SIMULATOR_DIR`, and
`PX4_DIR`. Prerequisites remain the migrated Pegasus port, built PX4 and AS2,
`as2_platform_pixhawk`, matching `px4_msgs`, a compatible DDS agent, PyYAML,
tmux/tmuxinator and a working GPU driver.

## Validation and limits

```bash
# Validate/print the world without starting Kit or checking installation paths.
python3 scripts/isaac_sim_run.py --print-config
python3 scripts/isaac_sim_run.py -m --print-config
python3 -m unittest discover -s tests -v

# Bounded check on a machine with a working GPU.
bash launch_isaacsim.bash --config config/world.yaml --headless --max-steps 120
```

GPS remains a standard Pegasus flight sensor. Camera/LiDAR payload declarations
enable their graphical sensors. Nested Gazebo gimbals become fixed mounts with
an explicit warning; gimbal actuation is not simulated. `flight_time` is metadata,
not a battery/endurance model. Unknown payload types fail early. Camera/LiDAR
`config` accepts the corresponding Pegasus sensor settings.

Offline tests cover parsing, routing, per-PX4 environment isolation, spawning
with API doubles, ROS namespaces and shell launcher commands. End-to-end GPU
rendering and swarm flight are not verified in this session.

## Simulator logs without attaching to tmux

`bash launch_as2.bash -m -x --no-attach` starts the swarm in tmux and streams
combined Isaac Sim/Pegasus stdout and stderr into the original terminal. Python
output is unbuffered. The printed `/tmp/as2-pegasus-launch.*/isaacsim.log` path
retains output from startup and remains available after the viewer exits.
Press Ctrl+C to stop viewing; the tmux sessions and simulator keep running.
The viewer stays open until interrupted, including after the simulator exits.

For a launcher that returns immediately, add `--no-log-follow`. Logs are still
saved, and `tail -f /path/printed/by/launcher/isaacsim.log` follows them later.
With `-s`, no simulator is launched or followed. With `-g`, logs remain in the
simulator's separate terminal and the printed log file. AS2 node logs stay in
their respective tmux panes.

## Diagnose a PX4 swarm freeze

If three drones run with `px4.enabled: false`, re-enable PX4 and test the
Pegasus receive path separately by setting these top-level world options:

```yaml
px4:
  enabled: true
  autolaunch: true
  enable_lockstep: false
```

`enable_lockstep` defaults to `true`, retaining the existing behavior. Setting
it to `false` selects Pegasus's nonblocking MAVLink polling; the simulation
can advance without waiting for an actuator response on every physics step.
This changes synchronization and may change control timing. It does not change
the PX4 firmware's internal lockstep configuration or establish flight readiness.
Check live odometry for every drone and test one vehicle before swarm missions.
A responsive GUI alone does not prove PX4 is exchanging sensor/actuator data.

To start the complete swarm without the Isaac Sim GUI, use:

```bash
bash launch_as2.bash -m -x --headless --no-follow-camera --no-attach
```

`--headless` is forwarded through tmuxinator to `SimulationApp`. It also works
when launching the simulator directly: `bash launch_isaacsim.bash -m --headless`.
With `-s`, AS2 does not launch a simulator, so this option has no effect on an
already running simulator.

## Physics and application timing

The launcher explicitly configures physics and timeline/app timing before Play,
with defaults matching this optional world section:

```yaml
simulation:
  physics_hz: 250
  render_hz: 50
```

Physics remains at 250 Hz in simulated time. Each app update advances a nominal
20 ms with a budget of five 4 ms physics substeps. The app loop is rate-limited
to 50 Hz; the PhysX minimum-frame-rate clamp is set to that same value. This
replaces the previously implicit timeline timing. Actual sensor render rates
still depend on payload configuration and scheduling. `--headless` also disables
viewport updates. Custom-world timeline timing is overridden intentionally.

Every five wall-clock seconds, `SIM_PERF` logs measured RTF, physics steps per
wall-clock second, app updates per wall-clock second and physics steps per update.
`SIM_TIMING` reports requested rates at startup. For the default configuration,
RTF near 1, physics_hz near 250 and app_hz near 50 indicate real-time progress.
A timing configuration does not guarantee the hardware achieves those rates;
compare measurements before missions. Pauses and startup affect the first sample.

## Cesium USD worlds

`world_usd` opens the stage, but a standalone app must also load Cesium's USD
schemas and streaming extension. The supplied custom-world configs now include:

```yaml
extension_paths:
  - /home/tolik/sim/IsaacSim-Extensions/CesiumGS-cesium-omniverse-linux-x86_64-v0.30.0
extensions:
  - cesium.omniverse
```

Paths must name extension search roots containing both `cesium.omniverse` and
`cesium.usd.plugins`; relative paths resolve against the world YAML. The launcher
registers Cesium schemas during Kit startup, enables Fabric, suppresses the Cesium
startup window and enables the streaming extension before opening the USD stage.
Missing directories or extensions fail explicitly. Cesium ion authorization and
tileset settings remain in your existing stage/application configuration.

```bash
bash launch_as2.bash --config config/custom_world_swarm.yaml -x --no-attach
```

Start with the GUI to verify tile streaming and camera location before testing
headless behavior. The stage-loading wait covers USD assets; Cesium streams tiles
asynchronously afterward. This does not establish a terrain-collision readiness
gate or align the stage georeference with Pegasus's GPS origin. Empty-world
configs load no Cesium extensions.
