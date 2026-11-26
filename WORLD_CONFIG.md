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
        config: {frequency: 25, resolution: [1280, 720]}
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
`--headless`, `--max-steps`, `--help`, plus the performance overrides below. Installation paths
remain machine-specific overrides; headless mode and the update limit are
execution controls. An explicit `--config` takes precedence over `-m`.

Default installation paths are `~/sim/IsaacSim-6.2.0`,
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
  physics_device: cuda          # cpu or cuda
  physics_hz: 250
  render_hz: 50
  physics_threads: -1           # -1 keeps current; 0 serial dispatch; >0 workers
  profile: false
  render_quality: current      # current or fast
  direct_tensors: false        # opt-in Pegasus Warp adapter for Isaac Sim 6.2
```

Physics remains at 250 Hz in simulated time. Each app update advances a nominal
20 ms with a budget of five 4 ms physics substeps. The app loop is rate-limited
to 50 Hz; the PhysX minimum-frame-rate clamp is set to that same value. This
replaces the previously implicit timeline timing. Actual sensor render rates
still depend on payload configuration and scheduling. `--headless` also disables
viewport updates. Custom-world timeline timing is overridden intentionally.

With `profile: true`, every five wall-clock seconds `SIM_PERF` logs measured RTF, physics steps per
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

## Camera rate compatibility

The current Pegasus camera uses Isaac Sim's legacy `Camera` API, which requires
`payload.config.frequency` to divide `simulation.render_hz`. With 50 Hz rendering,
use 25, 10, 5 or 50 Hz. A requested 30 or 60 Hz is incompatible with 50 Hz rendering.
For a 30 Hz camera, use a compatible render rate such as 60 Hz instead.

When frequency is omitted, the launcher chooses the largest divisor of the render
rate up to 30 Hz (25 Hz at 50 Hz rendering). Explicit rates are validated before
starting Kit, including cameras nested inside gimbal payloads. This applies even
without ROS publishers because camera rendering still uses the same API.

## Stream a ROS camera to QGroundControl over UDP

The relay `scripts/stream_video_udp.py` runs outside Isaac Sim using system Python
with ROS sourced. It subscribes with sensor-data QoS, encodes H.264 with x264,
and sends RTP payload type 96 over UDP. It uses wall-clock timestamps, drops
frames under load and limits the send rate; it cannot create new camera frames
when the simulator is slow. The software encoder adds CPU load, so defaults are
640x360 at up to 10 FPS and 1500 kbit/s. No cv_bridge or OpenCV is required.

Install dependencies if missing:

```bash
sudo apt install python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```

With a camera payload and ROS publishing enabled, source the same ROS workspace
and `ROS_DOMAIN_ID` as the simulator. Then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/aerostack2_ws/install/setup.bash
export ROS_DOMAIN_ID=10
python3 scripts/stream_video_udp.py \
  --topic /drone0/camera/color/image_raw --host 127.0.0.1 --port 5600
```

In QGC Video settings choose `UDP h.264` and port 5600 (or UDP URL
`0.0.0.0:5600` on newer builds). Enable Low Latency Mode and disable
`Disable Video Stream When Disarmed` when testing. Replace the host with the
QGC computer's LAN IP if it is remote. Allow incoming UDP on that computer.
The relay reports when the first image is sent. Ctrl+C stops just the relay.

For another drone, select its image topic and a distinct port such as 5601.
Select the corresponding port manually in QGC; these independent UDP feeds do
not automatically follow the selected vehicle or advertise MAVLink cameras.
The depth topic is not suitable for this color-video relay.

## Automatically relay configured cameras

`launch_as2.bash --video-relay` enables camera relays; it is off by default.
The project is an `ament_python` package. Its relay implementation lives in
`project_pegasus_isaac_sim/stream_video_udp.py` and is installed with setuptools
console entry points. The source-tree script remains a compatibility wrapper.
Build it once and source the workspace before enabling the flag:

```bash
cd ~/workspaces/aerostack2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select project_pegasus_isaac_sim
source install/setup.bash
cd src/project_pegasus_isaac_sim
bash launch_as2.bash --config config/custom_world_swarm.yaml -x \
  --video-relay --no-attach
```

Add `--video-relay-host 192.168.1.20` for QGC on another computer; the default
is 127.0.0.1. With `-s`, relays still start for a simulator running separately.
Only the drones selected by `-n` get relay processes. All camera payloads
(`camera`, `hd_camera`, `monocular_camera`), including nested gimbal cameras,
are discovered in YAML order; GPS and LiDAR do not consume video ports.

| Drone | Camera indices | UDP ports |
| --- | --- | --- |
| drone0 | 0–9 | 5600–5609 |
| drone1 | 0–9 | 5610–5619 |
| drone2 | 0–9 | 5620–5629 |

The formula is `5600 + 10 * drone_number + camera_index`. The number comes from
the `droneN` name, independently of its PX4 instance, YAML order or selected subset.
For arbitrary drone names, the explicit/default PX4 `instance` supplies the number.
More than ten cameras, duplicate camera names, overlapping ports and invalid port
blocks fail before tmux launches. `ros2: false` is incompatible with camera relays.
Drones without cameras print a warning and get no relay panes.

Each relay runs as, for example:

```bash
ros2 run project_pegasus_isaac_sim stream_video_udp.py \
  --topic /drone1/front/color/image_raw --host 127.0.0.1 --port 5610 \
  --ros-args -r __node:=video_relay_front -r __ns:=/drone1
```

The image namespace follows `sensor_namespace` when overridden, and each process
has a unique ROS node name. A `video_relay` window in each drone's tmux session
holds one pane per camera and follows the session's stop/detach lifecycle.
The original launch console prints warning-level routing lines such as:

```text
[WARN] [video-relay] drone0/front: /drone0/front/color/image_raw -> udp://127.0.0.1:5600
[WARN] [video-relay] drone0/down: /drone0/down/color/image_raw -> udp://127.0.0.1:5601
[WARN] [video-relay] drone1/front: /drone1/front/color/image_raw -> udp://127.0.0.1:5610
```

The relay itself also emits a ROS warning with its topic and destination at startup.
These UDP feeds do not automatically switch when QGC's active drone changes;
select the desired port in QGC. Relays retain the lightweight 640x360/10 FPS
software-encoding defaults, so enabling many cameras increases CPU/network load.

The canonical Python ROS entry point is:

```bash
ros2 run project_pegasus_isaac_sim stream_video_udp --help
```

The `stream_video_udp.py` console alias remains supported by existing launchers.
The package uses `setup.py`, `setup.cfg`, an ament resource marker and an
`ament_python` export in `package.xml`; no CMake project is required.

## Identify the simulation bottleneck

Enable optional Python callback profiling in the world file:

```yaml
simulation:
  physics_hz: 250
  render_hz: 50
  profile: true
```

`profile` defaults to false. Every five wall-clock seconds `SIM_PROFILE` reports
the twelve largest inclusive callback times, including `app.update`, vehicle
state reads, force/torque writes, propeller visuals, sensor updates and backend
updates. `total_ms` is cumulative time in the reporting window; `avg_ms` is per
call. Nested rows overlap and must not be added together. Times include native
calls and waits; they are not GPU kernel measurements or CPU-utilization values.
Callbacks are wrapped before Pegasus registers them, including callbacks that
Kit dispatches on another Python-capable thread. Profiling adds overhead.

If vehicle callbacks dominate, optimize the identified callback. If `app.update`
is much larger than the measured callbacks, use Tracy to investigate PhysX,
rendering, Cesium, driver synchronization and other native work. Keep the same
world, drone count and settings when comparing changes. Disable profiling again
for final RTF measurements. The profiler itself does not change physics rates or force equations.

## Isaac Sim 6 camera writer scheduling

Legacy Pegasus `Camera.set_frequency` does not author `omni:sensor:tickRate`.
The project now applies `OmniSensorAPI` to each payload camera and sets that
attribute from `config.frequency` (or the validated default) during camera
initialization. This schedules native render products/ROS helper writers, in
addition to the legacy Python acquisition rate. It prints the native rate for
each camera. ROS CameraInfo and depth helpers share that camera schedule.

For the custom-writer bottleneck described in IsaacSim discussion #442, make an
A/B comparison with the same world, camera payloads, rendering and physics rates:

```yaml
ros2: true
ros2_graphical_sensors: false
```

This keeps cameras and ordinary ROS sensor publishers, while preventing this
project's camera/LiDAR ROS helper writers from being attached. Start a fresh
simulator for each comparison, without `--video-relay` in the disabled-writer run.
RTF is still printed in the simulator console. Set the switch back to `true`
(the default) for image publication/QGC. This switch does not disable separate
ROS publishing graphs already authored in a custom USD stage, or the camera's
own internal annotators. A large speed difference implicates the graphical ROS
publishing path; little difference points to other rendering/native work.

After enabling publication, try a camera `config.frequency` of 10 Hz with the
existing 50 Hz rendering rate. This change does not promise real-time performance;
compare measured RTF and verify actual image reception.

## Performance overrides and direct tensors

All six settings can be overridden for an individual run without editing YAML:

```bash
bash launch_as2.bash --config config/world.yaml --no-attach \
  --physics_device cuda --physics_hz 300 --physics_threads 0 \
  --render_quality fast --direct_tensors --profile
```

Use the same arguments with `launch_isaacsim.bash` for a simulator-only run.
Both underscore and hyphen spellings work, with separate values or `--flag=value`
for device/rate/thread/render settings. Boolean flags are `--profile`,
`--no-profile`, `--direct-tensors` and `--no-direct-tensors` (the last two also
accept underscores). These are shell flags, rather than ROS launch `name:=value`
arguments. Unspecified flags retain YAML values.

| Setting | Default | Behavior |
| --- | --- | --- |
| physics_device | cuda | Passed to `SimulationManager.setup_simulation(device=...)`; actual device printed at startup |
| physics_hz | 250 | Sets `dt=1/physics_hz`; must be a positive integer at least render_hz |
| physics_threads | -1 | Keep current setting; 0 serial PhysX dispatch; positive values select worker count |
| profile | false | Enable SIM_PERF and inclusive SIM_PROFILE output |
| render_quality | current | Keep existing rendering; fast selects RaytracedLighting and disables reflections, indirect diffuse and AO |
| direct_tensors | false | Enable the project-local Pegasus four-rotor Warp IO adapter |

`physics_threads` changes `/persistent/physics/numThreads`, not Kit's general
Carb/TBB thread pool. CUDA physics still performs CPU dispatch work. The fast
render preset reduces visual effects and can change camera images.

Direct tensor IO uses the original Pegasus `Multirotor.update` force model.
Forces remain on their original rotor/body rigid bodies in local coordinates.
The adapter accumulates local forces and torques, preserving application-point
moments, then submits one native batched wrench operation. Body pose/velocities
are packed into one readback, and propeller visuals into one articulation write.
External force/torque calls outside the vehicle update retain the original API.
The adapter targets the installed Isaac Sim 6.2 experimental Warp RigidPrim's
internal native view, independently of the rs_isaac_uav_sim Torch adapter. It
rejects unsupported layouts rather than approximating the vehicle as one body.
It rebuilds views on start and releases them on stop.

Compare the same world, sensor settings, flight state and drone count with
`--no-direct-tensors` and `--direct-tensors`. Start at 250 Hz, then test 300 Hz;
leave render_hz unchanged. Check hover, attitude response, timestamps and sensors
as well as RTF. CUDA behavior and performance require validation on the target
machine; offline tests exercise configuration, callbacks, force/state semantics
and native Warp CPU buffer operations. Profiling adds overhead, so disable it
for final RTF comparisons.

## Physical sensor sampling rates

Optional `simulation.sensor_rates` entries change Pegasus's physical sensors
before initialization. Omitted entries keep their existing Pegasus defaults;
adding a `gps` payload does not itself set the physical GPS model's rate.

```yaml
simulation:
  physics_hz: 250
  render_hz: 50
  sensor_rates:
    imu: 250
    magnetometer: 50
    barometer: 50
    gps: 10
```

Allowed names are `imu`, `magnetometer`, `barometer` and `gps`. Values must be
finite, positive numbers no greater than `physics_hz`. They are simulated-time
rates, independent of camera sampling. Startup prints `SIM_SENSOR_RATES` for
every drone so overrides can be verified. Sensor updates remain on Pegasus's
native physics-step scheduler: a requested period is rounded up to a whole
number of physics steps, and the first sample is immediate. For predictable
cadence, select rates that divide physics_hz, as in the 250 Hz example above.

This changes sensor sampling and message frequency, not physics dt or the force
model. Check PX4 estimator and flight behavior before using changed sampling
rates for missions. The native Pegasus extension is not modified.
