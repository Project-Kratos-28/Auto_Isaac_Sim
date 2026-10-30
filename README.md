# athena_rover — Isaac Sim Digital Twin

A digital twin of **athena_rover**, a 4-wheel independent steering (4WIS / swerve) rover with
rocker-bogie suspension, built in **NVIDIA Isaac Sim 6.0.1** and controlled through **ROS 2 Jazzy**.

The sim streams a **ZED 2i** stereo camera (RGB + depth + registered point cloud), a
**Livox Mid-360**-style 3D lidar (approximated with an RTX Lidar) plus an IMU, and publishes
ground-truth **odometry**, so the ROS 2 topic layout matches the real hardware and the same
control/perception stack runs in sim and on the physical rover.

On top of that, **`athena_nav_sim`** runs the team's autonomy stack against the sim:
**Nav2** waypoint following with obstacle avoidance, and **GLIM** LiDAR-IMU SLAM building the map
live as the rover drives (see [Autonomous navigation & mapping](#autonomous-navigation--mapping-athena_nav_sim)).

The repo also includes **Differential Drive** and **Ackermann Drive** OmniGraph controllers, built and
validated as standalone drive-mode prototypes (see [Differential & Ackermann Drive](#differential--ackermann-drive)).
These give the team simpler skid-steer and car-like steering options to fall back on or compare against
the swerve controller, and reuse the same `/cmd_vel`-style ROS 2 interface pattern.

> **Setting this up on a new machine?** Follow [`docs/SETUP_FROM_SCRATCH.md`](docs/SETUP_FROM_SCRATCH.md):
> every step, in order, with a check after each one (written so a Claude agent can run it end to end).

---

## Stack

| Component | Version / Notes |
|---|---|
| OS | Ubuntu 24.04 |
| Isaac Sim | 6.0.1 (Kit 110.1.2) — primary; Differential/Ackermann prototypes also validated on 5.0.0 and 6.1.0, see version note below |
| ROS 2 | Jazzy (Isaac Sim's ROS 2 bridge uses its bundled Jazzy libraries) |
| Navigation | Nav2 (Jazzy apt packages), configured by the team's `kratos_nav` (repo `auto_fastlio2`, branch `jazzy-nvblox`) |
| SLAM | GLIM 1.2.2 (koide3 PPA) + patched `glim_ros` overlay (fixes phantom walls in `/glim_ros/map`) |
| Camera | ZED 2i (via Stereolabs ZED SDK + zed-ros2-wrapper) |
| ZED SDK | 5.4.1 (needs CUDA + cuDNN + TensorRT) |
| Lidar | Livox Mid-360 — **approximated** via RTX Lidar (Ouster OS0 base, reconfigured to Mid-360 specs) |

> **Version note:** the Differential/Ackermann drive graphs were prototyped and screen-recorded on
> Isaac Sim **5.0.0** test scenes (a stock TurtleBot3 Burger for differential drive, a standalone
> Ackermann test rover) before being ported into the main `athena_rover` stage on **6.1.0**. OmniGraph
> node names/versions and physics schemas (e.g. Articulation Root) can differ across 5.0.0 / 6.0.1 /
> 6.1.0 — if a graph node shows as unresolved after opening on a different install, check the node's
> version against what's installed and re-add it from the node library if needed. See also the
> [Known issues](#known-issues--gotchas) entry on cross-version stages.

---

## Repository layout

```
Isaac_sim/
├── README.md
├── docs/
│   └── SETUP_FROM_SCRATCH.md   # full new-machine setup, step by step
├── src/
│   ├── athena_controller/      # 4WIS swerve controller, joystick mapper + bringup launch
│   ├── athena_nav_sim/         # Nav2 + GLIM on the sim: launch, tools, behavior trees, RViz config
│   └── circle_test/            # test node: constant /cmd_vel (spins in place)
├── description/
│   └── athena_description/     # rover URDF + meshes + the Isaac scene (athena_rover-6_3/athena_rover-6.usda)
├── glim/                       # GLIM config for the sim (glim_config/) + its README
└── isaac/
    └── demo_1/                 # older saved Isaac Sim scene (.usd)
```

> **Not included in this repo** (clone/install separately — see Setup):
> the ZED SDK, the `zed-ros2-wrapper` / `zed-ros2-interfaces` packages, the `zed-isaac-sim` Isaac
> extension, GLIM, and the team's `auto_fastlio2` (Nav2 config, pcd2pgm, waypoint manager) and
> `Auto_Gui` (operator GUI) repos.

### Development workflow

Packages are developed and built in **`~/athena_ws`** (sourced from `~/.bashrc`). Once something works,
its folder is copied from `~/athena_ws/src/` into this repo's `src/` and pushed. The copies in this
repo are the "published" versions — editing them does not change what runs.

---

## Prerequisites

The short version (the full, ordered list with exact commands is in
[`docs/SETUP_FROM_SCRATCH.md`](docs/SETUP_FROM_SCRATCH.md)):

1. **Isaac Sim 6.0.1** installed at `~/isaacsim` (or 6.1.0 for the ported Differential/Ackermann graphs — see version note above).
2. **ROS 2 Jazzy** (`ros-jazzy-desktop`) plus **Nav2** (`ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`).
3. **ZED SDK 5.4.1+** (`https://www.stereolabs.com/developers/release`).
   The installer will prompt to download AI models + optimize them — accept both.
   Requires a working **CUDA toolkit**, **cuDNN**, and **TensorRT**.
4. **zed-isaac-sim** extension (streams the simulated camera into the ZED SDK):
   `https://github.com/stereolabs/zed-isaac-sim` — built locally at **`~/Desktop/zed-isaac-sim`**
   (the scene references the ZED 2i model by a relative path to that exact location; extension
   internal name `sl.sensor.camera`).
5. **GLIM** from the koide3 PPA, plus the patched `glim_ros` overlay (see [GLIM](#glim-slam)).
6. For the Ackermann drive test topic: the **`ackermann_msgs`** package
   (`sudo apt install ros-jazzy-ackermann-msgs`) — not part of the core install.

> The scene expects this repo at **`~/Isaac_sim`** (relative references to the ZED model and the drive graphs).

---

## Setup

### 1. Clone this repo and create the ROS 2 workspace

```bash
git clone git@github.com:Project-Kratos-28/Auto_Isaac_Sim.git ~/Isaac_sim
mkdir -p ~/athena_ws/src
cp -r ~/Isaac_sim/src/athena_controller ~/Isaac_sim/src/circle_test ~/Isaac_sim/src/athena_nav_sim ~/athena_ws/src/
```

### 2. Clone the ZED ROS 2 driver (third-party)

```bash
cd ~/athena_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/stereolabs/zed-ros2-interfaces.git
```

### 3. Apply the required ZED wrapper config tweak

Edit `zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml`:

```yaml
depth:
  depth_mode: 'NEURAL_LIGHT'      # SDK 5.x depth is neural-only; NEURAL_LIGHT avoids the TRT-not-found crash (already the default upstream)
pos_tracking:
  pos_tracking_enabled: false     # the simulated ZED stream carries no IMU, so on-board pos tracking crashes
```

### 4. Build

```bash
cd ~/athena_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

> If the wrapper build complains `Specify CUDA_TOOLKIT_ROOT_DIR`, export `CUDA_TOOLKIT_ROOT_DIR`
> (e.g. `/usr/local/cuda`) before rebuilding.

### 5. Source (add to `~/.bashrc`)

```bash
source /opt/ros/jazzy/setup.bash
source ~/athena_ws/install/setup.bash
```

The Nav2/GLIM pieces need two more workspaces (`~/kratos_gui_ws`, `~/glim_ros_fix_ws`) — see
[`docs/SETUP_FROM_SCRATCH.md`](docs/SETUP_FROM_SCRATCH.md).

---

## Running

### 1. Isaac Sim
Launch Isaac with `~/isaacsim/isaac-sim.sh` (its `setup_ros_env.sh` selects the bundled Jazzy ROS 2
libraries; no sourcing needed). Open
**`description/athena_description/urdf/athena_rover-6_3/athena_rover-6.usda`** and press **Play once**.
The stage contains the rover, a ground plane, a walled room (~8 × 8 m), a cone ~1.7 m ahead and a cube
~2.2 m to the left of the start pose. The obstacles have **no colliders**: the rover drives through
them, but they block the lidar.

Let the rover settle on the ground **before** pressing Play (a free-fall at t=0 skews odometry and
puts the rover below the RViz grid — see Known Issues).

Action Graphs in the stage:

| Graph | Purpose |
|---|---|
| **Time** | publishes `/clock` (sim time) |
| **rover_controller** | ROS2 Subscribe Joint State → Articulation Controller: applies `/joint_command` to the joints |
| **Odom** | Compute Odometry → `/odom` and the `odom → base_link` TF (ground truth), sim-time stamped |
| **Lidar** | Create Render Product (`OS0/sensor`) → RTX Lidar Helper → `/livox/points` (frame `livox_frame`) |
| **IMU_Publisher** | Isaac Read IMU → `/livox/imu` (frame `imu_link`) |
| **camera_streamer** | streams the ZED 2i to the ZED SDK (port 30000) |
| **Differential_Drive** / **ACKERMAN_DRIVE_GRAPH** | referenced from `athena rover for 6.0.1 ackermann and differential drive/`; `Differential_Drive` subscribes to `/cmd_vel` |

> **Important:** every ROS publish node that emits a timestamp must have `Isaac Read Simulation Time`
> wired into its `timeStamp` input, and every ROS 2 node below must run with `use_sim_time:=true`.
> Otherwise timestamps don't match and RViz drops the point cloud ("0 points" in the `odom` frame).

### 2. Bring up the ROS 2 stack (one command)

```bash
ros2 launch athena_controller bringup.launch.py            # add use_zed:=true for the camera
```

This starts:
- the **swerve controller** (`/cmd_vel` → `/joint_command`)
- a **static transform** `base_link → zed_camera_link`
- a **static transform** `base_link → livox_frame` (`--z 0.09`, matches the OS0 mount height)
- **RViz** (with `use_sim_time:=true`)
- with `use_zed:=true`: the **ZED wrapper** (`sim_mode:=true`, `sim_address:=127.0.0.1`, `sim_port:=30000`, `use_sim_time:=true`)

The ZED is **off by default**, so bringup also works on a machine without the ZED SDK/wrapper. Nav2 and
GLIM only need the lidar and IMU.

The lidar cloud (`/livox/points`) and IMU (`/livox/imu`) come straight from the Isaac Action Graphs —
no external driver needed (unlike the ZED, which goes through the wrapper).

Or run the wrapper standalone for debugging:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i sim_mode:=true \
  sim_address:=127.0.0.1 sim_port:=30000 use_sim_time:=true
```

### 3. Spin test

```bash
ros2 run circle_test circle_drive     # publishes linear.x = 0.0, angular.z = 0.5 on /cmd_vel: spins in place
```

Set `msg.linear.x` in `circle_test/circle_drive.py` to a non-zero value (e.g. `0.2`) to drive a circle instead.

### 4. Verify everything is on the same clock

```bash
ros2 topic echo /clock --once                                                   # small, increasing sim time
ros2 topic echo /tf --once                                                      # odom->base_link, sim-time stamp
ros2 topic hz /livox/points                                                     # ~5-7 Hz
ros2 topic echo /zed/zed_node/point_cloud/cloud_registered --field header.stamp --once   # with use_zed:=true
```

All stamps should be small matching sim-time values. Then set RViz **Fixed Frame = `odom`** and the
clouds appear. Add a **PointCloud2** on `/livox/points` to see the lidar.

---

## Autonomous navigation & mapping (`athena_nav_sim`)

Runs the Kratos autonomy stack (the same `kratos_nav` Nav2 config, `pcd2pgm` and GLIM
`waypoint_manager` that run on the real rover) against the sim, up to `/cmd_vel`.

### Two modes

| `localization:=` | TF / localization | Global map | Use it for |
|---|---|---|---|
| `ground_truth` (default) | Isaac's `odom → base_link` + identity `map → odom` | blank grid, filled live by the lidar obstacle layer | testing Nav2 alone (no SLAM in the loop) |
| `glim` | GLIM localizes; `glim_map_odom` publishes `map → odom` on top of Isaac's odometry (like AMCL) | GLIM's map → `pcd2pgm` → `/map`, starting from a blank map (`map_seed`) | the real rover pipeline: map built while driving |

Because Isaac's odometry is perfect, in `glim` mode any change in `map → odom` is GLIM's own error.
Measured over full loops of the room: worst **0.15–0.21 m / ~2.5°**.

### What the package contains

| File | What it does |
|---|---|
| `launch/nav_sim.launch.py` | Everything below in one launch. Starts kratos_nav's 7 Nav2 nodes itself (not via `nav.launch.py`, which forces the no-spin trees), with `nav2_params_sim.yaml` adjusted at launch for the sim |
| `behavior_trees/*_spin_no_backup.xml` | kratos_nav's trees with a **90° Spin recovery** added; BackUp stays removed (the rover never reverses). Recoveries also run on "start occupied" |
| `athena_nav_sim/send_waypoints.py` | Drives through `x,y` waypoints with `navigate_through_poses` and reports how close the rover got. `--frame odom` (ground-truth coordinates) or `--relative` (rover frame) |
| `athena_nav_sim/rover_bridge.py` | Sim stand-in for the rover's `rover_bridge`: `/rover_bridge/set_auto` (the GUI's AUTO button) gates Nav2's `cmd_vel_auto` onto `/cmd_vel`; a joystick stick drops back to MANUAL |
| `athena_nav_sim/glim_map_odom.py` | `map → odom` from GLIM's pose while Isaac owns `odom → base_link` |
| `athena_nav_sim/map_seed.py` | Blank `/map` until pcd2pgm has a real one (Nav2 can't plan without a map) |
| `rviz/nav_sim.rviz` | GLIM 3D map, live scan, `/map`, global costmap, footprint, plan |

### Run

Isaac playing, `bringup.launch.py` running, then:

```bash
source ~/kratos_gui_ws/install/setup.bash
source ~/glim_ros_fix_ws/install/local_setup.bash        # glim mode only; must be sourced LAST

# Phase 1 — Nav2 on ground truth
ros2 launch athena_nav_sim nav_sim.launch.py

# Phase 2 — GLIM localization + mapping (keep the rover still until GLIM logs
# "initial IMU state estimation result", ~5 s)
ros2 launch athena_nav_sim nav_sim.launch.py localization:=glim glim_viewer:=false rviz:=true
```

Send waypoints (another terminal, same `source` lines):

```bash
ros2 run athena_nav_sim send_waypoints 1.2,-1.5                          # map frame
ros2 run athena_nav_sim send_waypoints --frame odom -2.2,-1.5 0.8,-1.6 -0.8,1.0   # ground-truth frame
ros2 run athena_nav_sim send_waypoints --relative 2,0                     # 2 m ahead
```

Or click **2D Goal Pose** in RViz (launch with `use_bridge:=false` so Nav2 drives without the AUTO switch).

Useful launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `localization` | `ground_truth` | `ground_truth` or `glim` |
| `rviz` | `false` | open RViz with `rviz/nav_sim.rviz` |
| `use_bridge` | `true` | `true`: Nav2 only drives in AUTO (`/rover_bridge/set_auto`); `false`: Nav2 → `/cmd_vel` directly |
| `min_obstacle_height` / `max_obstacle_height` | `-0.5` / `2.5` | costmap obstacle band (the sim lidar gets almost no ground returns) |
| `glim_viewer` | `true` | GLIM's own OpenGL viewer (uses GPU, slows Isaac's lidar) |
| `glim_gpu` | `false` | GLIM's CUDA modules — **leave off**: they starve Isaac's RTX lidar (7 Hz → 0.4 Hz) |
| `submap_keyframes` | `5` | keyframes per GLIM submap (rover config: 15); fewer = `/map` sooner |
| `map_min_height` | `0.05` | lowest z (GLIM map frame) that goes into `/map`; floor is ~-0.43 |

### Sim-only adjustments the launch makes (and why)

All applied to temporary copies at launch; the GLIM config in `glim/` and the `kratos_nav` files are untouched.

- **GLIM on CPU:** the GPU modules compete with Isaac's RTX lidar; the lidar fell from 7 Hz to 0.4 Hz and GLIM diverged.
- **DISPLACEMENT keyframes (0.5 m / 0.5 rad):** with GLIM's OVERLAP strategy, a ~8 m room never produces a new keyframe (the lidar sees everything from everywhere), so submaps never close and `/glim_ros/map` is never published.
- **Crop box around the rover body:** the lidar hits the rover's own wheel modules (up to ~560 points per scan while steering). Nav2's costmaps clear under the footprint, but GLIM keeps them as a trail of phantom obstacles.
- **`map_min_height` 0.05:** GLIM's map keeps sparse floor returns, smeared up to ~0 m by the rocker-bogie pitching; the live costmap raytraces them away, the static map doesn't.
- **No velocity deadband, no nvblox layer, `odom_topic: /odom`** (ground-truth mode).

### Operator GUI (Auto_Gui)

The team's operator GUI (`kratos_gui`, repo `Auto_Gui`) builds on Jazzy and works with the sim in
`glim` mode (GLIM's `waypoint_manager` provides LOG WAYPOINT / missions; `rover_bridge` provides AUTO).
Start it with sim time:

```bash
KRATOS_USE_SIM_TIME=1 ros2 run kratos_gui kratos_gui --ros-args -p use_sim_time:=true
```

`KRATOS_USE_SIM_TIME` needs a small local patch to `kratos_gui/src/missionpanel.cpp` (in
[`docs/SETUP_FROM_SCRATCH.md`](docs/SETUP_FROM_SCRATCH.md)). GUI-driven missions in the sim are **not
fully tested yet**.

### GLIM (SLAM)

Sim GLIM config and background: [`glim/README.md`](glim/README.md). Two extra requirements on Jazzy:

- **Patched `glim_ros` overlay:** apt GLIM 1.2.2's `rviz_viewer` corrupts `/glim_ros/map` (phantom
  walls). The fix (`auto_fastlio2/glim/glim_ros_fix/rviz_viewer_submap_guard.patch`) is built into
  `~/glim_ros_fix_ws` and sourced **last**. Check: `ros2 pkg prefix glim_ros` must print
  `.../glim_ros_fix_ws/install/glim_ros`.
- **`waypoint_manager`** GLIM extension (from `auto_fastlio2`), built in `~/kratos_gui_ws`.

---

## Differential & Ackermann Drive

Two additional OmniGraph-based drive controllers were built and tested as alternatives to the 4WIS
swerve controller above. Unlike the swerve controller, both use Isaac Sim's **built-in** controller
nodes (`Differential Controller`, `Ackermann Controller`) directly inside the Action Graph — no
custom Python IK node is required.

### Differential Drive

Graph: **`Differential_Drive`**

| Node | Role |
|---|---|
| `on_playback_tick` | drives graph execution each sim step |
| `ros2_context` | shared ROS 2 domain/context |
| `ros2_subscribe_twist` | subscribes to `/cmd_vel` (`geometry_msgs/Twist`) |
| `break_3_vector` (x2) | splits linear/angular `Twist` vectors into components |
| `differential_controller` | converts desired linear/angular velocity → left/right wheel velocities |
| `constant_token` + `make_array` | build the `Joint Names` array for the Articulation Controller |
| `articulation_controller` | applies the velocity command to the drive joints |

**Articulation Controller joint targets** (velocity command) — no steering joints are included,
since differential drive steers purely by left/right wheel speed:

```
wheel_front_left_spin
wheel_rear_left_spin
wheel_front_right_spin
wheel_rear_right_spin
```

If the graph is split into two `Articulation Controller` nodes (front / rear, as in the athena_rover
stage), both receive the **same** `[left_velocity, right_velocity]` pair:
- Front controller: `[wheel_front_left_spin, wheel_front_right_spin]`
- Rear controller: `[wheel_rear_left_spin, wheel_rear_right_spin]`

`steer_*` joints are **not** driven in this mode and should stay at their default fixed position.

### Ackermann Drive

Graph: **`Ackermann_Drive`**

| Node | Role |
|---|---|
| `on_playback_tick` | drives graph execution each sim step |
| `ros2_context` | shared ROS 2 domain/context |
| `ros2_subscribe_ackermanndrive` | subscribes to `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`) |
| `ackermann_controller` | converts speed + steering angle → per-wheel steering angle and wheel spin velocity, using chassis params (wheel base, track width, wheel radii, max steering angle/velocity) |
| `constant_token` (x6) + `make_array` (x2) | build the Joint Names / value arrays feeding each Articulation Controller |
| `articulation_controller`, `articulation_controller_01` | one for steering (position command), one for wheel spin (velocity command) |

Key `Ackermann Controller` inputs to set for your chassis: `Front Wheel Radius`, `Back Wheel Radius`,
`Wheel Base`, `Track Width`, `Max Steering Angle Velocity`, `Max Wheel Velocity`, `Invert Steering`.

### Articulation / physics setup notes (apply to both graphs)

These were the main gotchas hit while wiring these graphs up on an imported URDF robot:

- Exactly **one** prim in the robot's hierarchy should carry `PhysicsArticulationRootAPI` — this
  should be the root link (e.g. `base_link`), and no descendant link should also have one. Two roots
  in the same chain causes `Unable to create articulation view ... Physics backend not found` errors
  and the Articulation Controller silently fails to find its target.
- The same root link also needs a **Rigid Body API** applied for the articulation to resolve.
- **Never enable `Kinematic`** on any body that's part of the articulation — kinematic bodies are
  excluded from PhysX articulations entirely and will break the whole chain (parts detach / fly apart
  under simulation instead of moving together).
- The ground plane's collision prim must be **collider-only** (or an explicitly static/kinematic rigid
  body) — if it ends up as a dynamic rigid body it will drift away from the robot under gravity/contact
  forces instead of staying put.
- The stage's root layer needs a valid `startTimeCode`/`endTimeCode` range, or `On Playback Tick`
  reports zero delta time and none of the controllers will actually update, even while the physics
  step keeps running.
- If the Articulation Controller's `Robot Path` / target prim input was set by typing a path string,
  re-drag/re-target it from the Stage tree instead — a stale or mistyped path is a common cause of
  `Pattern '...' did not match any articulations`.

### Testing drive commands from the terminal

Both graphs can be driven directly from a terminal with `ros2 topic pub`, without any other nodes
running — useful for isolating whether an issue is in the Isaac graph or somewhere else in the stack.

**Differential drive — publish a `Twist` on `/cmd_vel`:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

This drives the rover straight ahead at `0.2 m/s` with no rotation. `ros2 topic pub` republishes the
message continuously by default — press `Ctrl+C` to stop. Add `-r 10` to control the publish rate, or
`--once` to send a single message.

**Ackermann drive — publish an `AckermannDriveStamped` on `/ackermann_cmd`:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.0}}"
```

This drives forward at `1.0 m/s` with zero steering angle. Change `steering_angle` (radians) to test
turning, e.g. `0.3` for a gentle left turn.

> Both commands assume the graph's `ros2_context` `Domain Id` matches your terminal's `ROS_DOMAIN_ID`
> (default domain if unset) and that the `topicName` on the subscribe node matches exactly
> (`cmd_vel` / `ackermann_cmd`) — double-check these in the Property panel if the rover doesn't move.

---

## The Livox Mid-360 (approximated)

There is **no official Isaac Sim support** for the Livox Mid-360 (confirmed on the NVIDIA forums —
Livox never provided a model). It's a **non-repetitive, solid-state "rosette" scanner**, which Isaac's
RTX Lidar can only reproduce faithfully via a custom emitter pattern extracted from real recordings.

We use a pragmatic approximation: create an **RTX Lidar → Ouster OS0** (`Create → Isaac → Sensors →
RTX Lidar → Ouster → OS0`), then **override its config attributes** on the inner `OS0/sensor`
(`OmniLidar`) prim to Mid-360 specs — this is NVIDIA's recommended approach (a raw custom JSON
config segfaults). Values applied (via Script Editor, `omni:sensor:Core:*` attributes,
after [rbelshaw's forum config](https://forums.developer.nvidia.com/t/livox-mid360/283074)):

| Attribute | Value | Note |
|---|---|---|
| `numberOfChannels` / `numberOfEmitters` | 40 / 40 | Mid-360-like |
| `emitterState:s001:elevationDeg` | −7.0° … +50.524° (40 values) | **asymmetric upward dome** (OS0 default is symmetric ±45°) |
| `emitterState:s001:azimuthDeg` | `[0.0] * 40` | |
| `emitterState:s001:fireTimeNs` | `[i*1000 for i in range(40)]` | sequential timing → quasi-non-repetitive |
| `nearRangeM` / `farRangeM` | 0.1 / 40.0 | |
| `rangeResolutionM` / `rangeAccuracyM` | 0.004 / 0.025 | |
| `scanRateBaseHz` / `tickRate` | **10 / 10** | see multi-tick note below |
| `reportRateBaseHz` / `patternFiringRateHz` | 7761 (raise for more density) | |

> **All per-emitter arrays** (`elevationDeg`, `azimuthDeg`, `channelId`, `fireTimeNs`,
> `distanceCorrectionM`, `focalDistM`, `focalSlope`, `horOffsetM`, `reportRateDiv`, `vertOffsetM`)
> **must be resized to the same length (40)** together, or the sensor corrupts/crashes.

**Mounting:** the `OS0` prim is a plain Xform parented under `base_link` (no rigid body, no fixed
joint — it follows via the hierarchy), with local Translate ≈ `(0, 0, 0.09)`. TF: `base_link → livox_frame`.
The OS0 asset itself is loaded from NVIDIA's S3 asset server, so Isaac needs internet access on first load.

**Publishing:** the **Lidar** Action Graph (see table above). The `Isaac Create Render Product`
`cameraPrim` must target the inner **`OS0/sensor`** (`OmniLidar`) prim — not the outer `OS0` Xform —
or you get `Render product ... not attached to RTX Lidar (OmniLidar prim is required)` and no topic.

---

## The 4WIS swerve controller

`src/athena_controller/athena_controller/swerve_controller.py` subscribes to `/cmd_vel` (`geometry_msgs/Twist`)
and publishes `/joint_command` (`sensor_msgs/JointState`):

- **Steer joints** (revolute, axis Z): commanded by **target position** (angle).
- **Drive joints** (`wheel_*_spin`, continuous, axis Y): commanded by **target velocity** (spin rate).

Inverse kinematics per wheel at body offset `(x, y)`:

```
wheel_vx = vx - wz * y
wheel_vy = vy + wz * x
steer    = atan2(wheel_vy, wheel_vx)
speed    = hypot(wheel_vx, wheel_vy)
spin     = speed / wheel_radius
```

Wheel positions relative to `base_link` (from the URDF):

| Wheel | x | y |
|---|---|---|
| front_left  | -0.393 |  0.392 |
| front_right | -0.393 | -0.392 |
| rear_left   |  0.345 |  0.379 |
| rear_right  |  0.345 | -0.379 |

> `wheel_radius` in the script is currently a placeholder (`0.10`) — **update it to the real tyre radius**
> or the drive speed will be scaled wrong. `steer_limit` = 90°.

### Isaac joint drive tuning that works
- **Steer joints:** position control — Stiffness ~1000, Damping ~50, **Target Velocity 0**, command via Target Position.
- **Drive joints:** velocity control — **Stiffness 0**, high Damping ~1000, command via Target Velocity.
- Steering `Max Force`/effort was preset to **10 N·m** from the URDF `<limit effort>` (real actuator spec —
  may be too weak to scrub-steer in place; treat as a real design signal).

### Import settings that produce a robot that falls & drives correctly
- **Base Type = Mobile** (not Static/Source, or you get a fixed base).
- **Non-instanceable** (instanceable breaks physics).
- Wheel colliders as **cylinder/convex** (not triangle mesh).
- Don't disable gravity; make sure the chassis collider box doesn't drag on the floor.

---

## Known issues / gotchas

- **Rover renders below the RViz grid:** `odom` origin is fixed at the rover's pose the instant you press
  Play. If the rover then falls to the ground, `base_link.z` goes negative and everything drops below the
  grid. Fix: let the rover settle before pressing Play, or spawn it already resting on the ground.
- **Point cloud shows in `base_link` but "0 points" in `odom`:** timestamp/clock mismatch. Ensure the Time
  graph publishes `/clock`, the Odom graph stamps its transforms with sim time, and every ROS 2 node
  (wrapper + RViz) runs with `use_sim_time:=true`.
- **Isaac publishes nothing after Stop → Play:** seen once — sim time advanced but no Isaac ROS node
  existed (no `/clock`, empty `/livox/points`). Restarting Isaac and pressing Play once fixed it.
- **GPU contention:** Isaac alone keeps the GPU near 99 %. Anything else on the GPU (GLIM's CUDA modules,
  GLIM's viewer) slows the RTX lidar; check `ros2 topic hz /livox/points` (~5–7 Hz is healthy).
- **Bursty lidar → GLIM jump:** the sim lidar sometimes delivers with 1 s+ gaps. If that happens during
  GLIM's startup, its pose can jump metres while parked. Restart `nav_sim.launch.py`.
- **Lidar self-hits:** the lidar sees the rover's own wheel modules; filtered for GLIM by a crop box in
  `nav_sim.launch.py`. Anything else consuming `/livox/points` raw will see them too.
- **Waypoints close to walls fail in `glim` mode:** GLIM's 0.5 m voxels make walls ~0.4 m thicker in
  `/map`; with 1 m inflation and the 1.3 × 1.2 m footprint, goals ~1 m from a wall get "no valid path".
  Keep goals ≥1.5 m from walls for now (tuning open).
- **No reversing:** the planner is forward-only and recoveries never back up, so a rover parked with its
  footprint touching an obstacle (possible in the sim, where obstacles have no colliders) cannot plan
  out ("start occupied"). Drive it clear with `/cmd_vel`.
- **ZED SDK "CORRUPTED SDK INSTALLATION" / "NEURAL TRT NOT FOUND":** missing cuDNN/TensorRT or wrong depth
  mode — reinstall the SDK accepting the AI deps, and use `depth_mode: NEURAL_LIGHT`.
- **Relative references in the scene:** `athena_rover-6.usda` references the ZED 2i model as
  `../../../../../Desktop/zed-isaac-sim/...`, i.e. `~/Desktop/zed-isaac-sim`, and expects this repo at
  `~/Isaac_sim`. Other locations break the reference (the camera goes missing).
- **Cross-version stages (5.0.0 / 6.0.1 / 6.1.0):** a stage saved on a newer Isaac Sim can embed
  OmniGraph node versions or PhysX schema features (e.g. Newton Articulation Root) that don't exist on
  an older install — nodes show as missing/unresolved, or physics fails to initialize. Prefer having
  whoever's on the older version author shared files, or pin the whole team to one version.
- **Duplicate/misplaced Articulation Root:** if more than one prim in a robot's link chain has
  `PhysicsArticulationRootAPI`, or it ends up on the wrong link after an import/undo, the Articulation
  Controller can't find its target (`Physics backend not found` in the console) and the robot may not
  behave correctly under gravity. Only the root link should have it.
- **Enabling Kinematic on an articulated body:** breaks the whole articulation chain immediately (limbs
  detach/fly apart) — kinematic bodies can't be part of a PhysX articulation. Don't use this as a way to
  "lock" a robot in place; use a proper Fixed Joint to the world instead if that's actually the goal.
- **Lidar stripe / sparse cloud (multi-tick without motion BVH):** in Isaac 6.0 the RTX lidar renders
  multi-tick, which needs motion BVH; the lidar has no user toggle for it, so the fix is to set
  `scanRateBaseHz = tickRate = 10` (one rotation per frame → no multi-tick). Symptom before the fix:
  the Console warns `Multi-tick is enabled but motion BVH is not active` and objects show only a single
  vertical column of points instead of a filled surface.
- **Lidar near-field blind spot (expected):** with vertical FOV −7°…+52°, the lowest beam only points 7°
  down, so there's a blind cone below/around the sensor. Close, low objects aren't seen — this is real
  Mid-360 behaviour; use the ZED depth for close-in obstacles. In RViz, raise the PointCloud2
  **Decay Time** (e.g. 2 s) to accumulate the non-repetitive scan into a dense cloud.
- **ZED asset gizmo offset:** the `ZED_2i` asset nests its geometry under internal wrapper prims, so the
  outer prim's gizmo doesn't sit on the camera body — cosmetic. A **fresh drag-in from the Content tab**
  comes in aligned but **as a rigid body**, so it needs either a fixed joint to `base_link` or removal of
  its `RigidBodyAPI` to follow the rover. Reparenting the ZED changes its prim path — **re-point the
  `camera_streamer` graph** to the new path or the stream breaks.

---

## Roadmap / TODO

- [x] Get the **[GLIM](https://github.com/koide3/glim)** 3D LiDAR SLAM stack working on the simulator
      (consumes `/livox/points` + `/livox/imu` for real-time mapping / localization).
- [x] Nav2 waypoint following with obstacle avoidance on ground truth (`localization:=ground_truth`).
- [x] Nav2 on GLIM localization with a map built while driving (`localization:=glim`).
- [x] Spin recovery (no reversing) in the sim behavior trees.
- [ ] Waypoints near walls in `glim` mode: finer GLIM voxels for the sim and/or smaller inflation.
- [ ] Operator GUI (`kratos_gui`) end to end on the sim: LOG WAYPOINT → mission → AUTO.
- [ ] Turn Isaac's `odom → base_link` TF off (rename its topic) and run GLIM exactly as on the rover
      (`isaac_odom_tf:=false`).
- [ ] Port the `Differential_Drive` and `Ackermann_Drive` graphs from their standalone test scenes into
      the main `athena_rover` stage / `demo_1` scene, and reconcile joint names against the actual URDF.
- [ ] Decide which drive mode (swerve, differential, Ackermann) is primary for the real rover and align
      the bringup launch file / controller choice accordingly.
- [ ] Re-verify Articulation Root placement and joint drive settings after any future URDF re-import,
      since import settings have previously reset or duplicated the articulation root.
