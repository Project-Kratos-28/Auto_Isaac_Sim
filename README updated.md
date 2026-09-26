# athena_rover — Isaac Sim Digital Twin

A digital twin of **athena_rover**, a 4-wheel independent steering (4WIS / swerve) rover with
rocker-bogie suspension, built in **NVIDIA Isaac Sim 6.0.1** and controlled through **ROS 2 Humble**.

The sim streams a **ZED 2i** stereo camera (RGB + depth + registered point cloud), a
**Livox Mid-360**-style 3D lidar (approximated with an RTX Lidar), and publishes
ground-truth **odometry**, so the ROS 2 topic layout matches the real hardware and the same
control/perception stack runs in sim and on the physical rover.

In addition to the primary 4WIS swerve controller, the repo now also includes **Differential Drive**
and **Ackermann Drive** OmniGraph controllers, built and validated as standalone drive-mode
prototypes (see [Differential & Ackermann Drive](#differential--ackermann-drive) below). These give
the team simpler skid-steer and car-like steering options to fall back on or compare against the
swerve controller, and reuse the same `/cmd_vel`-style ROS 2 interface pattern.

---

## Stack

| Component | Version / Notes |
|---|---|
| Isaac Sim | 6.0.1 (Kit 110.1.2) — primary; Differential/Ackermann prototypes also validated on 5.0.0 and 6.1.0, see version note below |
| ROS 2 | Humble |
| Camera | ZED 2i (via Stereolabs ZED SDK + zed-ros2-wrapper) |
| ZED SDK | 5.4.1+ (needs CUDA + cuDNN + TensorRT) |
| Lidar | Livox Mid-360 — **approximated** via RTX Lidar (Ouster OS0 base, reconfigured to Mid-360 specs) |
| OS | Ubuntu 22.04 |

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
├── .gitignore
├── src/
│   ├── athena_controller/     # 4WIS swerve controller + bringup launch
│   └── circle_test/           # test node: publishes a circle /cmd_vel
├── description/
│   └── athena_description/     # rover URDF + meshes
└── isaac/
    └── demo_1/                 # saved Isaac Sim scene (.usd) with Action Graphs
```

> **Not included in this repo** (clone/install separately — see Setup):
> the ZED SDK, the `zed-ros2-wrapper` / `zed-ros2-interfaces` packages, and the
> `zed-isaac-sim` Isaac extension. These are third-party and are pulled in during setup.

---

## Prerequisites

1. **Isaac Sim 6.0.1** installed (or 6.1.0 for the ported Differential/Ackermann graphs — see version note above).
2. **ROS 2 Humble** installed and sourced.
3. **ZED SDK 5.4.1+** installed (`https://www.stereolabs.com/developers/release`).
   The installer will prompt to download AI models + optimize them — accept both.
   Requires a working **CUDA toolkit**, **cuDNN**, and **TensorRT** (the SDK installer pulls the
   AI deps; make sure CUDA is on your system first).
4. **zed-isaac-sim** extension (streams the simulated camera into the ZED SDK):
   `https://github.com/stereolabs/zed-isaac-sim` — built locally (extension internal name
   `sl.sensor.camera`; ZED 2i USD lives at `exts/sl.sensor.camera/data/usd/ZED_2i.usdc`).
5. For the Ackermann drive test topic: the **`ackermann_msgs`** ROS 2 package
   (`sudo apt install ros-humble-ackermann-msgs`, or build from source) — it's not a core Humble package.

---

## Setup

### 1. Create the ROS 2 workspace and clone this repo

```bash
mkdir -p ~/athena_ws/src
cd ~/athena_ws/src
git clone <THIS_REPO_URL> Isaac_sim
# make the two ROS packages visible to colcon:
ln -s Isaac_sim/src/athena_controller athena_controller
ln -s Isaac_sim/src/circle_test circle_test
```

*(Or just copy `src/athena_controller` and `src/circle_test` directly into `~/athena_ws/src/`.)*

### 2. Clone the ZED ROS 2 driver (third-party)

```bash
cd ~/athena_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/stereolabs/zed-ros2-interfaces.git
```

### 3. Apply the required ZED wrapper config tweaks

Edit `zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml`:

```yaml
depth:
  depth_mode: 'NEURAL_LIGHT'      # SDK 5.x depth is neural-only; NEURAL_LIGHT avoids the TRT-not-found crash
pos_tracking:
  pos_tracking_enabled: false     # the simulated ZED stream carries no IMU, so on-board pos tracking crashes
```

### 4. Build

```bash
cd ~/athena_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

> If the wrapper build complains `Specify CUDA_TOOLKIT_ROOT_DIR`, install the CUDA toolkit and export
> `CUDA_TOOLKIT_ROOT_DIR` (e.g. `/usr/local/cuda`) before rebuilding.

### 5. Source (add to `~/.bashrc` for convenience)

```bash
source /opt/ros/humble/setup.bash
source ~/athena_ws/install/setup.bash
```

---

## Running

### 1. Isaac Sim
Open the scene `isaac/demo_1/*.usd` and press **Play**.
Let the rover settle on the ground **before** pressing Play (a free-fall at t=0 skews odometry and
puts the rover below the RViz grid — see Known Issues).

The scene contains four Action Graphs:

| Graph | Nodes | Purpose |
|---|---|---|
| **Clock** | On Playback Tick → ROS2 Publish Clock; Isaac Read Simulation Time → Time Stamp | publishes `/clock` (sim time) |
| **rover_drive** | Subscribe Joint State → Articulation Controller (target `/athena_rover`) | applies `/joint_command` to the joints |
| **Odom** | Compute Odometry → Publish Odometry + Publish Raw Transform Tree; Isaac Read Simulation Time → both timeStamps | publishes `/odom` and the `odom → base_link` TF, stamped with sim time |
| **Lidar** | On Playback Tick → Isaac Create Render Product (cameraPrim = `OS0/sensor`) → ROS2 RTX Lidar Helper | publishes the lidar cloud on `/livox/points` (frame `livox_frame`) |

> **Important:** every ROS publish node that emits a timestamp must have `Isaac Read Simulation Time`
> wired into its `timeStamp` input, and every ROS 2 node below must run with `use_sim_time:=true`.
> Otherwise timestamps don't match and RViz drops the point cloud ("0 points" in the `odom` frame).

### 2. Bring up the ROS 2 stack (one command)

```bash
ros2 launch athena_controller bringup.launch.py
```

This starts:
- the **ZED wrapper** (`sim_mode:=true`, `sim_address:=127.0.0.1`, `sim_port:=30000`, `use_sim_time:=true`)
- the **swerve controller** (`/cmd_vel` → `/joint_command`)
- a **static transform** `base_link → zed_camera_link`
- a **static transform** `base_link → livox_frame` (`--z 0.09`, matches the OS0 mount height)
- **RViz** (with `use_sim_time:=true`)

The lidar cloud (`/livox/points`) is published by the Isaac **Lidar** Action Graph directly — no
external driver needed (unlike the ZED, which goes through the wrapper).

Or run the wrapper standalone for debugging:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i sim_mode:=true \
  sim_address:=127.0.0.1 sim_port:=30000 use_sim_time:=true
```

### 3. Drive a test circle

```bash
ros2 run circle_test circle_drive     # publishes vx=0.2, wz=0.5 on /cmd_vel
```

### 4. Verify everything is on the same clock

```bash
ros2 topic echo /clock --once                                                   # small, increasing sim time
ros2 topic echo /tf --once                                                      # odom->base_link, sim-time stamp
ros2 topic echo /zed/zed_node/point_cloud/cloud_registered --field header.stamp --once
```

All stamps should be small matching sim-time values. Then set RViz **Fixed Frame = `odom`** and the
registered point cloud appears. Add a second **PointCloud2** on `/livox/points` to see the lidar.

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
source /opt/ros/humble/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

This drives the rover straight ahead at `0.2 m/s` with no rotation. `ros2 topic pub` republishes the
message continuously by default — press `Ctrl+C` to stop. Add `-r 10` to control the publish rate, or
`--once` to send a single message.

**Ackermann drive — publish an `AckermannDriveStamped` on `/ackermann_cmd`:**

```bash
source /opt/ros/humble/setup.bash
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
- **Point cloud shows in `base_link` but "0 points" in `odom`:** timestamp/clock mismatch. Ensure the Clock
  graph publishes `/clock`, the Odom graph stamps its transforms with sim time, and every ROS 2 node
  (wrapper + RViz) runs with `use_sim_time:=true`.
- **ZED SDK "CORRUPTED SDK INSTALLATION" / "NEURAL TRT NOT FOUND":** missing cuDNN/TensorRT or wrong depth
  mode — reinstall the SDK accepting the AI deps, and use `depth_mode: NEURAL_LIGHT`.
- **Isaac `.usd` scene uses absolute paths** to meshes and the ZED extension. A teammate on a different
  machine will hit broken references unless their paths match — re-point references or save with
  relative/payload references.
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
  down, so there's a blind cone below/around the sensor (~3–4 m radius on the ground). Close, low objects
  aren't seen — this is real Mid-360 behaviour; use the ZED depth for close-in obstacles. In RViz, raise
  the PointCloud2 **Decay Time** (e.g. 2 s) to accumulate the non-repetitive scan into a dense cloud.
- **ZED asset gizmo offset:** the `ZED_2i` asset nests its geometry under internal wrapper prims, so the
  outer prim's gizmo doesn't sit on the camera body — cosmetic. A **fresh drag-in from the Content tab**
  comes in aligned but **as a rigid body**, so it needs either a fixed joint to `base_link` or removal of
  its `RigidBodyAPI` to follow the rover. Reparenting the ZED changes its prim path — **re-point the
  `camera_streamer` graph** to the new path or the stream breaks.

---

## Roadmap / TODO

- [ ] Get the **[GLIM](https://github.com/koide3/glim)** 3D LiDAR SLAM stack working on the simulator
      (consume `/livox/points` for real-time mapping / localization).
- [ ] Port the `Differential_Drive` and `Ackermann_Drive` graphs from their standalone test scenes into
      the main `athena_rover` stage / `demo_1` scene, and reconcile joint names against the actual URDF.
- [ ] Decide which drive mode (swerve, differential, Ackermann) is primary for the real rover and align
      the bringup launch file / controller choice accordingly.
- [ ] Re-verify Articulation Root placement and joint drive settings after any future URDF re-import,
      since import settings have previously reset or duplicated the articulation root.
