# athena_rover — Isaac Sim Digital Twin

A digital twin of **athena_rover**, a 4-wheel independent steering (4WIS / swerve) rover with
rocker-bogie suspension, built in **NVIDIA Isaac Sim 6.0.1** and controlled through **ROS 2 Humble**.

The sim streams a **ZED 2i** stereo camera (RGB + depth + registered point cloud) and publishes
ground-truth **odometry**, so the ROS 2 topic layout matches the real hardware and the same
control/perception stack runs in sim and on the physical rover.

---

## Stack

| Component | Version / Notes |
|---|---|
| Isaac Sim | 6.0.1 (Kit 110.1.2) |
| ROS 2 | Humble |
| Camera | ZED 2i (via Stereolabs ZED SDK + zed-ros2-wrapper) |
| ZED SDK | 5.4.1+ (needs CUDA + cuDNN + TensorRT) |
| Lidar | Livox Mid-360 (RTX Lidar — **planned, not yet added**) |
| OS | Ubuntu 22.04 |

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

1. **Isaac Sim 6.0.1** installed.
2. **ROS 2 Humble** installed and sourced.
3. **ZED SDK 5.4.1+** installed (`https://www.stereolabs.com/developers/release`).
   The installer will prompt to download AI models + optimize them — accept both.
   Requires a working **CUDA toolkit**, **cuDNN**, and **TensorRT** (the SDK installer pulls the
   AI deps; make sure CUDA is on your system first).
4. **zed-isaac-sim** extension (streams the simulated camera into the ZED SDK):
   `https://github.com/stereolabs/zed-isaac-sim` — built locally (extension internal name
   `sl.sensor.camera`; ZED 2i USD lives at `exts/sl.sensor.camera/data/usd/ZED_2i.usdc`).

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

The scene contains three Action Graphs:

| Graph | Nodes | Purpose |
|---|---|---|
| **Clock** | On Playback Tick → ROS2 Publish Clock; Isaac Read Simulation Time → Time Stamp | publishes `/clock` (sim time) |
| **rover_drive** | Subscribe Joint State → Articulation Controller (target `/athena_rover`) | applies `/joint_command` to the joints |
| **Odom** | Compute Odometry → Publish Odometry + Publish Raw Transform Tree; Isaac Read Simulation Time → both timeStamps | publishes `/odom` and the `odom → base_link` TF, stamped with sim time |

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
- **RViz** (with `use_sim_time:=true`)

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
registered point cloud appears.

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

---

## Roadmap / TODO

- [ ] Add **Livox Mid-360** as an RTX Lidar sensor + Livox scan config (not a URDF import).
- [ ] **Joystick teleop** for `/cmd_vel`.
- [ ] `robot_state_publisher` for the full rover TF tree.
- [ ] Replace placeholder `wheel_radius` with the measured value.
- [ ] Steering drive tuning so the 4WIS circle tracks cleanly.
