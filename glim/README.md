# GLIM SLAM — Livox Mid-360 (real hardware + Isaac Sim digital twin)

3D LiDAR-IMU SLAM ([GLIM](https://github.com/koide3/glim)) configured for the **athena_rover**
digital twin. GLIM installs from apt (nothing to build, no `colcon build`) — this `glim/` folder is
**config only** (`glim_config/`, `glim_ros.rviz`, this README), part of the `Auto_Isaac_Sim` repo. It
consumes two ROS 2 topics and works identically whether they come from a real Mid-360 or the Isaac
Sim digital twin:

| Topic | Type | Notes |
|---|---|---|
| `/livox/points` | `sensor_msgs/PointCloud2` | **not** Livox `CustomMsg` — GLIM cannot read that (silent empty screen) |
| `/livox/imu` | `sensor_msgs/Imu` | accel in m/s² incl. gravity, gyro in rad/s |

| Path | |
|---|---|
| `glim_config/` | 15 JSON configs — keep them together |
| `glim_ros.rviz` | RViz layout (the apt package doesn't ship this) |

```bash
git clone git@github.com:Project-Kratos-28/Auto_Isaac_Sim.git ~/Isaac_sim   # GLIM config lives in glim/
```

> **`auto_fastlio2` is a *different*, separate stack maintained by the team** — not part of this repo.
> You only need it for the **real Mid-360 driver** (§6, `livox_ros_driver2`). The **Isaac Sim +
> GLIM** workflow (§3–§5) needs only this `glim/` config. Get `auto_fastlio2` from the team if you're
> running real hardware.

---

## 1. Install GLIM

### 1a. Add the koide3 PPA

```bash
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2204/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
```

```bash
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list
```

### 1b. Choose CPU or GPU (CUDA)

**CPU build** (simplest, works everywhere):

```bash
sudo apt update && sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev libgtsam-points-dev ros-humble-glim-ros && sudo ldconfig
```

**GPU (CUDA) build** — faster, needs an NVIDIA GPU. **You must pick the CUDA variant that matches
your driver**, not blindly `cuda12.2`. Check what your driver supports:

```bash
nvidia-smi   # read "CUDA Version: XX.X" (the max the driver supports)
nvcc --version
```

Available variants in the PPA: `cuda12.2`, `cuda12.6`, `cuda13.1`.

> ⚠️ **Critical rule:** the build's embedded PTX must be **≤ your driver's CUDA version**, or you
> get `cudaErrorUnsupportedPtxVersion` at the first kernel launch. A `cuda13.1` build will **not**
> run on a CUDA-13.0 driver. Pick the highest variant **≤** your driver's CUDA. Because the
> driver JIT-compiles the PTX, a slightly older build still runs on a newer GPU (e.g. a `cuda12.6`
> build JITs onto a Blackwell RTX 50-series card via a CUDA-13.0 driver).
>
> **This rig:** RTX 5080 Laptop (Blackwell `sm_120`), driver 580 / CUDA 13.0 toolkit → use the
> **`cuda12.6`** build.

The CPU packages and CUDA packages **conflict on files** (they ship the same headers, with no
`Replaces`), so remove the CPU ones first:

```bash
sudo apt remove -y ros-humble-glim ros-humble-glim-ros libgtsam-points-dev
```

Then install the matching CUDA build (substitute your variant for `cuda12.6`):

```bash
sudo apt update && sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev libgtsam-points-cuda12.6-dev ros-humble-glim-ros-cuda12.6 && sudo ldconfig
```

> Don't forget `sudo ldconfig` — the libraries land in `/usr/local/lib`, and without a cache
> refresh you'll get `libgtsam_points_cuda.so.1: cannot open shared object file`.

### 1c. CUDA runtime for the 12.x GPU builds

The `cuda12.x` GLIM builds dynamically link **`libcudart.so.12`**. If your system only has the
CUDA 13 runtime (`libcudart.so.13`), the node fails with
`libcudart.so.12: cannot open shared object file`. Provide a CUDA 12 runtime **without changing
your CUDA toolkit**:

```bash
pip install nvidia-cuda-runtime-cu12
```

Then point the loader at it when launching (see the wrapper in §4). A `cuda13.1` build statically
links cudart and does not need this (but see the PTX rule above).

### 1d. Verify

```bash
ros2 pkg executables glim_ros
ls /opt/ros/humble/lib/libodometry_estimation_gpu.so   # exists only for CUDA builds
```

---

## 2. Configure

All files are in `glim_config/`. The defaults in this repo are already set for the Isaac Sim
digital twin (Mid-360 + ZED 2i IMU). Key knobs:

### `config.json` — module selection
Point these at the `*_gpu.json` variants for the CUDA build, or `*_cpu.json` / `*_passthrough` /
`*_pose_graph` for CPU:

```jsonc
"config_odometry":       "config_odometry_gpu.json",
"config_sub_mapping":    "config_sub_mapping_gpu.json",
"config_global_mapping": "config_global_mapping_gpu.json"
```

### `config_ros.json` — topics & QoS
```jsonc
"imu_topic":    "/livox/imu",
"points_topic": "/livox/points",
"imu_qos":    { "profile": "sensor_data", "depth": 1000 },
"points_qos": { "profile": "sensor_data" }
```
QoS is `sensor_data` (best-effort), which accepts data from a reliable **or** best-effort
publisher — so Isaac's default publisher QoS is compatible.

### `config_sensors.json` — extrinsic & LiDAR fields
```jsonc
"global_shutter_lidar": true,   // sim RTX cloud has no per-point time → disable deskew
"T_lidar_imu": [0.304729, 0.082168, -0.037001, 0.5, -0.5, -0.5, 0.5],
"intensity_field": "",          // sim cloud is XYZ-only; leave blank if no intensity field
"ring_field": ""
```
`T_lidar_imu` transforms a point from the IMU frame into the LiDAR frame
(`p_lidar = T_lidar_imu · p_imu`), TUM format `[x, y, z, qx, qy, qz, qw]`. **This value is
specific to this rig's sensor placement** — recompute it for yours (see §3c).

### `config_preprocess.json` — downsampling
```jsonc
"downsample_resolution": 0.25   // 0.25 for indoor/small scenes; 1.0 is coarse (outdoor Mid-360)
```

---

## 3. Isaac Sim integration

GLIM doesn't care the data is simulated — you just make Isaac publish the two topics. In sim you
**replace the real `livox_ros_driver2` entirely**; the sim *is* the driver. Also publish `/clock`
and run GLIM with `use_sim_time:=true`.

### 3a. LiDAR → `/livox/points`
RTX Lidar (Mid-360 approximation) → **ROS2 RTX Lidar Helper** (`type = point_cloud`), topic
`/livox/points`, with a `frameId`. Publishes a `PointCloud2` (XYZ). See the rover-USD notes for the
Mid-360 sensor-attribute overrides and the `scanRateBaseHz = tickRate` multi-tick fix.

### 3b. IMU → `/livox/imu` (OmniGraph)
There is no one-click IMU helper — build a small Action Graph:

| Node | Setting |
|---|---|
| **On Playback Tick** (or **On Physics Step** for regular high-rate IMU) | — |
| **Isaac Read IMU** | `Imu Prim` = your IMU sensor prim; **Read Gravity = ON** |
| **Isaac Read Simulation Time** | — |
| **ROS2 Publish IMU** | `topicName = /livox/imu`, `frameId = imu_link`, publish accel/gyro/orientation |

Wiring: `Tick → ReadIMU.execIn`; `ReadIMU.execOut → PublishIMU.execIn`; `ReadIMU.{LinAcc, AngVel,
Orientation} → PublishIMU.{linearAcceleration, angularVelocity, orientation}`;
`ReadSimTime.simulationTime → PublishIMU.timestamp`.

> **Read Gravity must be ON** — GLIM initializes gravity direction from the accelerometer. Verify:
> `ros2 topic echo /livox/imu --once` should show `|linear_acceleration| ≈ 9.81` at rest.
> Prefer **On Physics Step** so IMU timing stays regular even when render FPS dips.

### 3c. Compute the extrinsic `T_lidar_imu`
The LiDAR and IMU prims usually live in different USD branches, so you **cannot** subtract their
local transforms. Compute from their world poses — paste into Isaac's **Script Editor** (edit the
two prim paths):

```python
import numpy as np, omni.usd
from pxr import Usd, UsdGeom
from scipy.spatial.transform import Rotation as R

stage = omni.usd.get_context().get_stage()
LIDAR = "/athena_rover/Geometry/base_footprint/base_link/OS0/sensor"
IMU   = "/athena_rover/Geometry/base_footprint/base_link/ZED_2i/base_link/ZED_2i/Imu_Sensor"

def world_T(path):
    m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array([[m[i][j] for j in range(4)] for i in range(4)]).T  # USD row-vector -> column

T_li = np.linalg.inv(world_T(LIDAR)) @ world_T(IMU)
t = T_li[:3, 3]; q = R.from_matrix(T_li[:3, :3]).as_quat()  # (x,y,z,w)
print("T_lidar_imu: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]" % (t[0],t[1],t[2],q[0],q[1],q[2],q[3]))
```

Paste the printed value into `config_sensors.json`.

### 3d. ⚠️ The scene must have geometric structure
LiDAR SLAM localizes by matching geometry. A **flat floor + a couple of objects is not enough** —
the floor only constrains height/roll/pitch, leaving x/y/**yaw** under-constrained, so the map
smears and slowly rotates and loop closure can't fix it. **Test in a scene with walls and scattered
clutter in multiple directions, and drive an exploring path** (not a tight in-place circle). The
RTX lidar raycasts visual meshes, so hand-built `Create → Mesh → Cube` walls/boxes are visible
without colliders.

> Isaac's cloud sample environments (`Simple_Room`, etc.) may fail to load on restricted networks
> (`Failed to open crate file … Sdf_CrateFile MmapAsset`) — build structure by hand instead.

---

## 4. Run

Make sure the sim is **playing** and `/livox/points`, `/livox/imu`, `/clock` are all live.

**Verify the feeds first:**
```bash
ros2 topic hz /livox/points     # ~10 Hz
ros2 topic hz /livox/imu        # 100+ Hz ideally
ros2 topic type /livox/points /livox/imu
```

**GLIM node** (add the cudart path for a `cuda12.x` GPU build — see §1c):
```bash
source /opt/ros/humble/setup.bash
LD_LIBRARY_PATH=$(python3 -c "import nvidia.cuda_runtime, os; print(os.path.join(os.path.dirname(nvidia.cuda_runtime.__file__),'lib'))"):$LD_LIBRARY_PATH \
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=$(realpath ~/Isaac_sim/glim/glim_config) \
  -p use_sim_time:=true
```
(CPU build: drop the `LD_LIBRARY_PATH` prefix.)

**RViz:**
```bash
rviz2 -d ~/Isaac_sim/glim/glim_ros.rviz
```

Hold still ~2 s at startup for IMU init, then drive smoothly. Check it's alive:
```bash
ros2 topic hz /glim_ros/odom     # ~8-10 Hz
```
`/glim_ros/points` = live scan; `/glim_ros/map` = accumulated map (updates every ~10 s).

---

## 5. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `failed to load odometry estimation module … libodometry_estimation_gpu.so: No such file` | config asks for GPU modules but only the CPU build is installed | install the CUDA build (§1b) |
| `libgtsam_points_cuda.so.1: cannot open shared object file` | ldconfig cache stale | `sudo ldconfig` |
| `libcudart.so.12: cannot open shared object file` | `cuda12.x` build, no CUDA 12 runtime present | `pip install nvidia-cuda-runtime-cu12` + set `LD_LIBRARY_PATH` (§1c/§4) |
| `cudaErrorUnsupportedPtxVersion: PTX compiled with an unsupported toolchain` | GPU build newer than driver's CUDA (e.g. `cuda13.1` on a 13.0 driver) | install a build **≤** your driver's CUDA (§1b) |
| dpkg "trying to overwrite … also in package" during install | CPU and CUDA packages conflict on files | `sudo apt remove` the CPU packages first (§1b) |
| Empty viewer, no error | LiDAR publishing Livox `CustomMsg`, not `PointCloud2` | publish `PointCloud2` (RTX Lidar Helper, `type=point_cloud`) |
| GLIM starts but does nothing / IMU init fails | no IMU on `/livox/imu`, or accel ≈ 0 at rest | add the IMU graph (§3b); **Read Gravity ON** |
| Map smears + slowly rotates, loop closure never helps | **feature-poor scene** (x/y/yaw under-constrained) | add walls + clutter, drive an exploring path (§3d) |
| `per-point timestamps are not given … pseudo timestamps by order` | XYZ-only cloud, no per-point time | benign with `global_shutter_lidar: true`; for correct deskew, emit real per-point times from the RTX Lidar Helper |
| Low sim FPS / jittery rates | RTX lidar + GPU GLIM + ZED render load | RTX render preset = **Minimal**; `use_sim_time` keeps SLAM correct in slow-motion; publish IMU on **On Physics Step** |
| `IMU prediction is not good / possibly T_lidar_imu inaccurate` warnings | benign sim artifact (noise-free IMU ties the no-IMU predictor on translation) | ignore if rotation error clearly improves with IMU |

---

## 6. Real Mid-360 hardware

This is the only step that needs the team's **`auto_fastlio2`** stack (it ships `livox_ros_driver2`);
build/source it separately. Launch the driver with **`rviz_`, not `msg_`** (`msg_` publishes Livox
CustomMsg which GLIM cannot read):

```bash
cd ~/auto_fastlio2 && source install/setup.bash   # team stack, not this repo
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
Then run the GLIM node and RViz as in §4 (drop `use_sim_time`). Use the real mechanical
`T_lidar_imu` from the sensor datasheet, `global_shutter_lidar: false`, and `intensity_field:
"intensity"`.

---

## 7. Saving / editing / exporting

**Save:** `Ctrl+C` the GLIM node — it auto-saves to `/tmp/dump` (overwritten next run, deleted on
reboot), so move it:
```bash
mkdir -p ~/maps && mv /tmp/dump ~/maps/$(date +%Y%m%d_%H%M%S)
```
Trajectories inside are TUM format: `odom_*.txt` = before loop closure, `traj_*.txt` = after.

**Edit:** `ros2 run glim_ros offline_viewer` → `File → Open Map`. Loop closure: right-click a submap
→ `Loop begin`, another → `Loop end`, align, `Create Factor`. Point removal:
`ros2 run glim_ros map_editor`.

**Export:** `File → Save → Export Points` writes **PLY**:
```bash
sudo apt install pcl-tools
pcl_ply2pcd map.ply map.pcd
```
