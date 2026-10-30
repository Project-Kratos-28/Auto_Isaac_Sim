# Setting up the athena_rover sim stack from scratch

This file is written for a **Claude agent** (or a person) setting up a new computer so it runs
everything the development laptop runs: Isaac Sim with the athena_rover digital twin, the ROS 2
controller/bringup, the ZED camera pipeline, **Nav2** waypoint following, **GLIM** SLAM mapping, and
the team's operator **GUI**. Follow the steps **in order**. Every step ends with a **Check** — do not
continue past a failed check; fix it (see [Troubleshooting](#troubleshooting)) or stop and ask the human.

**Assumed:** Ubuntu **24.04**, a recent NVIDIA GPU with the driver installed (`nvidia-smi` works),
and **Isaac Sim 6.0.1 already installed at `~/isaacsim`**. Internet access is required.

## Rules for the agent

- **`sudo` needs the human's password.** You cannot type it. For every `sudo` command, ask the human
  to run it themselves by typing it in the prompt prefixed with `!` (for example
  `! sudo apt install -y ros-jazzy-desktop`), then continue once they confirm.
- **Steps marked 👤 HUMAN** need a GUI or an interactive installer. Tell the human exactly what to
  do, wait for them to confirm, then run the step's check yourself.
- **Paths matter.** The Isaac scene uses relative references that only resolve with the repo at
  **`~/Isaac_sim`** and the ZED extension at **`~/Desktop/zed-isaac-sim`**. Use exactly the paths
  given here.
- **Never `git push`, never commit** in any repo unless the human asks.
- **Development workflow:** packages are edited and built in **`~/athena_ws`**. The copies in
  `~/Isaac_sim/src` are the published versions; they get copied into `~/athena_ws/src` in step 9.
  Never build inside `~/Isaac_sim` or `~/Auto_Gui` (it leaves `build/install/log` in the repos).
- Long builds: run them in the foreground with a long timeout, or in the background and wait for
  completion. Do not start the next step while a build is running.

## What ends up where

| Path | What | Built with |
|---|---|---|
| `~/isaacsim` | Isaac Sim 6.0.1 (pre-installed) | — |
| `~/Isaac_sim` | this repo (branch `nav2-mapping`) — scene, GLIM config, published packages | not built |
| `~/athena_ws` | **main workspace:** `athena_controller`, `circle_test`, `athena_nav_sim`, ZED ROS 2 wrapper | `colcon`, sourced in `~/.bashrc` |
| `~/auto_fastlio2_jazzy` | team repo `auto_fastlio2`, branch `jazzy-nvblox` (Nav2 config, pcd2pgm, GLIM waypoint manager) | via `~/kratos_gui_ws` |
| `~/Auto_Gui` | team operator GUI repo (+ one local patch) | via `~/kratos_gui_ws` |
| `~/kratos_gui_ws` | workspace of **symlinks** into the two repos above | `colcon` |
| `~/glim_ros_fix_ws` | patched `glim_ros` overlay (fixes phantom walls in GLIM's map) | `colcon` |
| `~/Desktop/zed-isaac-sim` | Stereolabs ZED extension for Isaac Sim | its own `build.sh` |

Source order whenever you run the stack: **`/opt/ros/jazzy` → `~/athena_ws` → `~/kratos_gui_ws` →
`~/glim_ros_fix_ws` (last)**. The first two come from `~/.bashrc`.

---

## Step 0 — Preflight

```bash
lsb_release -ds                                   # Ubuntu 24.04.x
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
cat ~/isaacsim/VERSION                            # starts with 6.0.1
ls ~/isaacsim/isaac-sim.sh ~/isaacsim/setup_ros_env.sh
df -h ~ | tail -1                                 # want 60 GB+ free
ssh -T git@github.com 2>&1 | head -1              # "Hi <user>! You've successfully authenticated"
git ls-remote git@github.com:Project-Kratos-28/Auto_Isaac_Sim.git | grep nav2-mapping
```

**Check:** all commands succeed; the last one prints a `refs/heads/nav2-mapping` line.
- No GitHub SSH access → 👤 HUMAN: add this machine's SSH key (`~/.ssh/id_ed25519.pub`, create it with
  `ssh-keygen -t ed25519` if missing) to their GitHub account, which must have access to the
  `Project-Kratos-28` organisation.
- `nav2-mapping` missing on GitHub → stop: the human has to push that branch from the development laptop first.
- Isaac Sim not at `~/isaacsim` → ask the human where it is and substitute that path everywhere below.

---

## Step 1 — Base tools

```bash
sudo apt update
sudo apt install -y git curl wget build-essential cmake python3-pip software-properties-common \
  gnupg lsb-release rsync
```

**Check:** `git --version && cmake --version | head -1`

---

## Step 2 — ROS 2 Jazzy

```bash
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true          # "already initialized" is fine
rosdep update
```

**Check:**
```bash
source /opt/ros/jazzy/setup.bash && ros2 pkg list | grep -c . && echo $ROS_DISTRO   # a number > 200, then "jazzy"
```

---

## Step 3 — Nav2 and other apt dependencies

```bash
sudo apt install -y \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-ackermann-msgs \
  python3-numpy python3-scipy \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-libav gstreamer1.0-plugins-bad \
  qtbase5-dev
```

- `navigation2` / `nav2-bringup`: Nav2 (controller, planner incl. Smac Lattice, RotationShim + RPP, BT navigator).
- `ackermann-msgs`: only for the Ackermann drive test graph.
- `numpy`/`scipy`: used by `athena_nav_sim` tools.
- GStreamer + Qt5: needed to build the operator GUI.

**Check:**
```bash
source /opt/ros/jazzy/setup.bash
for p in nav2_controller nav2_smac_planner nav2_rotation_shim_controller nav2_regulated_pure_pursuit_controller nav2_bt_navigator rviz2; do ros2 pkg prefix $p >/dev/null && echo "ok $p" || echo "MISSING $p"; done
python3 -c "import scipy, numpy; print('py ok')"
pkg-config --modversion gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0
```

---

## Step 4 — CUDA toolkit (for the ZED SDK and the ZED wrapper build)

The development laptop has **CUDA 13.0** at `/usr/local/cuda-13.0` (driver 580). Install the CUDA
toolkit version that the **ZED SDK 5.4.1** installer for Ubuntu 24.04 asks for (step 5) — check the
download page first and match it.

```bash
nvidia-smi | grep "CUDA Version"                  # the max CUDA the driver supports
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-13-0             # replace 13-0 with the version the ZED SDK needs
```

Add to the end of `~/.bashrc` (use the version you installed):
```bash
export PATH=/usr/local/cuda-13.0/bin:$PATH
export CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-13.0
```

**Check (new terminal):** `nvcc --version | tail -2` shows the installed version; `echo $CUDA_TOOLKIT_ROOT_DIR` is set.

---

## Step 5 — ZED SDK 5.4.1 (👤 HUMAN: interactive installer)

1. 👤 HUMAN: download **ZED SDK 5.4.1 for Ubuntu 24.04** (the build matching the CUDA from step 4) from
   `https://www.stereolabs.com/developers/release` into `~/Downloads`.
2. 👤 HUMAN: run it in a terminal (it asks questions; answer **yes** to installing the AI models,
   optimizing them, and the CUDA/cuDNN/TensorRT dependencies it offers):
   ```bash
   cd ~/Downloads && chmod +x ZED_SDK_*.run && ./ZED_SDK_*.run
   ```

**Check:**
```bash
grep -E '#define ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION' /usr/local/zed/include/sl/Camera.hpp   # 5 / 4 / 1
ls /usr/local/zed/lib | head -3
```

---

## Step 6 — ZED extension for Isaac Sim (`zed-isaac-sim`)

Must live at **`~/Desktop/zed-isaac-sim`** (the scene references `ZED_2i.usdc` inside it by relative path).

```bash
mkdir -p ~/Desktop
git clone https://github.com/stereolabs/zed-isaac-sim.git ~/Desktop/zed-isaac-sim
cd ~/Desktop/zed-isaac-sim
git checkout 0164268          # "5.2.1 (#58)" — the commit the development laptop uses
cat README.md | sed -n 1,80p  # read the build instructions for this version
./build.sh                    # builds the extension (needs the ZED SDK from step 5)
```

**Check:**
```bash
ls ~/Desktop/zed-isaac-sim/exts/sl.sensor.camera/data/usd/ZED_2i.usdc
ls ~/Desktop/zed-isaac-sim/exts/sl.sensor.camera/config/extension.toml
```
If `build.sh` fails, follow the repo README for this commit exactly; do not move the folder.

---

## Step 7 — GLIM (SLAM) from the koide3 PPA

The sim runs GLIM's **CPU** modules (the GPU ones starve Isaac's lidar), so the **CPU packages are
enough**. (The development laptop happens to have the CUDA 12.6 variant installed; it is not needed.)

```bash
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2404/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg > /dev/null
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list
sudo apt update
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev libgtsam-points-dev ros-jazzy-glim-ros
sudo ldconfig
```

**Check:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg executables glim_ros | grep glim_rosnode
ls /opt/ros/jazzy/lib/libodometry_estimation_cpu.so /opt/ros/jazzy/lib/librviz_viewer.so
dpkg -l ros-jazzy-glim-ros | tail -1 | awk '{print $3}'      # 1.2.2-...
```

---

## Step 8 — Clone the repos

```bash
git clone -b nav2-mapping git@github.com:Project-Kratos-28/Auto_Isaac_Sim.git ~/Isaac_sim
git clone -b jazzy-nvblox  git@github.com:Project-Kratos-28/auto_fastlio2.git  ~/auto_fastlio2_jazzy
git clone                  git@github.com:Project-Kratos-28/Auto_Gui.git       ~/Auto_Gui
```

(On the development laptop `~/auto_fastlio2_jazzy` is a `git worktree` of `~/auto_fastlio2`; a
plain clone of the branch is equivalent.)

**Check:**
```bash
git -C ~/Isaac_sim branch --show-current                 # nav2-mapping
ls ~/Isaac_sim/src                                       # athena_controller athena_nav_sim circle_test
ls ~/Isaac_sim/description/athena_description/urdf/athena_rover-6_3/athena_rover-6.usda
ls ~/auto_fastlio2_jazzy/src/kratos_nav/config/nav2_params_sim.yaml
ls ~/auto_fastlio2_jazzy/glim/glim_ext_addon/waypoint_manager ~/auto_fastlio2_jazzy/glim/glim_ros_fix/rviz_viewer_submap_guard.patch
ls ~/Auto_Gui/kratos_gui/src/missionpanel.cpp
```

### 8b — Patch the GUI for sim time

The GUI starts mission scripts without sim time; in the sim they must use `/clock`. Apply this patch
(it is a local change on the development laptop, not in the GUI repo):

```bash
cd ~/Auto_Gui
cat > /tmp/kratos_sim_time.patch <<'EOF'
--- a/kratos_gui/src/missionpanel.cpp
+++ b/kratos_gui/src/missionpanel.cpp
@@ -187,13 +187,17 @@ void MissionPanel::startMission() {
     quoted.append("'" + n.replace("'", "''") + "'");
   }
   const QString param = "waypoints:=[" + quoted.join(",") + "]";
+  QStringList args = QStringList() << script << "--ros-args" << "-p" << param;
+  // Isaac Sim: the mission stamps its Nav2 goals, so it must use /clock too.
+  if (qEnvironmentVariable("KRATOS_USE_SIM_TIME") == "1") {
+    args << "-p" << "use_sim_time:=true";
+  }
 
   log_->clear();
-  appendLog("$ waypoint_mission.py --ros-args -p " + param);
+  appendLog("$ waypoint_mission.py " + args.mid(1).join(" "));
   // Run the script itself, not `ros2 run`: ros2 run doesn't forward SIGTERM,
   // so cancelling would orphan the script with its Nav2 goal still active.
-  process_->start("python3",
-                  QStringList() << script << "--ros-args" << "-p" << param);
+  process_->start("python3", args);
   setState(State::Running, names.join(" → "));
 }
 
EOF
git apply --check /tmp/kratos_sim_time.patch && git apply /tmp/kratos_sim_time.patch
```

**Check:** `grep -c KRATOS_USE_SIM_TIME ~/Auto_Gui/kratos_gui/src/missionpanel.cpp` prints `1`.
If `git apply --check` fails (the GUI repo changed), make the same edit by hand: build an `args`
list, append `-p use_sim_time:=true` when the env var `KRATOS_USE_SIM_TIME` is `1`, and pass `args`
to `process_->start("python3", args)`.

---

## Step 9 — Main workspace `~/athena_ws`

```bash
mkdir -p ~/athena_ws/src
cp -r ~/Isaac_sim/src/athena_controller ~/Isaac_sim/src/circle_test ~/Isaac_sim/src/athena_nav_sim ~/athena_ws/src/

cd ~/athena_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
git -C zed-ros2-wrapper checkout 33dfefa && git -C zed-ros2-wrapper submodule update --init --recursive   # = v5.4.1+3, as on the laptop
git clone https://github.com/stereolabs/zed-ros2-interfaces.git
git -C zed-ros2-interfaces checkout e970087                                                               # "5.3.0"
```

ZED wrapper tweak — the simulated ZED stream has no IMU, so on-board positional tracking must be off.
In `~/athena_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml` set
`pos_tracking_enabled: false` (under `pos_tracking:`), and make sure `depth_mode: 'NEURAL_LIGHT'`:

```bash
F=~/athena_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml
sed -i 's/^\(\s*pos_tracking_enabled:\s*\)true/\1false/' "$F"
grep -nE "pos_tracking_enabled:|depth_mode:" "$F"
```

Build:
```bash
cd ~/athena_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y        # may ask for sudo → human runs it with !
colcon build --symlink-install                            # ZED wrapper takes several minutes
```

Add to the end of `~/.bashrc` (after the CUDA lines from step 4):
```bash
source /opt/ros/jazzy/setup.bash
[ -f ~/athena_ws/install/setup.bash ] && source ~/athena_ws/install/setup.bash
```

**Check (new terminal):**
```bash
grep -n "pos_tracking_enabled: false" ~/athena_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml
for p in athena_controller circle_test athena_nav_sim zed_wrapper zed_components; do printf "%-18s %s\n" $p "$(ros2 pkg prefix $p)"; done   # all under ~/athena_ws/install
ros2 launch athena_nav_sim nav_sim.launch.py --show-args | grep -E "'(localization|rviz|glim_gpu)'"
```

---

## Step 10 — Patched `glim_ros` overlay `~/glim_ros_fix_ws`

apt GLIM 1.2.2's `rviz_viewer` corrupts `/glim_ros/map` (phantom walls → fake obstacles in Nav2).
The team's backported fix is applied to a separate build of `glim_ros`:

```bash
mkdir -p ~/glim_ros_fix_ws/src && cd ~/glim_ros_fix_ws/src
git clone --branch v1.2.2 https://github.com/koide3/glim_ros2.git
cd glim_ros2 && git apply ~/auto_fastlio2_jazzy/glim/glim_ros_fix/rviz_viewer_submap_guard.patch
cd ~/glim_ros_fix_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select glim_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Check:**
```bash
git -C ~/glim_ros_fix_ws/src/glim_ros2 diff --stat          # src/glim_ros/rviz_viewer.cpp | 7 ++++++-
source /opt/ros/jazzy/setup.bash && source ~/glim_ros_fix_ws/install/local_setup.bash
ros2 pkg prefix glim_ros                                    # /home/<user>/glim_ros_fix_ws/install/glim_ros
ldd ~/glim_ros_fix_ws/install/glim_ros/lib/librviz_viewer.so | grep "not found" || echo "links ok"
```

---

## Step 11 — Team packages + GUI workspace `~/kratos_gui_ws`

Symlinks only (the repos stay clean):

```bash
mkdir -p ~/kratos_gui_ws/src && cd ~/kratos_gui_ws/src
ln -s ~/auto_fastlio2_jazzy/glim/glim_ext_addon/waypoint_interfaces .
ln -s ~/auto_fastlio2_jazzy/glim/glim_ext_addon/waypoint_manager .
ln -s ~/auto_fastlio2_jazzy/src/kratos_nav .
ln -s ~/auto_fastlio2_jazzy/src/pcd2pgm .
ln -s ~/Auto_Gui/kratos_msgs .
ln -s ~/Auto_Gui/kratos_gui .
cd ~/kratos_gui_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -Ddraco_DIR=/usr/lib/x86_64-linux-gnu/cmake/draco
```

Do **not** add `athena_nav_sim` here (it lives in `~/athena_ws`). Do not build `kratos_cameras`
(rover-only).

**Check:**
```bash
ls ~/kratos_gui_ws/install   # kratos_gui kratos_msgs kratos_nav pcd2pgm waypoint_interfaces waypoint_manager (+ setup files)
source /opt/ros/jazzy/setup.bash && source ~/athena_ws/install/setup.bash && source ~/kratos_gui_ws/install/setup.bash && source ~/glim_ros_fix_ws/install/local_setup.bash
for p in athena_nav_sim kratos_nav pcd2pgm waypoint_manager kratos_gui glim_ros; do printf "%-18s %s\n" $p "$(ros2 pkg prefix $p)"; done
```
Expected: `athena_nav_sim` → `~/athena_ws`, `kratos_nav`/`pcd2pgm`/`waypoint_manager`/`kratos_gui` →
`~/kratos_gui_ws`, `glim_ros` → `~/glim_ros_fix_ws`.

---

## Step 12 — Isaac Sim configuration (👤 HUMAN, one time)

1. 👤 HUMAN: start Isaac Sim with **`~/isaacsim/isaac-sim.sh`** (not from a terminal that sourced ROS;
   Isaac's `setup_ros_env.sh` selects its bundled Jazzy libraries). First start can take several minutes.
2. 👤 HUMAN: **Window → Extensions** → ☰/⚙ (extension search paths) → **add**
   `/home/<user>/Desktop/zed-isaac-sim/exts`. Search for **`sl.sensor.camera`**, enable it and tick
   **Autoload**. Also make sure **`isaacsim.ros2.bridge`** is enabled with **Autoload**.
3. 👤 HUMAN: **File → Open** `~/Isaac_sim/description/athena_description/urdf/athena_rover-6_3/athena_rover-6.usda`.
   Wait until it has fully loaded (the OS0 lidar model is fetched from NVIDIA's S3 asset server).
4. 👤 HUMAN: press **Play once**. Do not Stop/Play repeatedly — if topics ever stop, restart Isaac.

**Check (agent, in a terminal):**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /clock        # ~30-45 Hz, Ctrl+C after a few lines (or use: timeout 8 ros2 topic hz /clock)
timeout 8 ros2 topic hz /livox/points   # ~5-7 Hz
timeout 8 ros2 topic hz /livox/imu      # ~30-45 Hz
timeout 8 ros2 topic hz /odom
timeout 5 ros2 topic echo /tf --once | grep -E "frame_id|child"   # odom -> base_link
```
- No topics at all → Isaac's ROS graphs aren't running: confirm Play was pressed; restart Isaac and press Play once.
- Isaac console shows missing `ZED_2i.usdc` → step 6 path is wrong.
- "No robot prim found for the articulation controller" errors for `Differential_Drive` /
  `ACKERMAN_DRIVE_GRAPH` also appear on the development laptop and are harmless.

---

## Step 13 — Run and verify: bringup

Terminal A:
```bash
ros2 launch athena_controller bringup.launch.py use_zed:=true
```
Starts the swerve controller, the `base_link → zed_camera_link` and `base_link → livox_frame` mount
TFs, RViz and — because of `use_zed:=true` — the ZED wrapper in sim mode. Without `use_zed:=true` the
ZED is off (its default); Nav2 and GLIM don't need it.

**Check:**
```bash
ros2 node list | grep -E "swerve_controller|base_to_livox|zed_mount_tf|zed_node"   # all four present
timeout 5 ros2 run tf2_ros tf2_echo base_link livox_frame 2>&1 | grep -m1 Translation   # [0.000, 0.000, 0.090]
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"   # rover creeps forward in Isaac
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"                   # stop
```

---

## Step 14 — Run and verify: Nav2 on ground truth (phase 1)

Terminal B:
```bash
source ~/kratos_gui_ws/install/setup.bash
ros2 launch athena_nav_sim nav_sim.launch.py
```
Wait for `Managed nodes are active`.

Terminal C:
```bash
source ~/kratos_gui_ws/install/setup.bash
ros2 run athena_nav_sim send_waypoints --frame odom 1.2,-1.5
```
(Odom frame = Isaac's ground truth, origin at the spawn pose: this goal is 1.2 m ahead and 1.5 m to
the right, clear of the cone ~1.7 m straight ahead and the cube ~2.2 m to the left.)

**Check:** the last line is `SUCCEEDED: rover at (...), <0.5 m from the last waypoint`, and the rover
moved in Isaac.
Then stop Terminal B (Ctrl+C).

---

## Step 15 — Run and verify: GLIM mapping + Nav2 (phase 2)

Terminal B (the GLIM overlay **must be sourced last**):
```bash
source ~/kratos_gui_ws/install/setup.bash
source ~/glim_ros_fix_ws/install/local_setup.bash
ros2 launch athena_nav_sim nav_sim.launch.py localization:=glim glim_viewer:=false rviz:=true
```
Do not move the rover until the log shows `initial IMU state estimation result` (~5 s), then
`GLIM pose received, publishing map -> odom` and `Managed nodes are active`.

**Check:**
```bash
grep -E "librviz_viewer|libglim_ros" /proc/$(pgrep -x glim_rosnode)/maps | awk '{print $6}' | sort -u   # both from ~/glim_ros_fix_ws
timeout 8 ros2 topic hz /glim_ros/odom                         # ~3-8 Hz
timeout 8 ros2 topic hz /livox/points                          # still ~5-7 Hz (GLIM on CPU)
```

Terminal C — a route through open space, in Isaac's ground-truth frame:
```bash
source ~/kratos_gui_ws/install/setup.bash
ros2 run athena_nav_sim send_waypoints --frame odom -2.2,-1.5 0.8,-1.6 -0.8,1.0
```

**Check:** `SUCCEEDED`; Terminal B's log shows `map_seed ... First map from pcd2pgm`; in RViz the
GLIM 3D map and `/map` fill in with the room's walls, the cone and the cube. Waypoints closer than
~1.5 m to a wall can fail with "no valid path" (known limitation, see README).

---

## Step 16 — Operator GUI (optional)

With Isaac, bringup and phase 2 (Terminal B) running:
```bash
source ~/kratos_gui_ws/install/setup.bash
KRATOS_USE_SIM_TIME=1 ros2 run kratos_gui kratos_gui --ros-args -p use_sim_time:=true
```

**Check:** the **Kratos Command Center** window opens on the MAP tab and the waypoint panel shows
`GLIM: ONLINE` within a few seconds. (GUI missions on the sim are not fully tested yet.)

---

## Daily startup (after setup)

1. Isaac: `~/isaacsim/isaac-sim.sh` → open `athena_rover-6.usda` → **Play once**.
2. Terminal A: `ros2 launch athena_controller bringup.launch.py` (add `use_zed:=true` for the ZED camera)
3. Terminal B: `source ~/kratos_gui_ws/install/setup.bash && source ~/glim_ros_fix_ws/install/local_setup.bash`
   then `ros2 launch athena_nav_sim nav_sim.launch.py localization:=glim glim_viewer:=false rviz:=true`
   (or plain `ros2 launch athena_nav_sim nav_sim.launch.py` for ground truth).
4. Terminal C: `send_waypoints ...`, RViz **2D Goal Pose** (launch with `use_bridge:=false`), or the GUI.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Package 'athena_nav_sim' not found` | `~/athena_ws` not sourced (check `~/.bashrc`) or not built |
| `Package 'kratos_nav'`/`'pcd2pgm' not found`, or old Nav2 behaviour | `~/kratos_gui_ws/install/setup.bash` not sourced after `~/athena_ws` |
| `ros2 pkg prefix glim_ros` → `/opt/ros/jazzy` | overlay not sourced last; phantom walls will appear in `/map` |
| No `/clock`, `/livox/points` has no publisher | Isaac not playing, or its ROS graphs died after Stop/Play → restart Isaac, Play once |
| `/livox/points` < 2 Hz | GPU contention: keep `glim_gpu:=false`, `glim_viewer:=false`, close other GPU apps |
| GLIM warns `large time difference between points and imu` and its pose jumps | bursty lidar during GLIM startup → restart `nav_sim.launch.py` with the rover still |
| Nav2: `Start occupied` | rover footprint touches an obstacle (sim obstacles have no colliders). Drive it clear with `ros2 topic pub /cmd_vel ...` |
| Nav2: `no valid path found` near walls | goal within ~1.5 m of a wall in glim mode; pick a goal further out |
| `send_waypoints` goal succeeds instantly | the last waypoint is within 0.5 m of the start; end the route elsewhere |
| ZED wrapper crashes on start | `pos_tracking_enabled` not false, or ZED SDK AI models missing → step 5/9 |
| kratos_gui build: `cannot find -ldraco_shared` | keep the `-Ddraco_DIR=...` flag from step 11 |
| `GLIM: OFFLINE` in the GUI | GLIM not running in `localization:=glim` mode, or different `ROS_DOMAIN_ID` |
