# ros-pyorbslam3

ROS 2 node for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) visual and
visual-inertial SLAM, built on the [pyorbslam3](https://github.com/Vulcan758/pyorbslam3)
Python bindings (`pip install pyorbslam3`). There is no ORB-SLAM3 build step: the
wheel ships it.

- All six sensor modes: mono, stereo, RGB-D, each with or without IMU
- REP-105 outputs: pose of `base_link` in `map`, `map → odom` or `map → base_link` TF
- Odometry, pose, path, tracked map points, tracking status, `/diagnostics`
- Services: reset, reset active map, localization mode, save trajectory
- Dataset player for EuRoC / TUM-VI folders (no bag conversion), with lockstep playback
- Presets for TUM-VI, EuRoC, RealSense D435i and ZED 2i
- ROS 2 **Humble** (Ubuntu 22.04) and **Jazzy** (Ubuntu 24.04)

The full design is in [specs.md](specs.md).

## Install

```bash
# 1. pyorbslam3 (manylinux wheel, includes ORB-SLAM3)
pip install 'pyorbslam3>=0.1.3'                              # Humble
pip install --break-system-packages 'pyorbslam3>=0.1.3'      # Jazzy (externally managed system Python)
sudo apt install libgl1 libegl1 libopengl0 libsm6   # runtime libs, present on any desktop

# 2. this repo
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/Vulcan758/ros-pyorbslam3
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -y --skip-keys python3-pyorbslam3-pip
colcon build
source install/setup.bash
```

pyorbslam3 is not in rosdistro. To let rosdep install it too, register the key
shipped in `rosdep/`:

```bash
echo "yaml file://$HOME/ros_ws/src/ros-pyorbslam3/rosdep/pyorbslam3.yaml" \
  | sudo tee /etc/ros/rosdep/sources.list.d/50-pyorbslam3.list
rosdep update
```

The ORB vocabulary (`ORBvoc.txt`) downloads automatically on first start to
`~/.cache/pyorbslam3`.

A Docker image is also provided:
`docker build -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy -t ros-pyorbslam3:jazzy .`

## Quick start: TUM-VI or EuRoC folder

```bash
ros2 launch pyorbslam3_ros dataset.launch.py dataset:=tumvi \
    dataset_path:=/data/dataset-room1_512_16 sensor:=stereo_imu rviz:=true
```

`sensor` is `mono`, `mono_imu`, `stereo` or `stereo_imu`; `dataset:=euroc` for
EuRoC (`V1_01_easy` etc.).

Playback is **lockstep** by default: each frame is sent once the previous one
has been tracked, so results do not depend on how fast the machine is.
`lockstep:=false rate:=0.5` plays in (scaled) real time instead. ORB-SLAM3 must
then keep up, or frames are dropped. On a laptop CPU ORB-SLAM3 typically tracks
TUM-VI at 5–7 fps, below the 20 fps the camera recorded.

## Cameras

```bash
# RealSense D435i (realsense2_camera installed)
ros2 launch pyorbslam3_ros d435i.launch.py sensor:=stereo_imu start_camera:=true rviz:=true

# ZED 2i through the SDK-less driver from the orbulation package
ros2 launch pyorbslam3_ros zed2i.launch.py sensor:=stereo_imu start_camera:=true
```

D435i modes: `stereo_imu` (recommended: global-shutter IR pair), `stereo`,
`rgbd_imu`, `rgbd`. With `start_camera:=true` the driver is configured with
the settings SLAM needs:
- `unite_imu_method:=2`, without which there is no combined IMU topic.
- The IR emitter is off for the stereo modes, since its dot pattern ruins feature
  matching.
- `align_depth` is on for RGB-D.

ZED 2i bags recorded before the driver applied its −80 ms image-time
correction can be replayed with `use_sim_time:=true image_time_offset:=-0.08`
and `ros2 bag play <bag> --clock`. A 90 s handheld ZED 2i bag replayed in real
time tracked 1305 of 1306 processed frames in `stereo_imu` (IMU initialized
after about 3 s, no resets) and 1607 of 1610 in `stereo`.

Inertial modes need motion to initialize. Rotate and accelerate the camera
for a few seconds after start; poses appear once the IMU is initialized.

## Your own camera

```bash
ros2 launch pyorbslam3_ros slam.launch.py sensor:=stereo_imu \
    settings_file:=/path/to/my_camera.yaml \
    left_image:=/my/left/image_raw right_image:=/my/right/image_raw imu:=/my/imu
```

The settings file is a normal ORB-SLAM3 YAML (see `pyorbslam3_ros/config/orbslam3/`
for examples). The node checks it at startup for the mistakes that otherwise
crash or silently break ORB-SLAM3:
- `Camera.type: "Rectified"` with stereo, which segfaults upstream. Use `PinHole`
  with `Camera2` and `Stereo.T_c1_c2` instead.
- A missing or identity `IMU.T_b_c1`.
- Depth units that don't match `RGBD.DepthMapFactor`.

The node also needs the TF from `base_frame` to the camera's optical frame (the
image `header.frame_id`), so it can report the pose of your robot rather than
the camera.

## Interface

**Subscribed** (remap at launch; only those the `sensor` needs):

| Topic | Type | Used by |
|---|---|---|
| `image` | `sensor_msgs/Image` | mono, mono_imu |
| `left/image`, `right/image` | `sensor_msgs/Image` | stereo, stereo_imu |
| `rgb/image`, `depth/image` | `sensor_msgs/Image` | rgbd, rgbd_imu |
| `imu` | `sensor_msgs/Imu` (rad/s, m/s²) | *_imu |

Supported encodings:
- Camera images: `mono8`, `rgb8`, `bgr8`, `rgba8` and `bgra8`. All are converted
  to grayscale without cv_bridge.
- Depth images: `16UC1` (mm) and `32FC1` (m).

**Published** (node name `orbslam3`):

| Topic | Type |
|---|---|
| `~/odometry` | `nav_msgs/Odometry`: `map` → `base_frame`, finite-difference twist in the body frame |
| `~/pose` | `geometry_msgs/PoseStamped` |
| `~/path` | `nav_msgs/Path` (published poses; not loop-closure corrected) |
| `~/tracked_points` | `sensor_msgs/PointCloud2`: map points seen in the current frame |
| `~/tracking_status` | `pyorbslam3_ros_msgs/TrackingStatus` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| `/tf` | `map → odom` (`tf_mode: map_odom`, default) or `map → base_frame` (`map_base`) |

All outputs are stamped with the image stamp.

**Services**:

| Service | Type |
|---|---|
| `~/reset` | `std_srvs/Trigger` |
| `~/reset_active_map` | `std_srvs/Trigger` |
| `~/set_localization_mode` | `std_srvs/SetBool` |
| `~/save_trajectory` | `pyorbslam3_ros_msgs/SaveTrajectory` (`tum` / `euroc` / `kitti`, optionally keyframes only) |

```bash
ros2 service call /orbslam3/save_trajectory pyorbslam3_ros_msgs/srv/SaveTrajectory \
  "{path: /tmp/traj.txt, format: tum}"
```

**Parameters**: every parameter with its default and a comment is in
[`config/params/slam_node.example.yaml`](pyorbslam3_ros/config/params/slam_node.example.yaml).

### Frames

| Mode | `map` frame |
|---|---|
| Inertial | ORB-SLAM3's world after IMU initialization: gravity-aligned (z up), metric, arbitrary yaw. Poses are held back until then (`publish_before_imu_init`), because ORB-SLAM3 rotates (and for mono, rescales) the map at that point. |
| Non-inertial | The initial pose of `base_frame`, so the robot starts at the origin facing +x. It is only level if the robot started level. |

## Results

On TUM-VI `dataset-room1_512_16` through `dataset.launch.py` (lockstep),
scored against mocap with a sim(3) alignment:

| Mode | Runs | Frames tracked | Final trajectory ATE | Live `~/odometry` ATE (whole run) | Live ATE, settled ² | Live scale | Map z vs gravity |
|---|---|---|---|---|---|---|---|
| `stereo_imu` | 6 | 98.1–98.7 % | 0.74–0.77 cm | 3.1–5.2 cm | 1.01–1.05 cm | 1.001–1.006 | ≤ 0.2° |
| `mono_imu` | 4 | 95.7 % | 0.77–0.79 cm | 1.36–1.89 cm ¹ | 1.15–1.29 cm | 1.002–1.004 | ≤ 0.07° |
| `stereo` | 3 | 100 % | 7.1–7.2 cm | 0.85–1.35 cm | 0.85–1.35 cm | 1.004–1.020 | — |

¹ In 1 of 4 runs, the first IMU initialization's scale was off. Live poses
were up to about 50 cm off for the first 20 s, and 19 cm over the whole run.
They settled to 1.25 cm once ORB-SLAM3's inertial refinements ran.

² Settled means from 20 s after IMU initialization, once ORB-SLAM3 has refined
it. Non-inertial modes have no settling window.

What the ATE columns measure:
- **Final trajectory** is ORB-SLAM3's refined estimate, saved at shutdown. It
  matches running pyorbslam3 directly on the same data: 0.78 cm for
  `stereo_imu`, 1.0–1.2 cm for `mono_imu` and 7.2 cm for `stereo`. Inertial
  modes are scored from IMU initialization on. ORB-SLAM3 saves the frames
  before that with the initial map's scale, which in `mono_imu` is tens of
  centimetres off.
- **Live odometry** is the causal estimate a robot receives as it happens.
  Before later bundle adjustments it is less accurate. Direct pyorbslam3 online
  poses score 3.7 cm on `stereo_imu`.

Fisheye stereo (TUM-VI) needs pyorbslam3 ≥ 0.1.3, which fixes upstream
ORB-SLAM3 out-of-bounds reads. Earlier versions segfault intermittently in
`stereo`/`stereo_imu` on fisheye cameras. The results above were confirmed with
the 0.1.3 wheel from PyPI.

The test is `pyorbslam3_ros/test/test_dataset_acceptance.py`. Run it with
`PYORBSLAM3_ROS_TUMVI=/path/to/dataset-room1_512_16 PYORBSLAM3_ROS_MODES=stereo_imu,mono_imu,stereo`.

## Limitations

- In `mono_imu`, the scale from the first IMU initialization can be off by a
  few percent until ORB-SLAM3 refines it, after about 15 s of varied motion.
  Live poses in that window can be tens of centimetres off. Wait for that
  window to pass, or use `stereo_imu`, when early accuracy matters.
- `~/path` is the history of published poses. It is not corrected after loop
  closures, because the bindings don't expose keyframe poses yet.
- After tracking loss, ORB-SLAM3 starts a new map with a new origin. The path is
  cleared, and poses are not continuous across maps.
- Plain monocular has no metric scale and drifts in scale.
- Bags replayed with `--loop` are not supported. Time going backwards requires
  `~/reset`.
- Python node, so it cannot be loaded as a composable component.

## License

GPL-3.0-or-later, like ORB-SLAM3 and pyorbslam3.
