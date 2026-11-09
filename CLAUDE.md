# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FAST-LIVO2 (T-RO '24) is a research implementation of tightly-coupled LiDAR-inertial-visual odometry. It is a **ROS 1 catkin package named `fast_livo`** (the package name predates the repo name — launch/find commands use `fast_livo`, the executable is `fastlivo_mapping`, the ROS node is `laserMapping`). C++17, GPLv2.

## Build & Run

This repo **cannot build standalone** — it must sit in `catkin_ws/src` alongside its dependencies:

- **Sophus at commit `a621ff`** (the old non-templated/double-only API; code includes `<sophus/se3.h>` and uses `SE3` directly). Newer templated Sophus will not compile.
- **`rpg_vikit` from the `xuankuzcr` fork** (not upstream) cloned as a sibling catkin package — provides `vikit_common`/`vikit_ros` (camera models, `vk::camera_loader`).
- PCL >= 1.8, Eigen >= 3.3.4, OpenCV >= 4.2, Boost (thread). OpenMP and mimalloc are auto-detected and optional.
- Livox message headers are **vendored** in `include/livox_ros_driver/` — the actual `livox_ros_driver` package is not needed to build.

```bash
cd ~/catkin_ws
catkin_make                                    # or: catkin_make --only-pkg-with-deps fast_livo
source devel/setup.bash
roslaunch fast_livo mapping_avia.launch        # then: rosbag play YOUR.bag
```

There is no test suite (package.xml declares rostest/rosbag test deps but no tests exist).

CMake gotchas:
- `CMAKE_BUILD_TYPE` is hard-set to `Release` (`-O3 -march=native`; ARM variants auto-detected). Edit CMakeLists.txt for debug builds.
- `ROOT_DIR` macro = package source dir; all runtime output is written under `Log/` relative to it.
- Build is split into libraries `vio`, `lio` (voxel_map), `pre` (preprocess), `imu_proc`, `laser_mapping` (LIVMapper) linked into `fastlivo_mapping`.

Trajectory evaluation: set `evo/pose_output_en: true` to write a TUM-format trajectory to `Log/result/<seq_name>.txt`, then evaluate with evo (`evo_ape tum est.txt gt.txt -a`). `Log/result/ntu_viral/README.md` documents the NTU-VIRAL evaluation workflow.

## Architecture

Single node; pipeline per cycle in `LIVMapper::run()`: `sync_packages()` → `processImu()` → `stateEstimationAndMapping()` (dispatches to `handleLIO()` or `handleVIO()`).

### Shared filter state
Everything revolves around one **error-state iterated Kalman filter (ESIKF)** with a 19-dim state (`StatesGroup` in `include/common_lib.h`): rotation(3), position(3), **inverse exposure time(1)**, velocity(3), gyro bias(3), accel bias(3), gravity(3). `LIVMapper` owns `_state` and hands pointers to both the LIO and VIO managers, which update the same state in turn — this shared filter is the core coupling mechanism.

### Sensor scheduling (`sync_packages`, LIVMapper.cpp)
Callbacks push IMU/LiDAR/image messages into mutex-protected deques. The operating mode is derived from config (`img_en && lidar_en` → LIVO; `imu_en` → ONLY_LIO; else ONLY_LO). In **LIVO mode**, the continuous LiDAR stream is *split at each image capture time* (`pcl_proc_cur`/`pcl_proc_next`, per-point time offsets in `curvature`): every camera frame triggers an **LIO update to the image timestamp, immediately followed by a VIO update** at the same instant (the `lio_vio_flg` state machine WAIT→LIO→VIO→LIO…). In ONLY_LIO/ONLY_LO modes there is one update per full LiDAR sweep.

### Preprocess (`src/preprocess.cpp`)
Per-sensor adapters (Livox Avia `CustomMsg`; Velodyne, Ouster, Hesai XT32, Pandar128, Robosense, L515 as `PointCloud2`) normalize everything to `PointCloudXYZI` (= `pcl::PointXYZINormal`). **Key invariant: each point's `curvature` field stores its time offset in milliseconds from the scan header time** — synchronization, undistortion and scan-splitting all depend on this. Adding a LiDAR = new `LID_TYPE` enum + registered point struct + handler.

### IMU (`src/IMU_Processing.cpp`)
`ImuProcess::Process2`: IMU init (static averaging over the first ~20 frames → gravity direction, `IMU_mean_acc_norm` used to rescale accelerations to `G_m_s2`) → forward propagation of the ESIKF prior → backward motion-compensation (undistortion) of the scan. With `imu_en: false`, `Forward_without_imu` does constant-velocity propagation + covariance growth only. Gravity/bias/exposure estimation can each be disabled by config flags.

### LIO (`src/voxel_map.cpp`, `include/voxel_map.h`)
`VoxelMapManager` maintains a probabilistic voxel octree: `unordered_map<VOXEL_LOCATION, VoxelOctoTree*>` keyed by a custom spatial hash; each voxel fits a plane with 6×6 uncertainty (`VoxelPlane::plane_var_`). `StateEstimation()` runs the iterated ESIKF on point-to-plane residuals, with per-point measurement covariance from beam/range error (`calcBodyCov`) and OpenMP-parallel residual building (`BuildResidualListOMP`). After the update, points + propagated covariances are inserted via `UpdateVoxelMap`; `mapSliding()` optionally frees distant voxels (`local_map/map_sliding_en`).

### VIO (`src/vio.cpp`, `include/vio.h`)
`VIOManager` is **direct and sparse** — no feature extraction, no optical flow. `processFrame()` per image:
1. `retrieveFromVisualSparseMap` — project LiDAR-derived world points (`pointWithVar` list passed from the LIO step, carrying position + covariance + normals) into the image via a grid; keep visible, front-facing, non-outlier points; warp their reference patches (`SubSparseMap`) from the reference frame using affine/homography warp matrices over the image pyramid.
2. `computeJacobianAndUpdateEKF` → `updateState` (or `updateStateInverse` when `exposure_estimate_en`, which jointly estimates inverse exposure time) — coarse-to-fine photometric-error minimization iterating the same 19-dim ESIKF; robust-kernel outlier rejection.
3. `generateVisualMapPoints` / `updateVisualMapPoints` / `updateReferencePatch` — seed new visual map points from LiDAR points (RGB sampled from the voxel map via `RGBFromVoxel`), maintain reference patches across frames.

The visual map (`feat_map`: `unordered_map<VOXEL_LOCATION, VOXEL_POINTS*>` of `VisualPoint`s, each owning `Feature` reference patches) lives in the **same voxel grid as the LiDAR map**. `Frame`/`Feature`/`VisualPoint` (src/frame.cpp, src/visual_point.cpp) are SVO-style structures; the camera model comes from vikit, loaded via `vk::camera_loader::loadFromRosNs("laserMapping", ...)` — so the node name in launch files must stay `laserMapping`.

### Extrinsics convention (`extrin_calib` in config YAML)
- `extrinsic_T` / `extrinsic_R`: **IMU ← LiDAR** (R is 9 values, row-major).
- `Pcl` / `Rcl`: **camera ← LiDAR**.

### UAV support
With `uav/imu_rate_odom: true`, a 250 Hz ROS timer (`imu_prop_callback`) publishes IMU-rate propagated odometry on `/LIVO2/imu_propagate` (and a mavros vision-pose topic) between filter updates.

## Configuration & Output

Each launch file pairs a sensor YAML (`config/avia.yaml`, `HILTI22.yaml`, `NTU_VIRAL.yaml`, `MARS_LVIG.yaml`) with a camera YAML loaded into the node's private namespace (`camera_pinhole.yaml`, `camera_fisheye_HILTI22.yaml`, …). In camera YAMLs, `cam_model` is `Pinhole` or `EquidistantCamera` and `scale` is the image resize factor applied in `processFrame`.

Sensor YAML groups map 1:1 to `LIVMapper::readParameters`: `common` (topics, `img_en`/`lidar_en`), `time_offset` (per-sensor time offsets + `exposure_time_init` — **time sync is the usual failure point when adapting to a new dataset**), `preprocess` (`lidar_type`, `blind`, `filter_size_surf`, `point_filter_num`; `hilti_en` decimates 40 Hz images to 10 Hz), `vio` (patch size, pyramid levels, `outlier_threshold`, `img_point_cov`), `imu` (noise covariances, `imu_int_frame`), `lio` (`voxel_size`, `max_layer`, `dept_err`/`beam_err`, `min_eigen_value`), plus `local_map`, `uav`, `publish`, `evo`, `pcd_save`, `image_save`.

Runtime outputs (all under `Log/`, gitignored except tracked guides/reference data):
- `mat_pre.txt` / `mat_out.txt` — per-frame state before/after each update; plot with `Log/plot.py`.
- `result/<seq>.txt` — TUM trajectory (`pose_output_en`).
- `pcd/` — saved cloud(s) + `lidar_poses.txt` (`pcd_save_en`; `interval: -1` accumulates everything into one file).
- `image/` — saved images + poses (`img_save_en`).
- `Colmap/sparse/0/` — COLMAP text model (`colmap_output_en`; on startup this **executes `scripts/colmap_output.sh`, which `rm -rf`s and recreates those directories**).

Key published topics: `/aft_mapped_to_init` (odometry), `/path`, `/cloud_registered` (RGB-colored when the camera is enabled), `/rgb_img` (tracked-points overlay), `/planes` (voxel map markers).
