# FAST-LIVO2

## FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

### 📢 News

- 🔓 **2025-01-23**: Code released!  
- 🎉 **2024-10-01**: Accepted by **T-RO '24**!  
- 🚀 **2024-07-02**: Conditionally accepted.

### 📬 Contact

For further inquiries or assistance, please contact [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk).

## 1. Introduction

FAST-LIVO2 is an efficient and accurate LiDAR-inertial-visual fusion localization and mapping system, demonstrating significant potential for real-time 3D reconstruction and onboard robotic localization in severely degraded environments.

**Developer**: [Chunran Zheng 郑纯然](https://github.com/xuankuzcr)

<div align="center">
    <img src="pics/Framework.png" width = 100% >
</div>

### 1.1 Related video

Our accompanying video is now available on [**Bilibili**](https://www.bilibili.com/video/BV1Ezxge7EEi) and [**YouTube**](https://youtu.be/6dF2DzgbtlY).

### 1.2 Related paper

[FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035)  

[FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/pdf/2501.13876)  

[FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2203.00893)

[FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://www.arxiv.org/pdf/2507.17210)

### 1.3 Our hard-synchronized equipment

We open-source our handheld device, including CAD files, synchronization scheme, STM32 source code, wiring instructions, and sensor ROS driver. Access these resources at this repository: [**LIV_handhold**](https://github.com/xuankuzcr/LIV_handhold).

### 1.4 Our associate dataset: FAST-LIVO2-Dataset
Our associate dataset [**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z) used for evaluation is also available online.

### 1.5 Our LiDAR-camera calibration method
The [**FAST-Calib**](https://github.com/hku-mars/FAST-Calib) toolkit is recommended. Its output extrinsic parameters can be directly filled into the YAML file. 

## 2. Prerequisited

### 2.1 Ubuntu and ROS

Ubuntu 18.04~20.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 2.2 PCL && Eigen && OpenCV

PCL>=1.8, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV>=4.2, Follow [Opencv Installation](http://opencv.org/).

### 2.3 Sophus

Sophus Installation for the non-templated/double-only version.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 2.4 Vikit

Vikit contains camera models, some math and interpolation functions that we need. Vikit is a catkin project, therefore, download it into your catkin workspace source folder.

```bash
# Different from the one used in fast-livo1
cd catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git 
```

## 3. Build

Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST-LIVO2
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 4. Run our examples

Download FAST-LIVO2-Dataset from [Global-LVBA](https://github.com/xuankuzcr/Global-LVBA) Section IV.

```
roslaunch fast_livo mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```


## 5. Map Saving and Prior-Map Relocalization

This fork extends FAST-LIVO2 with **map saving** and **prior-map based relocalization**, enabling the system to localize against a previously built map without mapping.

### 5.1 Map Saving

While running in mapping mode (`localization_en: false`), the voxel map and visual sparse map can be saved on exit or on demand via the `laserMapping/save_map` ROS service:

```yaml
pcd_save:
  pcd_save_en: true      # save registered LiDAR point clouds
  map_save_en: true      # save voxel map & visual sparse map (Log/pcd/)
  type: 0                # 0: World Frame, 1: Body Frame
  filter_size_pcd: 0.15  # downsample filter size [m]
  interval: -1           # -1: all frames saved into ONE pcd file
```

```bash
# Trigger map saving at any time (also saved automatically on exit):
rosservice call /laserMapping/save_map
# Map saved to Log/pcd/ (voxel_map.pcd, visual_map.pcd, etc.)
```

> **Note**: `Log/pcd/voxel_map.pcd` is **overwritten** by every save. Always copy the saved map to a separate file before using it as a prior map.

### 5.2 Prior-Map Relocalization

FAST-LIVO2 can localize against a previously saved map (no mapping update is performed, i.e. `Update Voxel Map` is disabled), using a prior-map guided initialization plus LIO tracking:

1. Run a **mapping** session (`localization_en: false`) and save the map;
2. Copy the saved map to a **separate** prior map file (e.g. `prior_map.pcd`);
3. Enable localization mode and set `prior_map_path` to that file, then replay data from (approximately) the same start pose:

```yaml
localization:
  localization_en: true
  prior_map_path: "/path/to/prior_map.pcd"  # use a SEPARATE prior map file!
  init_pos: [0.0, 0.0, 0.0]                 # initial position [m] in the prior-map frame
  init_yaw: 0.0                             # initial yaw [rad]
  sigma_num: 10.0                           # matching gate multiplier (wider than mapping's 3)
```

On startup, the prior map is loaded and voxelized (PCA plane fitting), and the initial pose is taken from `init_pos`/`init_yaw`. The system then performs direct LIO against the prior map with a widened matching gate (`sigma_num`) to tolerate initial pose error, without updating the voxel map.

## 6. License

The source code of this package is released under the [**GPLv2**](http://www.gnu.org/licenses/) license. For commercial use, please contact me at <zhengcr@connect.hku.hk> and Prof. Fu Zhang at <fuzhang@hku.hk> to discuss an alternative license.
