# FAST-LIVO2（本仓库）

本仓库基于原版 [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)（HKU MARS）修改。在线的激光–惯性–视觉里程计仍沿用原版流程。原版的论文、传感器示例和许可证见上游仓库；下面只写本仓库多出来的部分，以及 HH-LVGO 数据的运行步骤。

## 与原版的差别

### RTK

订阅 `gnss_comm/GnssPVTSolnMsg`（默认话题 `/ublox_driver/receiver_pvt`），用 GeographicLib 转到局部 ENU，并作为位置观测融进在线 LIO。相关配置在 yaml 的 `gps` 段。

### 离线优化

建图跑完后，在 `laserMapping` 所在终端按 Enter，做一次离线优化（`opt/opt_enable: true`，且不能处于定位模式）：

- 用速度序列估计激光里程计和 RTK 的时间偏移，再用样条把轨迹对齐到 RTK 的 ENU。
- 用 GTSAM 做批量位姿优化。
- 用优化后的 IMU 位姿把关键帧拼成下采样全局地图。

产物：

- `Log/pcd/after_optimization_downsampled.pcd`：下采样后的先验地图。
- `Log/pcd/init_map_pose.txt`：地图原点（纬度、经度、高程）和第一帧在该 ENU 下的位姿（位置 + 四元数）。
- `output/TUM/opt_trajectory_after.txt`：优化后的 TUM 轨迹。

`roslaunch` 若拿不到标准输入，关键帧缓冲空闲后会自动开始这次优化。

### 先验地图定位

`localization/localization_en: true` 时不再更新体素地图，而是加载 `prior_map_path` 里的先验点云做点面匹配。

- 先验图应使用上面优化得到的下采样地图的一份拷贝。不要直接把 `Log/pcd/voxel_map.pcd` 当先验图，每次保存都会覆盖它。
- 启动时读取 `Log/pcd/init_map_pose.txt`。IMU 初始化会把姿态重置成单位阵；初始化结束后，再用文件里的第一帧位姿写回状态，并把重力设成 ENU 的 `(0, 0, -g)`，使定位坐标系和优化后的地图一致。
- 需要从同一段数据的开头播放。文件里的位姿对应建图第一帧。

### 建图时的点云保存

彩色点云按帧分块缓存，保存时再合并，避免每帧拷贝整张地图。关键帧下采样使用 `pcl::ApproximateVoxelGrid`，避免大范围点云上 `VoxelGrid` 的整数索引溢出。退出或调用 `/laserMapping/save_map` 时，还可另存体素地图和视觉稀疏地图。

### 额外依赖

在原版依赖之外还需要 GTSAM、GeographicLib，以及本工作空间里的 `gnss_comm`。x86 上编译选项与 GTSAM 对齐（`-march=x86-64`），避免 Eigen 跨库 ABI 不一致。

## 运行

在本工作空间编译并 source：

```bash
cd fastlivo2
catkin_make
source devel/setup.bash
```

配置在 `config/HH-LVGO.yaml`，相机内参在 `config/camera_HH-LVGO-01.yaml`。数据包是压缩图像，`launch/HH.launch` 会把 `/left_camera/image/compressed` 转成原始图像。

### 建图

在 `config/HH-LVGO.yaml` 里设：

```yaml
localization:
  localization_en: false
opt:
  opt_enable: true
```

然后：

```bash
roslaunch fast_livo HH.launch
rosbag play xxx.bag
```

包播完后，在 `laserMapping` 终端按 Enter。确认生成 `Log/pcd/after_optimization_downsampled.pcd` 和 `Log/pcd/init_map_pose.txt`。把 pcd 拷到单独路径再给定位用，例如 `Log/pcd/hh-lvgo-01/after_optimization_downsampled_01.pcd`。

### 定位

把 yaml 改回定位，`prior_map_path` 指向刚才的拷贝：

```yaml
localization:
  localization_en: true
  prior_map_path: "/your/map/path/xxx.pcd"
```

`init_pos` / `init_yaw` 会被 `Log/pcd/init_map_pose.txt` 覆盖。重新编译后从头播放同一段 bag：

```bash
roslaunch fast_livo HH.launch
rosbag play xxx.bag
```

日志中应出现 `Applied init pose after IMU init`，位置和航向与 `init_map_pose.txt` 里的第一帧一致。

AGV 数据把上面的 launch 和 yaml 换成 `launch/AGV.launch`、`config/AGV-LVGO.yaml`，步骤相同。
