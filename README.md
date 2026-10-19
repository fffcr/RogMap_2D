# RogMap_2D

面向地面机器人的 ROG-Map 局部地图包。

本仓库由 HKU-MaRS 的 [SUPER](https://github.com/hku-mars/SUPER) 中的 ROS 2 版 `rog_map` 修改而来；
底层 3D 滑窗概率地图、射线投射与 ESDF 算法来自 [ROG-Map](https://github.com/hku-mars/ROG-Map)。
当前构建只启用 ROS 2（`CMakeLists.txt` 定义 `USE_ROS2`、`ORIGIN_AT_CORNER`），
`ros/` 下的 ROS 1 构建文件与 `include/rog_map_ros/rog_map_ros1.hpp` 不参与当前编译。

## 相对上游的改动

1. **按机器人 odom 高度切片**
   `projection.z_min` / `z_max` 定义在 odom 系（相对机器人），实际采样区间为
   `cur_odom.z + [z_min, z_max]`，再与 `esdf.local_update_box` 求交。
   切片带随机器人上下移动，只做 z 平移，不做旋转或 TF 变换。
   实现：`ProbMap::projectionZRange()`、`queryCell2D()`。

2. **ESDF / gridmap 的 2D 投影**
   `ESDFMap` 的 3D 带符号距离沿 z 取 min，生成 `Field2D` 2D 场。
   `projection.output_esdf: false` 发布 `nav_msgs/OccupancyGrid`（`rog_map/proj2d`）；
   `true` 发布 `sensor_msgs/PointCloud2` 2D 距离场（`rog_map/proj2d_sdf`，`intensity` = 有符号距离）。
   实现：`include/rog_map/projection2d.h`、`src/projection2d.cpp`、
   `ProbMap::buildField2D()`、`ROGMapROS::publishProj2D()`。

3. **gridmap 时间衰减**
   按体素最后一次命中时间 `last_hit_time_` 衰减障碍：
   `keep_time` 内保持不变；`keep_time ~ clear_time` 之间线性压向 free；
   超过 `clear_time` 直接压回 free。可选 active list 只遍历被命中过的体素。
   实现：`ProbMap::applyDecay()`。默认 `decay.enable: false`。

## 依赖

- ROS 2（`rclcpp`、`nav_msgs`、`sensor_msgs`、`visualization_msgs`、`tf2_ros`、`message_filters`、`pcl_ros`、`pcl_conversions`）
- PCL、Eigen3、yaml-cpp
- `libdw`（`Apps/rog_map_node.cpp` 中 `backward.hpp` 使用）

## 构建

```bash
cd ~/ros2_ws/src
git clone <this-repo> rog_map
cd ~/ros2_ws
colcon build --packages-select rog_map
source install/setup.bash
```

## 运行（launch 版本）

```bash
ros2 launch rog_map rog_map.launch.py
```

可选参数：

```bash
# 指定 config/ 下的 yaml
ros2 launch rog_map rog_map.launch.py config_name:=rog_map.yaml

# 仿真时钟
ros2 launch rog_map rog_map.launch.py use_sim_time:=true
```

说明：

- `config_name` 只是文件名，入口按 `<包源码根目录>/config/<config_name>` 读取，
  因此改 yaml 不需要重新 install。
- 输入点云必须已经变换到 odom / 世界系；本包不做 TF 查找。
- `visualization.frame_id` 只是给消息贴标签，需与上游实际 frame 一致（默认 `camera_init`）。
- 节点内部使用多个 `MutuallyExclusive` callback group，入口已使用 `MultiThreadedExecutor`。
- `visualization.enable` 控制发布者是否创建；`projection.enable` 只控制是否计算。
- `pub_odom_tf` 默认关闭，避免与 FAST-LIO 等上游的 TF 广播冲突。

## 接口

### 订阅

| Topic | 类型 | QoS |
|---|---|---|
| `ros_callback.cloud_topic`（默认 `/cloud_registered`） | `sensor_msgs/msg/PointCloud2` | best_effort, keep_last 1 |
| `ros_callback.odom_topic`（默认 `/Odometry`） | `nav_msgs/msg/Odometry` | best_effort, keep_last 1 |

### 发布

发布 QoS 为 reliable, keep_last 1, volatile。

| Topic | 类型 | 条件 / 含义 |
|---|---|---|
| `rog_map/occ` | `PointCloud2` | 3D 占据体素 |
| `rog_map/unk` | `PointCloud2` | `visualization.pub_unknown_map_en` |
| `rog_map/inf_occ` | `PointCloud2` | 膨胀后的占据体素 |
| `rog_map/inf_unk` | `PointCloud2` | `pub_unknown_map_en && unk_inflation_en` |
| `rog_map/esdf` | `PointCloud2` | `esdf.enable`，每生成一帧地图发一次 |
| `rog_map/proj2d` | `nav_msgs/OccupancyGrid` | `projection.enable && !projection.output_esdf` |
| `rog_map/proj2d_sdf` | `PointCloud2` | `projection.enable && projection.output_esdf` |
| `rog_map/frontier` | `PointCloud2` | `frontier_extraction_en` |
| `rog_map/map_bound` | `MarkerArray` | 局部地图 / 更新范围 / ESDF 范围可视化 |

`rog_map/proj2d` 取值：`-1` 未知，`0` 空闲，`99` 内膨胀带，`0~99` 线性外膨胀带，`100` 占据。

### 主要参数

```yaml
resolution: 0.1
map_size: [50, 50, 6]
map_sliding:
  enable: true
  threshold: 0.3

raycasting:
  ray_range: [0.03, 10]
  local_update_box: [10, 10, 2.5]
  p_hit: 0.9
  p_miss: 0.45

esdf:
  enable: true
  resolution: 0.1
  local_update_box: [12, 12, 1.6]   # 同时限制 2D 投影的 z 范围

projection:
  enable: true
  z_min: -0.5        # odom 系，相对机器人
  z_max: 0.2
  max_distance: 6.0
  min_distance: -3.0
  output_esdf: false # false -> proj2d; true -> proj2d_sdf

decay:
  enable: false
  keep_time: 0.5
  clear_time: 1.2
  active_list_enable: false

visualization:
  enable: true
  frame_id: "camera_init"
  time_rate: 2
  range: [15, 15, 4]
```

### C++ 接口

```cpp
#include <rog_map_ros/rog_map_ros2.hpp>

auto map = std::make_shared<rog_map::ROGMapROS>(node, cfg_path);
```

查询接口继承自 `ROGMap` / `ProbMap`：

```cpp
map->isLineFree(start, end);                 // 射线可通过性
map->getNearestCellIs(type, pos, out, max_d);// 最近指定类型栅格
map->getGridType(pos);                       // KNOWN_FREE / OCCUPIED / UNKNOWN / OUT_OF_MAP
map->getField2D();                           // 2D 投影场
```

`ROGMap::updateMap(cloud, pose)` 仅用于 `ros_callback.enable: false` 的手动更新模式。

## 目录

```text
Apps/                  # ROS 2 入口 rog_map_node
include/rog_map/       # 地图核心、ESDF、2D 投影、射线投射
include/rog_map_ros/   # ROGMapROS 封装（当前生效 ros2.hpp）
src/                   # 地图实现
launch/                # rog_map.launch.py
config/rog_map.yaml    # 参数与开关
docs/                  # 设计文档与改造记录
```

## License

核心代码沿用上游 ROG-Map 的 LGPL-3.0（或更高版本）许可，使用与再分发请遵循上游许可。
