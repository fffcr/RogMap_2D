# ROG-Map（SUPER 版）接口与参数说明

> 适用版本：`SUPER/rog_map`（接近上游原始 ROG-Map 的版本，与 BJLG 版不同代际）
>
> 源码位置（本文档只涉及本文件夹内）：
> - 参数定义：`include/rog_map/rog_map_core/config.hpp`
> - ROS1 封装：`include/rog_map_ros/rog_map_ros1.hpp` + `src/rog_map_ros/rog_map_ros1.cpp`
> - ROS2 封装：`include/rog_map_ros/rog_map_ros2.hpp`
> - 核心地图类：`prob_map` / `inf_map` / `esdf_map` / `sliding_map` / `counter_map` / `raycaster`

## 架构概述

本版本是**独立节点**（不内嵌到规划器），核心功能是：

> **输入 odom 位姿 + 激光点云 → raycast 更新 3D 概率占据图 → 派生膨胀图 / ESDF → 发布可视化话题**

- 同时提供 ROS1 和 ROS2 两套封装，接口一致，**ROS1 版为主、ROS2 版为简化版**。
- 配置从 **yaml 文件**加载（`yaml_loader::YamlLoader`），不是 ROS2 参数系统；命名空间固定为 `rog_map`。
- 地图坐标系默认 `world`（`visualization/frame_id`）。

---

# 一、输入输出

## 1. 输入（订阅）

| 话题 | 类型 | 含义 |
|------|------|------|
| `odom_topic`（默认 `/lidar_slam/odom`） | `nav_msgs/Odometry` | 机器人里程计位姿，给地图提供"传感器在哪、朝向哪" |
| `cloud_topic`（默认 `/cloud_registered`） | `sensor_msgs/PointCloud2` | 一帧已配准的激光点云，地图的原始数据源 |

> 话题名在 `ros_callback/cloud_topic`、`ros_callback/odom_topic` 里配。

**处理流程**：`cloudCallback` 收到点云 → 里程计超时（`odom_timeout`）则丢弃 → `updateCallback` 调 `updateProbMap()` 做 raycast 更新。

## 2. 输出

### A. 可视化发布（`world` 系，仅当 `visualization/enable=true`）

| 话题 | 类型 | 含义 |
|------|------|------|
| `rog_map/occ` | PointCloud2 | 占据体素点 |
| `rog_map/unk` | PointCloud2 | 未知体素点（需 `pub_unknown_map_en`） |
| `rog_map/inf_occ` | PointCloud2 | 膨胀后的占据点 |
| `rog_map/inf_unk` | PointCloud2 | 膨胀后的未知点（需 `unk_inflation_en`） |
| `rog_map/frontier` | PointCloud2 | 前沿点（需 `frontier_extraction_en`） |
| `rog_map/esdf` | PointCloud2 | ESDF 正距离场点云（需 `esdf_en`） |
| `rog_map/esdf/neg` | PointCloud2 | ESDF 负距离场点云（仅 ROS1 版，需 `esdf_en`） |
| `rog_map/esdf/occ` | PointCloud2 | ESDF 占据点云（仅编译宏 `ESDF_MAP_DEBUG` 时） |
| `rog_map/map_bound` | MarkerArray | 可视化范围 / 局部地图范围 / 更新范围 / ESDF 范围框 |

### B. TF

| 关系 | 含义 |
|------|------|
| `world → drone` | 广播机器人位姿（ROS1 版在 odom 回调里发） |

### C. 内部接口

地图本身（占据图 + 膨胀图 + ESDF）通过 `ROGMap` 基类对外提供查询接口（`boxSearch` 等），供同进程内的规划/探索模块直接调用。

---

# 二、参数说明

> 全部来自 `config.hpp` 的默认值。配置文件命名空间为 `rog_map`。

## 1. ESDF 参数（`esdf/*`）

| 参数 | 默认 | 含义 |
|------|------|------|
| `esdf/enable` | false | 是否构建 ESDF（欧几里得符号距离场） |
| `esdf/resolution` | 0.2 m | ESDF 栅格分辨率 |
| `esdf/local_update_box` | []（必须为 3 个元素） | ESDF 局部更新盒尺寸 [x,y,z] |

## 2. 加载 PCD（顶层）

| 参数 | 默认 | 含义 |
|------|------|------|
| `load_pcd_en` | false | 是否从 PCD 文件加载初始地图 |
| `pcd_name` | map.pcd | PCD 文件路径（支持 `${CMAKE_ROOT_DIR}/` 前缀替换） |

## 3. 地图滑动（`map_sliding/*`）

| 参数 | 默认 | 含义 |
|------|------|------|
| `map_sliding/enable` | true | 局部地图是否随机器人滑动 |
| `map_sliding/threshold` | -1.0 m | 触发滑动的位移阈值，-1 表示用默认策略 |

## 4. 固定地图原点（顶层）

| 参数 | 默认 | 含义 |
|------|------|------|
| `fix_map_origin` | [0,0,0] | 固定地图原点（长度必须为 3） |

## 5. 前沿提取（顶层）

| 参数 | 默认 | 含义 |
|------|------|------|
| `frontier_extraction_en` | false | 是否启用前沿点提取（已知/未知边界） |

## 6. 回调（`ros_callback/*`）

| 参数 | 默认 | 含义 |
|------|------|------|
| `ros_callback/enable` | false | 是否启用 ROS 回调（订阅话题更新地图） |
| `ros_callback/cloud_topic` | /cloud_registered | 输入点云话题 |
| `ros_callback/odom_topic` | /lidar_slam/odom | 输入里程计话题 |
| `ros_callback/odom_timeout` | 0.05 s | 里程计超时：点云与最新里程计时间差超过该值则丢弃该帧 |

## 7. 可视化（`visualization/*`）

| 参数 | 默认 | 含义 |
|------|------|------|
| `visualization/enable` | false | 可视化总开关 |
| `visualization/use_dynamic_reconfigure` | false | 是否用 ROS1 dynamic_reconfigure 动态调参 |
| `visualization/pub_unknown_map_en` | false | 是否发布未知地图 |
| `visualization/frame_id` | world | 地图/可视化坐标系 |
| `visualization/time_rate` | 0.0 Hz | 可视化发布频率（按时间） |
| `visualization/frame_rate` | 0 | 可视化发布频率（按帧） |
| `visualization/range` | [0,0,0] | 可视化范围（以机器人为中心）；任一维 ≤ 0 则自动禁用可视化 |

## 8. 地图分辨率（顶层）

| 参数 | 默认 | 含义 |
|------|------|------|
| `resolution` | 0.1 m | 概率栅格分辨率 |
| `inflation_resolution` | 0.1 m | 膨胀地图分辨率，必须 ≥ resolution |
| `map_size` | [10,10,0] m | 局部地图尺寸（长度必须为 3） |
| `point_filt_num` | 2 | 点云降采样间隔（每 N 点取 1），≤0 强制为 1 |
| `intensity_thresh` | -1 | 点云强度阈值，低于此值滤除（-1=关） |
| `virtual_ground_height` | -0.1 m | 虚拟地面高度（低于此视为地面） |
| `virtual_ceil_height` | -0.1 m | 虚拟天花板高度（高于此视为顶） |

## 9. 膨胀（顶层）

| 参数 | 默认 | 含义 |
|------|------|------|
| `inflation_step` | 1 | 占据膨胀步数（球邻域半径，单位=膨胀栅格） |
| `unk_inflation_en` | false | 是否对未知区域也做膨胀 |
| `unk_inflation_step` | 1 | 未知膨胀步数 |

## 10. Raycast 概率更新（`raycasting/*`）

| 参数 | 默认 | 含义 |
|------|------|------|
| `raycasting/enable` | true | 是否做 raycast 更新占据概率 |
| `raycasting/batch_update_size` | 1 帧 | 每批处理几帧点云 |
| `raycasting/unk_thresh` | 0.70 | UNKNOWN/FREE/OCCUPIED 判定阈值 |
| `raycasting/p_hit` | 0.70 | 命中（打到障碍）时的占据概率更新量 |
| `raycasting/p_miss` | 0.70 | 未命中（穿过）时的占据概率更新量 |
| `raycasting/p_min` | 0.12 | 占据概率下界（clamp） |
| `raycasting/p_max` | 0.97 | 占据概率上界（clamp） |
| `raycasting/p_occ` | 0.80 | 判定为 OCCUPIED 的概率阈值 |
| `raycasting/p_free` | 0.30 | 判定为 FREE 的概率阈值 |
| `raycasting/ray_range` | [0.3, 10] m | 射线有效距离 [min,max] |
| `raycasting/local_update_box` | [999,999,999] m | 每次局部更新盒尺寸 |

> 概率参数成组理解：`p_hit/p_miss` 是每次观测的增量（log-odds 域），`p_min/p_max` 是 clamp 边界，`p_occ/p_free` 是最终二值化阈值。判定规则：**FREE < p_free，OCCUPIED > p_occ，中间为 UNKNOWN**。
>
> 源码中 `p_free` 有重复加载两行（[config.hpp:191-192](include/rog_map/rog_map_core/config.hpp#L191-L192)），属冗余代码，无实际影响。

---

# 三、与 BJLG 版的差异（速查）

本文件夹（SUPER 版）是更早、更接近上游原始 ROG-Map 的版本，与 BJLG 版主要差异：

| 维度 | SUPER 版 | BJLG 版 |
|------|---------|---------|
| 运行方式 | 独立节点 | 内嵌 MincoPlanner 插件 |
| ROS | ROS1 为主 + 简化 ROS2 | 完整 ROS2 lifecycle |
| 配置加载 | yaml 文件（`yaml_loader`） | ROS2 参数系统 |
| 投影分类层 projection | ❌ 无 | ✅ 有 |
| ESDF 场 field | ❌ 无（只有 `esdf_map`） | ✅ 有 |
| 动态衰减 decay | ❌ 无 | ✅ 有 |
| 点云裁剪 cloud_filter | ❌ 无 | ✅ 有 |
| 先验地图 prior_map | ❌ 无 | ✅ 有 |
| 性能监控 performance | ❌ 无 | ✅ 有 |
| 输出话题命名 | `rog_map/occ`、`rog_map/unk`… | `/rog_map/occupied`、`/rog_map/unknown`… |
