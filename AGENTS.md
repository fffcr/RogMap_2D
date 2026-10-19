# AGENTS.md

# 项目 Agent 行为准则

本文件是本仓库中 AI Agent、Codex、Claude、superpower skill、多 agent 工作流的统一行为规范。  
所有代码修改、审计、重构、调参、文档化任务都必须优先阅读并遵守本文件。

本仓库是 RoboMaster 导航链路中的 ROG-Map 局部地图包，包含概率占据地图、滑窗局部地图、膨胀地图、ESDF、2D 投影、射线投射、ROS 2 封装、launch、参数文件等模块。仓库中存在已经过实车验证的建图与发布逻辑，任何修改都必须以“保持可复现、最小改动、可审计”为第一原则。

---

## 0. 最高优先级限制：编译构建限制

**禁止在未经用户明确允许的情况下执行任何编译或构建操作**，包括但不限于：

- `colcon build`
- `catkin build` / `catkin_make`
- `cmake` 构建
- `make` / `ninja`
- `cargo build` / `cargo check`
- `npm run build` / `yarn build`
- 任何其他会触发代码编译、链接的命令

在执行任何编译构建命令之前，必须先向用户确认并获得明确许可。

允许在未确认前执行的低风险检查包括：

- `grep` / `rg`
- `find`
- `ls`
- `sed` / `cat`
- `python3` 静态解析脚本
- XML 语法解析（`package.xml`）
- YAML 语法解析
- Python launch 文件语法检查（`ast.parse`，不生成字节码）
- git diff / git status
- 不触发编译、链接、生成代码的静态检查

如果不确定某条命令是否会触发构建，必须先询问用户。

## 0.1 最高优先级限制：禁止版本控制提交操作

**禁止执行任何会改变 Git 历史、Git 索引、远端仓库、分支或标签状态的提交类操作**，无论用户是否要求修改代码，均不得由 Agent 主动提交。

禁止命令包括但不限于：

- `git add`
- `git commit`
- `git commit --amend`
- `git push`
- `git tag`
- `git merge`
- `git rebase`
- `git cherry-pick`
- `git revert`
- `git reset --hard`
- `git checkout -- <file>` / `git restore <file>` 等会覆盖工作区修改的命令
- `gh pr merge`、`gh release create` 等任何会产生提交、合并、发布或远端变更的命令

允许的只读 Git 检查包括：

- `git status`
- `git diff`
- `git log`
- `git show`

Agent 只负责修改工作区文件和说明变更，不负责暂存、提交、推送、打标签或合并。  
即使任务已经完成，也只能提醒用户自行检查和提交，不能代替用户执行任何提交操作。

## 0.2 最高优先级限制：禁止 writingplans / 计划说明工具

**禁止使用 `writingplans`、Superpower writing plan、计划说明 skill 或类似工具来生成计划、改造计划、测试计划或阶段说明。**

执行方式要求：

1. 用户与 Agent 已经商讨清楚需求和边界后，直接修改代码或文件。
2. 不要在正式修改前额外输出长篇“计划说明”“改造计划说明”“测试计划说明”。
3. 不要新建 `writingplans` 相关文件。
4. 不要把计划说明作为修改前的阻塞步骤。
5. 必要的任务边界、风险、检查项可以写入最终改造记录或最终回复，但不能调用 writingplans 工作流。
6. 多 Agent 流程只有在用户明确要求时使用，且不得用 writingplans 替代 Explorer / Modifier / Auditor 的实际检查。

允许保留简短的执行摘要和最终检查结果，但不得以 writingplans 形式展开。  
核心原则是：**商讨完毕后直接按约束修改代码，修改完成后再给出摘要、检查结果和剩余风险。**

---

## 1. 项目背景

本仓库包含但不限于以下模块：

- 地图核心：`SlidingMap`（滑窗局部地图与索引系统）、`ProbMap`（log-odds 概率占据）、`CounterMap`、`InfMap`（膨胀层）、`FreeCntMap`（frontier 计数层）。
- 距离场：`ESDFMap`（3D EDT 距离场）、`Projection2D` / `Field2D`（min-over-z 的 2D 投影场）。
- 射线与感知输入：`RayCaster`、点云 / odom 回调、虚拟地面与天花板裁剪、障碍 decay。
- ROS 2 封装：`ROGMapROS`（`include/rog_map_ros/rog_map_ros2.hpp`）、入口 `Apps/rog_map_node.cpp`。
- 构建与通信：`CMakeLists.txt`、`package.xml`、`launch/rog_map.launch.py`、`config/rog_map.yaml`。
- 第三方与工具：`include/fmt`、`include/super_utils`。
- 文档与实验材料：`docs/`、`docs/ai_refactor_records/`。

当前 `CMakeLists.txt` 定义 `USE_ROS2` 与 `ORIGIN_AT_CORNER`；`ros/` 下的 ROS 1 构建文件与 `include/rog_map_ros/rog_map_ros1.hpp` 不参与当前构建。  
这是比赛导航链路中的局部地图包，不是单纯实验仓库。  
不要为了“理论更优”而随意改变已经验证过的建图、投影与发布逻辑。

---

## 2. 总原则

所有 agent 必须遵守：

1. 不破坏已经验证过的主逻辑。
2. 不进行无关重构。
3. 不顺手删除历史逻辑、注释分支或旧方案，除非用户明确要求。
4. 不引入大量新变量、新函数、新模块。
5. 不改变已有 topic、frame、参数名、消息语义，除非用户明确要求。
6. 不改变模块边界，除非这是本次任务目标。
7. 不添加高频 debug 日志。
8. 不添加无用统计。
9. 不擅自修改 launch 组合方式、QoS、timer、callback group。
10. 不擅自修改已验证的建图参数默认值。
11. 不把未验证推断写成确定事实。
12. 每次非平凡修改必须留下改造记录。

如果用户没有明确要求重构，应优先选择：

```text
最小修改 > 局部修补 > 结构整理 > 大规模重构
```

---

## 3. 用户确认时使用多 Agent 工作流

每次非平凡代码修改，如果用户明确说明使用三阶段流程：

```text
Explorer Agent → Modifier Agent → Auditor Agent
```

多 agent 的目标不是让多个 agent 同时乱改，而是通过角色隔离降低误判风险。  
多 agent 记录不得使用 writingplans 生成，也不得以计划说明替代真实仓库检查。

### 3.1 Explorer Agent：探索与事实确认

Explorer Agent 只读仓库，不修改代码。

职责：

1. 找到与用户需求相关的源码、头文件、launch、yaml、CMake、参数文件。
2. 梳理调用链、数据流、topic、参数来源。
3. 区分“当前生效逻辑”和“注释/废弃/历史逻辑”。
4. 标记已验证的建图/发布逻辑、隐式依赖、潜在冲突。
5. 给 Modifier Agent 明确修改边界。
6. 把探索结果写入改造记录。

必须检查：

- launch 中实际启动的节点、参数与 config 路径。
- cpp/hpp 中对应函数是否注册、是否被调用、是否被 `USE_ROS1` / `USE_ROS2` 宏排除。
- 关键状态（`robot_state_`、`occupancy_buffer_`、`distance_buffer`、`field_`、`map_frame_count_`）的读写者。
- ROS topic 的发布者、订阅者、QoS、频率限制。
- 参数是否来自 yaml、`Config` loader 默认值、launch override 或硬编码。
- 是否有 static、全局变量、类成员状态影响多实例或跨线程行为。
- 是否存在重复分支、历史注释、未启用配置项误导。
- 是否有用户明确要求不能动的模块。

Explorer Agent 禁止修改文件。

### 3.2 Modifier Agent：最小修改

Modifier Agent 只能根据 Explorer Agent 的记录和用户目标修改代码。

职责：

1. 做最小范围修改。
2. 保留原有行为优先级。
3. 不做用户未要求的清理、重命名、重构。
4. 修改后更新必要注释、参数说明、launch 参数或记录文件。
5. 如果发现 Explorer 结论不完整，先补充记录，不要直接扩大修改范围。

禁止：

- 顺手改其他模块。
- 顺手加调试统计。
- 顺手改参数默认值。
- 顺手删注释分支。
- 顺手重构类结构。
- 顺手改 topic 名、frame_id、QoS、消息语义、参数名。
- 顺手把逻辑改成“理论上更优”的版本。

### 3.3 Auditor Agent：审计与验收

Auditor Agent 不相信 Modifier 的自述，必须重新检查。

职责：

1. 查看 diff。
2. 重新 grep 关键变量、节点、topic、参数。
3. 检查是否越界修改。
4. 检查是否破坏原有优先级和数据流。
5. 运行用户允许范围内的静态检查。
6. 若需要编译构建，必须先向用户确认。
7. 给出 `PASS` 或 `NEEDS_FIX`。

Auditor 发现问题后，不要大范围自行改造。  
应记录问题，并让 Modifier 进行针对性修复。

---

## 4. 改造记录要求

每次非平凡任务默认需要在修改完成后新建或更新记录文件：

```text
docs/ai_refactor_records/<YYYYMMDD>_<task_name>.md
```

如果用户指定记录文件，则使用用户指定路径。  
改造记录是**事后审计材料**，不是 writingplans，也不是修改前计划说明。  
不得为了撰写计划、改造计划或测试计划而中断已经确认的代码修改流程；用户与 Agent 商讨完毕后，应直接进行修改，再在完成后补充必要记录。  
如果用户明确要求本次不写改造记录，则以用户要求为准。

记录文件必须包含：

```md
# <任务名称> 改造记录

## User Intent

用户原始目标和关键约束。

## Scope

本次允许修改的范围。

## Out of Scope

本次明确不处理的内容。

## Explorer Findings

### Files inspected

### Active logic path

### Data flow

### Risk notes

### Recommended modification boundary

## Modifier Changes

### Files changed

### Key changes

### Behavior preserved

### Behavior intentionally adjusted

### Notes

## Auditor Review

### Checks performed

- [ ] 关键路径检查
- [ ] diff 检查
- [ ] grep 检查
- [ ] launch / yaml / CMake 检查
- [ ] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

### Final result

PASS / NEEDS_FIX
```

最终回答用户时，必须提供：

1. 修改摘要。
2. 修改文件列表。
3. 检查结果。
4. 改造记录路径。
5. `PASS` 或 `NEEDS_FIX`。
6. 如果未执行构建，必须说明“未执行构建，因为 AGENTS.md 禁止未授权构建”。

---

## 5. 修改前必须建立任务边界

修改前必须在内部明确任务边界；记录文件可在修改完成后补充。不得使用 writingplans 输出独立计划说明：

### 5.1 本次修改属于哪个模块

可选：

- map core / SlidingMap
- probability / ProbMap
- inflation / InfMap / CounterMap
- ESDF / 3D 距离场
- projection / Field2D
- raycasting / pointcloud input
- ROS 通信 / 发布链路
- parameter / yaml / config
- build / launch
- 文档 / 实验记录

### 5.2 本次修改目标是什么

可选：

- bug 修复
- 策略调整
- 性能优化
- 通信链路改造
- 参数暴露
- 日志/统计
- 清理历史残余
- 文档化

### 5.3 是否允许改变运行行为

必须明确：

- 不允许：只修 bug 或清理边界。
- 小幅允许：保持主逻辑，仅修边界条件。
- 允许：用户明确要求策略变化。

### 5.4 是否涉及已验证逻辑

如果涉及，必须特别保守，并在记录中写明：

```text
本次修改涉及已验证的建图/发布逻辑，采用最小改动策略。
```

---

## 6. 核心地图数据结构修改规范

涉及 `SlidingMap`、`ProbMap`、`CounterMap`、`InfMap`、`FreeCntMap` 时必须遵守：

1. `SlidingMap` 的索引与滑动语义是地基，优先只读调用，不重写。
2. `ORIGIN_AT_CORNER` 下，全局索引对应格心 `pos = (id + 0.5) * resolution`；origin 是格角，不要混用。
3. `mapSliding()` 会重置滑出局部地图的 cell，修改时必须同步 `resetCell` / `resetOneCell` 与 counter 状态。
4. `ProbMap` / `InfMap` / `ESDFMap` / `FreeCntMap` 的滑动必须保持一致，不能只滑其中一个。
5. 修改 `occupancy_buffer_` 的 log-odds 语义时，必须同时检查 `classifyProb`、`isKnownFree` / `isOccupied` / `isUnknown`、`l_free` / `l_occ`。
6. 栅格类型要保持 `UNKNOWN` / `KNOWN_FREE` / `OCCUPIED` / `OUT_OF_MAP` / `FRONTIER` 语义稳定。
7. 不随意改变 `resolution`、`inflation_resolution`、`map_size`、`fix_map_origin`、`virtual_ground_height`、`virtual_ceil_height`。
8. `esdf_resolution` 与 `resolution` 不是一回事：ESDF 是 `CounterMap`，按 `inflation_ratio` 降分辨率。
9. 修改 `init()` / `initProbMap()` 时，必须检查所有子图的滑动初始化是否齐全。
10. 不在热路径新增高频日志或全图扫描。
11. 不引入会破坏实时性的锁竞争；现有 mutex 的粒度不要随意改变。
12. 不擅自启用被注释或默认关闭的 `decay`、`frontier_extraction_en`、`unk_inflation_en` 分支。

地图核心修改后至少执行用户允许范围内的静态检查，例如：

```bash
grep -RIn "mapSliding\|slideAllMap\|resetCell\|resetOneCell\|ORIGIN_AT_CORNER\|occupancy_buffer_\|l_occ\|l_free" include/ src/
```

如需构建，必须先请求用户确认。

---

## 7. ESDF / 2D 投影修改规范

涉及 `ESDFMap`、`Field2D`、`reduceMinZ`、`queryCell2D`、`buildField2D` 时必须遵守：

1. 区分 3D ESDF 与 2D 投影：ESDF 是带符号距离，2D 是 min-over-z 归约。
2. `distance_buffer` 中占据为负、自由为正，不要反转符号。
3. 不重写 Felzenszwalb 三趟 EDT 与符号合并，除非用户明确要求。
4. `ESDFMap::resetLocalMap()` 不清 `distance_buffer`，更新盒外是陈旧值；所有查询必须先确认在更新盒内。
5. `buildField2D()` 的 `origin` 是格角，`getUpdatedBbox()` 返回格心，必须保留半格回退。
6. `queryCell2D()` 的 z 采样范围 = ESDF 更新盒 ∩ (`cur_odom_.z() + [projection.z_min, projection.z_max]`)。
7. 修改 `projection.z_min` / `z_max` 时，必须同时检查 `esdf.local_update_box.z`。
8. `Field2D::version()` 是发布门控，不要绕过。
9. `projection.enable` 只控制是否计算；发布者由 `visualization.enable` 创建，不要混淆。
10. `output_esdf` 二选一：`false` 发布 2D 栅格占用图，`true` 发布 2D 距离场点云。
11. 投影层耗时优化必须先确认瓶颈是采样、ESDF 查询还是发布。
12. 不引入会破坏实时性的高频日志。

ESDF / 投影修改后至少做静态检查：

```bash
grep -RIn "updateESDF3D\|getUpdatedBbox\|queryCell2D\|buildField2D\|reduceMinZ\|Field2D\|projection\|esdf_resolution" include/ src/ config/
```

构建前必须先获得用户许可。

---

## 8. 概率更新 / 射线投射修改规范

涉及 `RayCaster`、`raycastProcess`、hit/miss 更新、decay 时必须遵守：

1. 不改变点云输入 topic、odom topic、消息类型，除非用户明确要求。
2. 不随意改变回调频率、QoS、callback group，以及入口节点使用 `MultiThreadedExecutor` 的前提。
3. `p_hit` / `p_miss` / `p_occ` / `p_free` / `l_*` 是一组联动物理量，不能单独改一个而不检查阈值。
4. `batch_update_size` 控制攒帧，改大增加延迟；当前默认 1，不要无理由改大。
5. `ray_range`、`raycasting.local_update_box`、`virtual_ground_height` / `virtual_ceil_height` 与近处障碍处理强相关。
6. 修改 decay 时必须明确 `keep_time` / `clear_time` / active list 的关系。
7. decay 时间戳必须是 `double` 秒，不能退化为 float。
8. 动态障碍清除优先检查 decay 与 active 维护，不盲目调 hit/miss。
9. 不在 callback 中打印高频详细日志。
10. 修改点云处理时，必须确认下游使用的 frame、stamp、topic 不变。

概率更新 / 射线修改后至少做静态检查：

```bash
grep -RIn "raycastProcess\|p_hit\|p_miss\|l_occ\|l_free\|batch_update_size\|decay\|applyDecay\|local_update_box\|virtual_ground" include/ src/ config/
```

构建前必须先获得用户许可。

---

## 9. ROGMap / 地图修改规范

涉及 ROGMap、ProjectionLayer、ESDF、占据栅格、decay、栅格分类时必须遵守：

1. 区分 occupancy、ESDF、projection、inflate、active window。
2. 修改 decay 时必须明确 hit/miss/decay 的关系。
3. 动态障碍清除优先检查 decay 与 active 维护，不盲目调 hit/miss。
4. 投影层耗时优化必须先确认瓶颈是遍历、邻域、内存访问还是通信。
5. 栅格类型分类要保持 `KNOWN_FREE` / `OCCUPIED` / `UNKNOWN` / `OUT_OF_MAP` 语义稳定。
6. 不随意改变 map resolution、frame_id、map_size。
7. 不增加无用调试日志。
8. 参数加载逻辑必须清楚，不保留无意义 fallback，除非用户要求。
9. 修改 decay 时，若用户要求“min 内保持、max 衰减到 free”，不得额外引入多余 decay_time 概念。

ROGMap 修改后至少做静态检查：

```bash
grep -RIn "decay\|hit\|miss\|active\|projection\|esdf\|occupancy" include/ src/ config/
```

构建前必须先获得用户许可。

---

## 10. ROS 2 封装 / 发布链路修改规范

涉及 `rog_map_ros2.hpp`、`publishProj2D`、`publishESDF`、回调组时必须遵守：

1. `ROGMapROS` 是 header-only 类，修改后所有 include 它的入口都要一起检查。
2. 当前构建定义 `USE_ROS2`，`rog_map_ros1.hpp` 整体被宏包住，不参与当前编译；不要只改一边就假定两边一致。
3. 不改变已有 topic 名、消息类型、frame_id、stamp 语义，除非用户明确要求。
4. 订阅 QoS 为 `best_effort / keep_last(1) / volatile`，发布 QoS 为 `reliable / keep_last(1) / volatile`；修改前说明影响。
5. odom / cloud / update timer / viz timer 分属 `MutuallyExclusive` callback group，入口节点必须使用 `MultiThreadedExecutor`。
6. `updateCallback` 是 1 ms 定时器，负责建图与同步发布；不要在回调内新增阻塞操作。
7. `publishProj2D()` / `publishESDF()` 的版本门控是“生成一帧发一帧”，不要改回定时器重复发。
8. 发布者是否创建受 `visualization.enable` 控制，使用前必须判空。
9. `frame_id` 只给消息贴标签，rog_map 不做 TF 变换；不要假设它和 odom 实际 frame 一致。
10. `pub_odom_tf` 默认关闭，避免与 FAST-LIO 的 TF 冲突。
11. 不擅自修改 launch 组合方式、QoS、timer、callback group。

ROS 2 封装修改后至少做静态检查：

```bash
grep -RIn "create_publisher\|create_subscription\|create_wall_timer\|CallbackGroup\|publishProj2D\|publishESDF\|frame_id\|pub_qos\|QoS" include/ src/ Apps/ launch/
```

构建前必须先获得用户许可。

---

## 11. Launch / 构建修改规范

涉及 launch、入口节点、CMake、编译宏时必须遵守：

1. 确认节点是否由 `launch/rog_map.launch.py` 启动，以及 `config_name` 的解析路径。
2. 入口 `Apps/rog_map_node.cpp` 通过 `ROOT_DIR + "config/" + config_name` 读取配置，不依赖 install。
3. `config_name` 只是文件名，不要传绝对路径或把参数塞进 launch 之外。
4. 修改 launch 时必须确认参数名与 `Apps/rog_map_node.cpp` 的 `declare_parameter` 一致。
5. 当前 `CMakeLists.txt` 定义 `-DUSE_ROS2` 和 `-DORIGIN_AT_CORNER`；不要随意切换。
6. `src/*.cpp` 与 `include/*.h(.hpp/.cpp)` 被 `GLOB_RECURSE` 进静态库；新增文件注意不要被重复编入。
7. `Apps/` 下的入口单独 `add_executable`，不要放进 `src/`。
8. 编译选项含 `-Werror=unused-variable` / `-Werror=return-type`，新增代码必须保持无警告。
9. `ros/` 下的 ROS 1 / ROS 2 备用构建文件不是当前生效路径，修改前先确认用户意图。
10. 不擅自替换官方 launch 或构建方式，除非用户明确要求。

Launch / 构建修改后至少做静态检查：

```bash
grep -RIn "config_name\|declare_parameter\|add_definitions\|USE_ROS2\|ORIGIN_AT_CORNER\|GLOB_RECURSE\|add_executable" launch/ Apps/ CMakeLists.txt ros/
```

构建前必须先获得用户许可。

---

## 12. 参数修改规范

涉及 yaml、`Config` loader、默认值时必须遵守：

1. 不保留无意义 fallback，如果用户明确要求参数必须配置。
2. 参数名要和 yaml 一致，`Config` loader 的 key 必须能对上 `config/rog_map.yaml`。
3. 删除参数时必须删除读取、默认值、文档和 yaml。
4. 修改默认值必须说明原因。
5. 已验证建图参数改动必须写入记录文件。
6. 不擅自把硬编码改为参数，除非用户明确要求。
7. 不擅自删除现有参数兼容性，除非用户明确要求。
8. 新增参数必须同时更新 `config.hpp` 的 `LoadParam` 与 `config/rog_map.yaml`，不要只改一边。
9. `resolution`、`inflation_resolution`、`esdf_resolution` 之间存在换算关系，不能孤立调整。
10. 修改 `projection.z_min` / `z_max` 时必须检查 `esdf.local_update_box.z`；修改 `output_esdf` 时必须说明发布 topic 变化。

---

## 13. ROS 话题 / 消息接口修改规范

涉及 topic、frame、stamp、消息类型、发布开关时必须遵守：

1. 先确认消息类型、frame_id、stamp 语义，再改发布或订阅。
2. 保持旧 topic 兼容，除非用户明确要求。
3. 修改消息字段或发布开关时必须同步：
   - 发布端
   - 订阅端与下游使用方
   - launch / yaml 开关
   - 文档与记录
4. 不要只改发布，不改订阅。
5. 不要只改订阅，不检查下游。
6. 不要只改 topic 名，不改 yaml 与下游配置。
7. 对 QoS / frame / stamp 的修改必须写入记录文件。
8. 新接口接入建议分阶段：
   - 先发布并记录
   - 再接入下游
   - 最后修改使用逻辑

---

## 14. 静态检查与命令限制

允许优先运行：

```bash
git status
git diff
grep -RIn "<keyword>" .
rg "<keyword>"
find . -name "*.yaml" -o -name "*.py"
python3 - <<'PY'
# 静态解析脚本
PY
```

XML 检查可使用：

```bash
python3 - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path
for f in Path(".").rglob("*.xml"):
    try:
        ET.parse(f)
        print("OK", f)
    except Exception as e:
        print("FAIL", f, e)
PY
```

YAML 检查可使用 Python 解析，但不得触发构建或代码生成。  
Python launch 文件可用 `ast.parse` 做语法检查，不得生成字节码或触发运行时依赖：

```bash
python3 - <<'PY'
import ast
from pathlib import Path
for f in Path("launch").rglob("*.py"):
    try:
        ast.parse(f.read_text(encoding="utf-8"))
        print("OK", f)
    except Exception as e:
        print("FAIL", f, e)
PY
```

禁止在未获用户允许前运行构建命令：

```bash
colcon build
colcon build --packages-select rog_map
catkin build
catkin_make
cmake
make
ninja
cargo build
cargo check
npm run build
yarn build
```

无论是否获得构建许可，始终禁止提交类和版本控制写操作：

```bash
git add
git commit
git commit --amend
git push
git tag
git merge
git rebase
git cherry-pick
git revert
git reset --hard
gh pr merge
gh release create
```

如果用户明确允许构建，构建后必须记录：

1. 构建命令。
2. 构建是否通过。
3. 失败关键日志。
4. 是否是本次修改导致。

---

## 15. 最终输出格式

每次完成后，向用户输出结果摘要；不要输出 writingplans 式计划说明或测试计划说明：

```md
## 修改摘要

## 修改文件

## 行为保持说明

## 有意改变的行为

## 检查结果

## 改造记录

## 未解决问题

## Final Result

PASS / NEEDS_FIX
```

如果没有执行构建，必须明确写：

```text
未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。
```

---

## 16. 禁止事项汇总

除非用户明确要求，否则禁止：

1. 大规模重构。
2. 删除历史注释分支。
3. 改动多个无关模块。
4. 修改已验证的建图/发布策略。
5. 添加复杂新框架。
6. 添加大量 debug 输出。
7. 修改 topic / frame / QoS / 消息语义。
8. 修改参数默认值。
9. 修改 launch 组合方式。
10. 把未验证推断写成确定事实。
11. 未获许可执行构建。
12. 以“清理”为名删除仍可能复用的验证逻辑。
13. 将单模块任务扩展为全仓库重构。
14. 在未记录的情况下修改行为优先级。
15. 使用 writingplans 或类似计划说明工具。
16. 执行任何 `git add` / `git commit` / `git push` / 合并 / 标签 / 发布等提交类操作。

---

## 17. 可选的内部阶段化工作方式

对于复杂任务，可以在内部按阶段拆分，但不得使用 writingplans 输出长篇计划说明：

```text
Step 1：职责边界和残余清理
Step 2：数据结构 / 配置字段
Step 3：概率更新 / 射线 / 膨胀接入
Step 4：ESDF / 2D 投影接入
Step 5：ROS 发布 / 异常处理
Step 6：审计和清理
```

阶段拆分只用于控制修改风险。  
用户与 Agent 已经商讨清楚后，应直接修改代码；完成后再给出摘要、检查结果和风险说明。

---

## 18. 当前项目已知偏好

用户偏好：

1. 最小改动。
2. 少新增变量、函数、类。
3. 不整体重构。
4. 不增加无用调试。
5. 先分析问题，再列约束，再给方案，再给风险和验收。
6. 复杂任务使用 Explorer → Modifier → Auditor 流程。
7. 构建必须先询问用户。
8. 输出应明确哪些行为保持不变、哪些行为被有意改变。
9. 建图行为以当前 `config/rog_map.yaml` 与已验证代码路径为准，不按理论重新设计。
10. 对 ROGMap、ESDF、2D 投影、点云回调的改造必须保持实时性和通信链路稳定。
11. 不使用 writingplans 做计划说明、改造计划说明或测试计划说明。
12. 禁止任何提交类操作，Agent 只修改工作区文件，不暂存、不提交、不推送。
