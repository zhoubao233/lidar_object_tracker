# Tracker 环境点云与动态预测统一接入

更新：2026-09-16。新 `start_singlepoint_tracker.sh` 统一启动感知、预测、环境点云适配、速度转换、TF 和 EGO，不再需要运行 `lidar2word.sh` 或 `start_tracker_shadow.sh`。

## 使用方式

Gazebo、SITL、MAVROS、业务维持现有流程。切换时先结束旧的 `lidar2word.sh`、`start_tracker_shadow.sh` 和 EGO 规划启动终端，避免同名节点、旧 TF 或重复发布者混用。然后只运行：

```bash
bash ~/APS/EGO-Planner-v2/tools/start_singlepoint_tracker.sh map_fading_time:=0.1
```

需要 tracker 的 RViz 时：

```bash
bash ~/APS/EGO-Planner-v2/tools/start_singlepoint_tracker.sh map_fading_time:=0.1 tracker_rviz:=true
```

已有独立规划 RViz 可以继续使用。更新后的配置增加 `Tracker environment (static + unknown)`，话题 `/tracker_environment/cloud`；旧 `/world_cloud` 显示没有数据属于预期。已打开的 RViz 需重载配置或手动添加该 PointCloud2 话题。

当前入口面向 mission_auto_avoid 仿真：Livox CustomMsg、瞬时扫描、MAVROS map/base_link、0.43 米安装外参。不是宣称这套安装参数已经适用于真机。

## 实际输入链路

```text
MAVROS → 速度/时间转换 → /ego/odom_world_velocity → EGO
                  └→ tracker 位姿配对
Livox → 零值/范围/机体过滤 → 雷达安装外参 → body 点云
      → tracker 自运动补偿 → map 世界点云与目标状态
      ├→ 目标预测 → /tracker_prediction/predictions → EGO 动态约束
      └→ 逐点目标 ID → 环境点云适配 → /tracker_environment/cloud → EGO 地图
```

map/world 静态 TF 由新入口发布，保留原场景同原点同方向约定。速度转换复用了旧转换节点的源码逻辑：旋转线速度、角速度和 twist 协方差，按当前 ROS 时间输出。原点云转换器本身不再启动；不再依赖旧 Fast-Drone-250 工作空间来提供这两个辅助功能。

机体过滤沿用 X/Y 半宽 1.45 米、Z 半高 0.9 米；范围 0.05～80 米，安装高度 0.43 米。过滤在雷达/机体系完成，之后通过既有位姿变换到 map；不对已经是世界坐标的点云重复转换。输出只有当前帧，没有跨帧点云累积。

## “静态点云”的准确含义

EGO 接收的是环境点云：静态候选、地面、UNKNOWN、未分组点，以及当前没有有效预测覆盖的运动目标点。不能只保留状态为 STATIC 的簇，否则地面、墙体和尚未确认的障碍可能丢失。

新增 `/lidar_object_tracker/classified` 在当前完整世界点云中携带 `track_id`：0 表示保留为环境，正值表示该点属于本帧观测到的 MOVING 目标。它依据聚类索引标注，不使用包围盒整体抠除，因此相邻或重叠的静态点不会仅因进入目标包围盒而被删除。

`tracker_environment_node` 等待同一时间戳的预测快照（最多约 40 ms）：

- 同帧、同坐标且预测有效：剔除该预测对应 ID 的运动点，保留其余点。
- 预测无效、没有匹配快照、超时或目标未被预测覆盖：保留完整当前帧。
- 已分流后预测过期：将最近一次实测中失去预测覆盖的点恢复到环境输入，保留原时间戳，不伪造新观测。
- 空扫描：发布空环境点云；不重复累积历史点，不触发额外 EGO 保持或接管。
- `external_obstacles:=false`：环境输入保留完整点云，避免关闭预测后动态物体从两条避障输入中消失。

地面不可见但跟踪已确认运动时，逐点 ID 仍可用于预测覆盖下的分流。原 tracker 的 background/dynamic 可视化语义保留，规划器使用专用 `/tracker_environment/cloud`，不要把 background 当成严格等价输入。

## 地图残影与行为边界

动态目标被确认之前，UNKNOWN 点仍会进入地图；确认后，这些旧占据按原地图衰减消退。本次没有按目标包围盒强行清空地图，也没有修改 EGO 的地图算法。

`map_fading_time:=0.1` 仍受约 0.5 秒衰减检查周期限制；0 表示关闭衰减。静态障碍在稀疏回波时也可能快速消失，完整场景效果需复测。分流不意味着任何时刻的避障都完全由预测独立完成，UNKNOWN、预测回退和已有占据仍参与地图避障。

没有恢复预测健康阻断、自动保持/恢复、制动接管；新增判断只决定哪些实测点进入地图。原 EGO 状态机和 MAVROS 控制桥未改。

## 验证状态

- tracker 与 EGO 工作空间编译通过。
- 环境节点隔离测试通过：精确按目标 ID 剔除、近邻未知点保留、未覆盖目标保留、预测过期恢复、时间戳不匹配回退、无效预测与空帧。
- 统一入口隔离测试通过：没有 `/world_cloud` 发布者，近机/零值/超范围点被过滤；490 个有效点保留；旋转姿态下速度与点云变换正确；无地面移动目标获得 ID 和预测并完成分流；空输入后 EGO 仍发布轨迹。
- tracker 完整回归通过：catkin_test_results 汇总 56 项，0 errors、0 failures、0 skipped。
- 没有切换当前正在运行的飞行节点；完整 Gazebo 移动场景和真机验收仍待完成。

复现环境路由及统一启动测试：先加载 driver、tracker 和 EGO 工作空间，再运行 EGO 工程 `tracker_ego_bridge/test/check_environment.py`、`check_integrated_startup.py`。测试自行创建独立 ROS master，不发布到当前飞行系统。

备份与日志：`/home/zhoubao/APS/lidar_object_tracker_diagnostics/20260916-static-integration/`。

## 回退对照

原 `start_singlepoint.sh` 保留原 `/world_cloud` 模式。只有选择该旧入口时，才重新按旧流程运行 `lidar2word.sh`；它不属于新的统一入口。

## 在规划 RViz 中查看目标和预测

重新打开 `bash ~/APS/EGO-Planner-v2/tools/start_singlepoint_rviz.sh`，展开 `Dynamic obstacles` 分组：

- `Tracked objects - boxes IDs velocity`：订阅 `/lidar_object_tracker/markers`，显示目标框、ID、状态和速度箭头。
- `Predicted trajectories - cyan`：订阅 `/tracker_prediction/markers`，显示青色未来轨迹及半透明终点范围球；仅有效预测目标有轨迹。

无需另开 shadow 或 tracker RViz。已打开的 RViz 需要重新加载配置或仅关闭该 RViz 窗口后重开；无需重启规划和业务。框显示跟踪目标，不能仅凭有框判断该目标存在有效预测。
