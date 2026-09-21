# EGO 外部动态障碍接入（M2）

> **2026-09-16 更新**：新 `start_singlepoint_tracker.sh` 已统一启动 tracker、预测、环境点云、速度转换和 TF，EGO 使用 `/tracker_environment/cloud`；不再单独运行 `lidar2word.sh` 或 `start_tracker_shadow.sh`。旧启动方式仅用于历史/原模式对照。当前说明见 [环境点云统一接入](STATIC_ENVIRONMENT_INTEGRATION.md)。

> **当前状态（恢复预测能力后）**：动态预测重新参与优化代价和候选轨迹碰撞检查；预测为空、过期、停更或不可用时仅移除对应预测约束，不增加保持、恢复、制动接管或健康阻断。地图与里程计继续走原 `/world_cloud`、`/ego/odom_world_velocity`。预测入口为 `start_singlepoint_tracker.sh`，原模式入口仍为 `start_singlepoint.sh`。当前实现、启动及验证范围见 [预测约束接入说明](PREDICTION_ONLY_INTEGRATION.md)。

日期：2026-09-15。当前已实现规划端外部障碍缓存、优化代价、安全检查和失效保持；8 个核心测试、启用新功能的 ROS 集成测试和关闭新功能的旧模式回归测试均通过；实际 Gazebo 交会和真机验收仍待进行。地图仍使用完整点云，M3 的动静分流与旧占据清除尚未实施。

## 使用现有仿真链路

Gazebo、SITL、MAVROS、lidar2word.sh、tracker 预测发布端及业务节点仍按原流程启动。下一次启动路径规划时，将 `start_singlepoint.sh` 换成：

```bash
bash ~/APS/EGO-Planner-v2/tools/start_singlepoint_tracker.sh
```

不要同时启动两个单机 EGO 入口。预测发布端仍使用：

```bash
bash ~/APS/EGO-Planner-v2/tools/start_tracker_shadow.sh rviz:=true
```

这个预测发布端脚本保留原名称；新规划入口会消费它的输出。无需重复启动 tracker。

新入口明确设置以下输入：

| 内容 | 输入/配置 |
|---|---|
| 动态预测 | `/tracker_prediction/predictions` → 独立外部障碍缓存 |
| 地图点云 | `/lidar_object_tracker/full`，包含所有有效点，不剔除 MOVING |
| 位姿 | `/mavros/local_position/odom`，map/base_link |
| 速度 | 将 MAVROS 的机体系线速度按里程计姿态旋转到 map 后供 FSM 使用 |
| 规划坐标 | map；严格检查预测和里程计 frame，不做名称替换或猜测 TF |
| 多机广播 | 保留原接口；外部目标不进入 swarm ID 向量或前序无人机握手 |

新入口与旧入口使用同一套规划/控制节点名称，控制桥仍遵循原有启停和 MAVROS 状态门控。本次开发和测试没有重启用户正在运行的规划器，也没有启用控制桥或改变飞控模式。

暂时保留 lidar2word.sh：新规划入口不再消费它的 world_cloud，但原 RViz 配置和 map/world 静态 TF 仍可用于对照。统一 RViz 和删除旧转换启动项放在 M4。tracker 的 full 输出尚未包含所有真机所需的自体点过滤策略；本次仅检查了当前仿真采样中没有近机体点，不能外推到真机。

## 查看接入状态

```bash
rostopic echo /drone_0_ego_planner_node/external_obstacles/status
rostopic info /tracker_prediction/predictions
rostopic info /lidar_object_tracker/full
```

- `ready=1`：预测、坐标、时间、剩余预测窗和里程计门控满足。
- `targets=N`：缓存中的目标数；ready=0 时这些目标不能当作有效输入。
- `hold=1`：已发布当前观测位置的保持轨迹，等待恢复条件。
- `external_peak_cost`：最近一轮优化中外部障碍的最大单采样点代价。大于零表示该代价参与过优化；不是最小间距，也不是避障成功证明。
- `accepted/rejected`：完整快照接收/拒收计数；`reason` 给出健康门控信息。

运行新入口时，预测话题应有 EGO 订阅者，full 点云应有地图转发节点订阅者。只有看到动态点云或预测 Marker，不能证明规划器已经使用了预测。

## 时间、几何与状态约定

外部快照按 session_id、epoch、时间戳和 source_id 管理。接受新快照后整体替换；空且健康的快照表示当前无动态目标。重复、乱序、旧 epoch、已退役 session、非法数值/尺寸、坐标不匹配或发布端不健康均使外部输入失效。首版只接受桥接端生成的一段匀速五次 MINCO 边界，其他轨迹形式明确拒绝。

报告最大年龄及接收时的目标观测最大年龄均为 0.5 s，同时检查 ROS 时间与墙钟接收超时。目标在预测桥自然过期时单独剔除，不拒绝其他有效目标；接收后在该快照的有界生命周期内按轨迹有效域检查，不再因观测年龄跨过 0.5 s 而单独切换整帧健康状态。规划/安全检查每次要求预测至少覆盖未来 1 s；轨迹查询超过 valid_until 或早于起点属于未知，不能当作自由空间，也不无限外推。EGO 的局部轨迹可以长于 1 s，但动态约束及接受检查仅覆盖滚动的 1 s 前缀；新观测、安全定时检查和重规划持续更新这段检查范围。默认前缀不得短于 fsm/emergency_time。

安全范围采用球形：目标预测 clearance + 无人机的 external_obstacles/own_clearance（新入口默认继承 obstacle_clearance；不再使用 demo 的 0.15 m 多机参数），再加速度界与 0.02 s 采样间隔计算的离散检查余量。接受检查验证候选轨迹速度不超过配置速度界。优化器在同一个硬安全球外增加 0.2 m 软缓冲；最终接受和运行安全检查仍按硬安全球判断。不会沿用多机分支中不同的椭球与距离倍率。

- 更新或删除目标：下一轮安全检查与重规划使用新完整快照。
- 停更、非法快照、时钟重置、session/epoch 变化：已有任务进入保持。重置时即使该帧健康，也先保持并要求后续新帧。
- 恢复：输入重新健康、收到比保持时更新的快照、观测速度低于 0.1 m/s，且 fail_safe 允许、没有 mandatory_stop，才从观测位置重新规划。
- 近期动态冲突：尝试重规划，失败则使用现有 EmergencyStop 保持轨迹。

外部预测模式的急停从实测位置与世界系速度生成单调减速轨迹，终点在原速度方向上。减速指令满足配置的加速度及 jerk 上限；结束后保持终点。制动期间不反复重置轨迹。外部健康恢复需连续至少 0.3 s、制动时长已结束且实测速度小于 0.1 m/s。旧入口保持原行为。该制动轨迹没有无碰撞保证，也不能代替飞控实际制动距离和真机最小间距验证。

## 构建和自动验证

```bash
source /opt/ros/noetic/setup.bash
cd ~/APS/EGO-Planner-v2/swarm-playground/main_ws
catkin_make -j2 -l2 ego_planner_node tracker_prediction_node
catkin_make -j2 -l2 run_tests_traj_opt run_tests_ego_ardupilot_bridge
source devel/setup.bash
rostest ego_ardupilot_bridge external_obstacles.test enabled:=false
```

8 个核心测试覆盖有效时间域、空快照与停更、快照删除/重启/乱序、非法输入、空间与两种时间梯度的有限差分、短时间窗积分截断后的系数/时长梯度，以及空地图下横穿目标改变轨迹可接受性。
ROS 集成测试运行在独立 master 上，向真实 EGO 节点提供合成里程计、固定地图和预测，检查动态代价、轨迹变化、近距离冲突保持、停更保持、恢复及 reset。测试不连接用户正在运行的 Gazebo/MAVROS 主系统。

回退时，下次启动规划器改回原 `start_singlepoint.sh`；默认 external_obstacles=false，重新使用原 world_cloud。M2 尚不包含 background 分流、地图旧占据清除、纯点云健康联动以及真机验收。

本次通过的测试日志、确定性轨迹对照和启动参数保存在 `/home/zhoubao/APS/lidar_object_tracker_diagnostics/20260915-ego-external/`。测试通过说明软件行为与这些合成场景相符，不替代实际 Gazebo/真机交会验收。


## 2026-09-15：起飞后地面不可见的处理

现场原因为：高度约 5.03 m，雷达高度约 5.46 m；当前扫描向下约 7.2°，水平姿态下地面回波需要约 43 m 量程，超过仿真雷达 40 m 和跟踪分割范围 25 m。原来 ground_valid=false 被直接转换成预测不健康，导致 EGO hold。

当前修正采用可选的无地面三维聚类，不延长旧地面平面的有效期，也不伪造 ground_valid：

- 有可靠地面：保持原来的地面过滤和运动分类。
- 无可靠地面：`allow_groundless=1` 时对范围内所有点聚类，继续原有时序关联和运动证据判断；报告 `ground_valid=false, segmentation_valid=true, segmentation_mode=unfiltered`。
- 此模式下 dynamic 点云为空，目标点保留在 background/uncertain 中；目标框、速度和运动预测仍可输出。full 始终保留所有输入点，EGO 当前使用 full。
- 全空或全无效点云：`segmentation_valid=false, segmentation_mode=unavailable`，预测不健康。仅范围外存在有效回波时，可以形成范围内零目标的有效报告；这不代表范围外没有障碍。
- 未显式启用的旧入口保持严格地面要求。当前 mission_auto_avoid 入口开启无地面聚类，shadow 入口同时允许预测桥消费这种报告。
- 预测桥将其标为 `healthy=true, reason=groundless_full_cloud_required`。EGO 按实际 lidar_topic 是否为 `/lidar_object_tracker/full` 设置接收资格，否则拒绝该降级报告。超时、坐标错误、非法轨迹等原有保护继续生效。

这个降级策略可以缓解地面退出视野造成的停飞，但不能恢复雷达视野之外的目标。无地面聚类可能将相连的地面与物体合并；大簇仍按原规则过滤，所以不能据此宣称所有动态障碍都已可检测，也不能直接切到只含静态点的地图。

### 验证与重新启动

新增测试覆盖：地面消失后的运动识别、冷启动静止目标、空/无效输入、地面恢复、预测协议兼容与非法状态拒绝、EGO 完整点云约束及超时。另有可单独运行的真实节点集成测试（独立 ROS master，不连接飞控）：

```bash
source /opt/ros/noetic/setup.bash
source ~/APS/livox_ros_driver2_ws/devel/setup.bash --extend
source ~/APS/lidar_object_tracker_ws/devel/setup.bash --extend
source ~/APS/EGO-Planner-v2/swarm-playground/main_ws/devel/setup.bash --extend
rostest tracker_ego_bridge groundless_tracker.test
```

修改不会更新已经运行的节点。结束当前飞行并落地、解除解锁后，停止旧 tracker/shadow 和 planner，再按原顺序启动：

```bash
bash ~/APS/EGO-Planner-v2/tools/start_tracker_shadow.sh rviz:=true
bash ~/APS/EGO-Planner-v2/tools/start_singlepoint_tracker.sh
```

地面不可见但聚类正常时，预测状态应出现 `healthy=true` 和 `groundless_full_cloud_required`；EGO 在里程计、预测新鲜度等其余条件满足后应为 `ready=1 hold=0`。完整 Gazebo 自动任务起飞复测仍待现场执行，本次没有重启飞行中的节点。

本轮离线验证结果：tracker 24 个、prediction 9 个、external cache 9 个单元测试通过；原有 tracker/bridge/FSM ROS 测试通过；新增 tracker→prediction 地面消失集成测试、EGO groundless 模式 FSM 测试及 external disabled 回归通过。日志位于 `/home/zhoubao/APS/lidar_object_tracker_diagnostics/20260915-groundless/`。

EGO 降级模式复测命令：`rostest ego_ardupilot_bridge external_obstacles.test groundless:=true`。这些测试采用合成观测/目标，不能替代当前 Gazebo 场景和真机验证。


## 后退与频繁起停修正（2026-09-15）

最新记录中，固定位置保持点曾落后实际位置约 1.49 m，实际最大回退约 0.72 m；后续9秒内保持状态切换47次。修正分为：目标自然过期按目标处理；快照接收后按预测有效域继续使用；急停改为从实测速度减速；制动期间禁止反复替换；健康稳定后才恢复。

制动速度为 v(t)=v0*(1-3s²+2s³)，s=t/T；终点 p0+v0*T/2。取 T=max(0.2,1.5*|v0|/a_max,sqrt(6*|v0|/j_max))，因此指令不会沿原运动方向反向。起始加速度设为零，未使用不可靠的里程计差分加速度；实际飞控响应仍需SITL验证。

新增录制观测回放：`rostest tracker_ego_bridge recorded_dropout.test`；移动中断流制动/恢复：`rostest ego_ardupilot_bridge braking.test`。参数与代码重新编译后，需在结束当前飞行、落地解除解锁后重启 shadow 和 planner，当前进程不会热更新。地图膨胀冲突和疑似扫描表面误跟踪尚需独立核实，此修正不宣称消除了所有停顿或反向绕障。

本轮验证完成：预测桥10项单元测试、外部缓存与制动11项单元测试均通过；录制目标过期回放、预测生命周期、正常外部模式、移动中断流制动恢复、无地面外部模式、旧入口模式的ROS测试通过。日志在 `/home/zhoubao/APS/lidar_object_tracker_diagnostics/20260915-braking-fix/`；单元测试最终结果以 `ego-braking-unit-final.log` 为准，初次测试日志保留了已修正的测试数据问题。未重启现场节点，完整自动任务复测待执行。
