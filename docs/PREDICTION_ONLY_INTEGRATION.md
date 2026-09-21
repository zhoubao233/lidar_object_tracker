# 预测避障保留，额外阻断与接管不恢复

> **2026-09-16 更新**：新 `start_singlepoint_tracker.sh` 已统一启动 tracker、预测、环境点云、速度转换和 TF，EGO 使用 `/tracker_environment/cloud`；不再单独运行 `lidar2word.sh` 或 `start_tracker_shadow.sh`。旧启动方式仅用于历史/原模式对照。当前说明见 [环境点云统一接入](STATIC_ENVIRONMENT_INTEGRATION.md)。

日期：2026-09-15。用户明确要求保留动态预测避障。上一轮将预测代价一起撤回的范围过大，本轮恢复必要能力，地图仍使用用户已经复测正常的原链路。

## 当前功能边界

| 能力 | 当前处理 |
|---|---|
| 根据目标位置、速度和时刻预测未来位置 | 保留预测桥 |
| 比较无人机与目标在同一未来时刻的位置 | 恢复到 EGO 优化代价 |
| 候选轨迹与有效预测相撞 | 进入原优化器的碰撞重试路径 |
| 无目标、空扫描或无地面 | 不新增全局阻断或控制接管 |
| 预测过期、停更或不可用 | 移除该预测源/目标的约束，EGO 原地图和规划继续 |
| 非法数值、错误 TF、过期目标 | 仅使相应预测不可用；单个无效目标不拖累其他有效目标 |
| 预测驱动的保持、自动恢复、制动轨迹 | 不恢复 |
| EGO 原有碰撞、规划失败与控制桥逻辑 | 保留 |

预测约束会因实际预测碰撞影响候选轨迹或导致原优化器重试，这是所需的动态避障行为；预测缺失本身不会触发新增的急停/保持。没有保证任何移动场景一定可解，实际效果需移动障碍测试。

## 数据路径

```text
原 LiDAR 转换 → /world_cloud → EGO 地图
原速度转换   → /ego/odom_world_velocity → EGO
tracker      → prediction → TF 到规划坐标 → 有效预测约束 → EGO 优化
EGO 原轨迹输出 → 原 MAVROS 控制桥
```

不把地图切回 `/lidar_object_tracker/full`，避免再次引入上次近机回波问题。map/world 不靠修改名字对齐：消费者使用已有 TF 在消息时刻变换预测位置和速度；无 TF 时跳过该源预测。

当前首版接受预测桥输出的单段匀速 MINCO 内容。预测使用有限时间域；无效目标跳过，空快照清理，输入停更后按原有预测年龄参数失效。每次优化开始固定有效目标集合，避免优化迭代中的梯度随回调变化。状态检查仅管理约束，不作为规划器 readiness 门槛。

## 启动

Gazebo、SITL、MAVROS 和业务按当前正常方式启动。继续运行原转换：

```bash
SIMULATION_MODE=false LIDAR_Z=0.43 bash ~/tools/lidar2word.sh
```

启动 tracker 和预测桥：

```bash
bash ~/APS/EGO-Planner-v2/tools/start_tracker_shadow.sh rviz:=true
```

启用预测规划：

```bash
bash ~/APS/EGO-Planner-v2/tools/start_singlepoint_tracker.sh
```

该入口只为原规划脚本增加 `external_obstacles:=true`，不改变地图、里程计与速度参数。只启动一个 EGO。若要原模式对照，使用 `start_singlepoint.sh`；也可给预测入口追加 `external_obstacles:=false`。无需为预测超时增加任何飞控模式切换。

默认预测约束前视 1 秒，可用 `external_obstacles_lookahead:=2.0` 配置；实际计算不超出目标预测有效期。输入和观测年龄默认 0.5 秒，余量沿用参数，仍需根据移动目标实测误差标定。

## 修改范围与验证

- 修改预测缓存/代价、优化器接入和 manager 预测订阅/TF 适配。
- 增加 launch 开关及预测入口；未改 EGO FSM、原地图代码和 MAVROS 控制桥。
- 单元回归覆盖未来运动位置、正负代价、解析梯度、空输入、超时、失效目标隔离、无地面报告与 reset。
- 隔离 ROS 回归覆盖关闭/开启预测时的不健康输入与点云停更，以及移动预测对已发布轨迹的影响。
- 本轮 main_ws 编译通过，5 项预测/梯度单元测试通过。
- 原模式和预测启用模式的无效预测/点云停更 ROS 测试均通过。
- 独立进程执行“空预测→横穿预测→空预测”三次对照：两次空预测首条轨迹一致；横穿预测使前 1.8 秒的轨迹采样最大变化约 0.419 米（包含减速产生的位移变化，不是额外安全距离）。三次测试均验证停更后仍可规划。
- 测试包含非零平移和旋转的 TF，验证预测位置与速度变换的接入路径。
- 未重启当前飞行节点。上述是合成输入的隔离测试，不等同完整 Gazebo 移动障碍或真机验收；这两项仍未完成。

可复现的测试命令：

```bash
source /opt/ros/noetic/setup.bash
source ~/APS/EGO-Planner-v2/swarm-playground/main_ws/devel/setup.bash
cd ~/APS/EGO-Planner-v2/swarm-playground/main_ws
catkin_make -j2 -l2 run_tests_traj_opt_gtest_test_prediction_constraints
rostest ego_ardupilot_bridge original_planner.test
rostest ego_ardupilot_bridge original_planner.test predictions:=true
python3 src/ego_ardupilot_bridge/test/run_prediction_comparison.py
```

代码备份与测试记录：`/home/zhoubao/APS/lidar_object_tracker_diagnostics/20260915-prediction-only/`。

剩余工作见 [总计划](REMAINING_DYNAMIC_AVOIDANCE_PLAN.md)。先用当前正常链路录制移动目标对照，再在同一场景验证预测开关效果；静态运行正常不等于移动预测避障已验收。

## 2026-09-16：关闭转换器历史点云累积

`~/tools/lidar2word.sh` 现在显式使用 `max_buffer_size:=0`。转换节点将 0 解释为只发布当前帧，保留这一帧全部过滤后的有效点；不会把上一帧的点拼接进来。当前扫描全部为零值或被过滤后，会发布带当前扫描时间戳的空点云，不重复发布旧回波。

机体过滤、雷达外参、世界坐标转换、里程计同步和预测约束未改变。消息同步队列用于雷达与里程计配对，仍保留；它与历史点云累积不同。EGO 占据地图自身的记忆/衰减未改，关闭转换器累积并不意味着空扫描会清空地图。

编译通过；在独立 ROS master 下分别验证 PointCloud2 和 Livox CustomMsg：第二帧不包含第一帧点，近机/零值点被过滤，空帧输出 0 点，单帧 11001 个有效点全部保留，坐标和时间戳正确。完整飞行尚未复测，当前飞行进程未重启。

启动命令不变，下一次启动转换器时生效。通用 `livox_to_world.launch` 的正数缓存参数仍保留兼容，当前 `lidar2word.sh` 已关闭它。备份和验证记录位于 `/home/zhoubao/APS/lidar_object_tracker_diagnostics/20260916-single-frame/`。

### 重启后仍看到拖影时

现场采样已确认：`/world_cloud` 与送入 EGO 的 `/drone_0_external/cloud` 同期每帧 440～604 点，最后一帧时间戳和点数一致，转换器 `buffered=0`。膨胀地图同期约 9683～11320 点；地图分辨率/膨胀与约 3 秒占据衰减是另一层状态，不是原始点云历史缓存。

EGO RViz 的 `drone0 → Mapping → map inflate` 显示这层地图；临时取消勾选可单独查看 `simulation_map`（`/world_cloud`）。配置中另一个默认关闭的 `PointCloud2` 同样订阅 `/world_cloud`，但原 Decay Time 为 10000，若打开会由 RViz 累积历史点；现已将保存配置中的该值改为 0。已打开窗口需手动把该项 Decay Time 改为 0，或下次加载更新配置。未修改 EGO 地图衰减或清除逻辑。
