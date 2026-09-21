# 2026-09-15：恢复原 EGO 链路与异常规划排查

> 本文记录上一轮完整撤回的历史。用户随后明确要求保留预测避障能力，现已按“预测约束参与规划、无额外控制接管”恢复；当前状态见 [预测约束接入说明](PREDICTION_ONLY_INTEGRATION.md)。

按要求撤回本次新增的 EGO 控制逻辑。空旷高空没有有效雷达回波是可能的正常观测结果，不能由旁路预测超时触发额外的保持/制动接管。

## 当前代码与数据链路

```text
Livox + MAVROS → lidar2word.sh（原机体过滤、坐标与速度转换）
              → /world_cloud + /ego/odom_world_velocity
              → 原 EGO 地图 / 状态机 / 优化器 → 原 MAVROS 控制桥

Livox + MAVROS → tracker → prediction / RViz（旁路，无 EGO 控制消费者）
```

- 已撤回：外部预测缓存消费、动态障碍代价、额外轨迹验收、预测健康/地面/超时阻断、自动保持与恢复、新增制动轨迹及接管。
- 原 EGO 的地图碰撞检查、规划失败处理与用户既有 early avoidance 保留。
- 规划核心源文件和单机 launch 已对照当前分支 HEAD 恢复；未覆盖用户已有 RViz、仿真 launch 等其他改动。
- `start_singlepoint_tracker.sh` 现在转到 `start_singlepoint.sh`。不再传入 `external_obstacles` 等已撤销的参数。
- tracker/预测桥保留旁路诊断，状态可能仍报告无地面或输入超时，但规划端不再订阅这些预测消息。M2 预测避障不再启用；原完整点云避障保留。

## 异常弧线的证据与修复

记录：`uav_application/rosbag/EGO-Planner-2.bag`。

105.48 秒，原始 Livox 数据出现一个近机点 `(0.19615, -0.23518, 0.000023)`，距离雷达 0.30624 米。新适配器只剔除 0.3 米以内回波，因此该点进入新 `/lidar_object_tracker/full` 输入。使用当时里程计和 0.43 米外参变换后，位置约为 `(-1.07943, 50.79445, 5.08463)`。

地图分辨率 0.35 米，XY 膨胀 5 格，Z 膨胀 1 格。105.928～107.075 秒记录的 12 帧非空膨胀地图，全部点都落在该近机点的膨胀核中；其中 106.076 秒 20/20 点、107.075 秒 1/1 点匹配。它覆盖飞行路径，触发“飞机在障碍物里”的日志、反复规划和减速。不能把这些点解释为新出现的远处实体障碍。

原 `lidar2word.sh` 的机体过滤为 X/Y 1.45 米、Z 0.9 米，该回波位于过滤区域内。恢复 `/world_cloud` 输入就是本次修复方式，不在 EGO 里增加绕过碰撞检查的条件。未修改原地图膨胀算法。

148 秒后的无有效回波导致新适配器不再输出，预测桥超时，随后新增 EGO 保持逻辑让飞机停住。这部分接管逻辑已移除。没有改高速度上限来掩盖问题。

## 下次启动

Gazebo、飞控、MAVROS、RViz、业务按原流程。保留：

```bash
SIMULATION_MODE=false LIDAR_Z=0.43 bash ~/tools/lidar2word.sh
bash ~/APS/EGO-Planner-v2/tools/start_singlepoint.sh
```

若要观察目标/预测，再单独启动：

```bash
bash ~/APS/EGO-Planner-v2/tools/start_tracker_shadow.sh rviz:=true
```

两个规划入口只启动一个。已运行进程不会因源码重新编译而自动切换；本次没有重启正在运行的飞行节点。

## 验证与限制

- main_ws 编译通过。
- 隔离 ROS 测试通过：持续发布不健康预测、随后停止点云输入，EGO 仍能发布运动轨迹；预测发布端对 EGO 的连接数为零，无已撤销的外部状态机输出。
- 107 秒现场点云的原输入与单帧输入静态夹具均通过隔离规划测试。原 `/world_cloud` 夹具轨迹从 y=52.627 前进到 y=78.128，x 仅从 -1.230 变为 -1.150，无大幅横向弧线。
- 测试使用静态里程计和点云夹具，不等于整段飞行回放或完整自动航线验收。下次完整 Gazebo 飞行仍需验证；当前运行节点尚未加载撤回后的二进制。

证据及撤回前代码备份：`/home/zhoubao/APS/lidar_object_tracker_diagnostics/20260915-restore-ego/`，近机点核对结果为 `self-return-evidence.json`。
