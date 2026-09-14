# 历史记录：初次 Python 拆包

当前项目已转换为 C++，最新说明见 CPP_MIGRATION.md 和 README.md。以下为历史过程。

# 独立包迁移验证（2026-09-11）

- 原仓库 HEAD：a542aef2ea27d4b346e463c7c04a0482f58a61a4；迁移取当前工作树，包括已有未提交修改。
- 原 M-detector 仓库文件未改动；未启动、停止或重新配置用户的雷达、飞控和原跟踪节点。
- 独立 catkin 工作空间：/home/zhoubao/APS/lidar_object_tracker_ws，仅构建 lidar_object_tracker。
- 跟踪核心算法保持原样；修改 Python 导入、ROS 包安装、启动命名空间及仿真 max_points 参数读取。
- 新节点 /lidar_object_tracker，输入适配器 /lidar_object_tracker/adapter。
- 仿真和真机由一条 run.sh 命令同时启动适配器、跟踪器和可选 RViz。
- 未迁移原 dynfilter、历史深度图算法或 FAST-LIVO2 历史对齐代码。
- 保留离线 evaluate_bag.py，支持旧、新输出话题；它仅适用于有 Gazebo 箱子真值的验证录包。

## 已通过

1. ROS Noetic 下独立 catkin 构建。
2. 17 项原有单元测试：跟踪 9 项、仿真适配 3 项、真实扫描补偿 5 项。
3. 两套完整 launch 通过 run.sh --nodes 参数解析，节点均属于新包。
4. Bash 语法检查。
5. tests/check_pipeline_ros.py 启动独立临时 ROS Master，仿真、真机模式分别收到
   24 帧跟踪报告，均识别到合成运动目标；输出坐标系为 map。
   测试只发布 /migration/raw 和 /migration/pose，完成后停止本次测试进程。

## 范围

此次验证证明拆包后构建、导入、ROS 消息链路与既有行为可用。没有再次验证
现场 Gazebo 场景或真实飞行、真实运动目标的准确性，也没有打开 RViz GUI。
迁移前的现场量化结果见 HISTORICAL_VALIDATION.md，不应当作新包新测结果。

## 复现端到端测试

```bash
source /opt/ros/noetic/setup.bash
source ~/APS/livox_ros_driver2_ws/devel/setup.bash --extend
source ~/APS/lidar_object_tracker_ws/devel/setup.bash --extend
cd ~/APS/lidar_object_tracker
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tests/check_pipeline_ros.py
```
