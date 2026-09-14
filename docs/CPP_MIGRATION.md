# Python → C++17 迁移验证（2026-09-11）

## 范围

新仓库中的全部项目 Python 源码均已替换，包括跟踪核心、ROS 接口、仿真适配、
真机逐点补偿、离线录包评分与测试。原 M-detector 仓库未改动。
Shell、launch、YAML、RViz 和文档保留。ROS/catkin 工具链使用的系统 Python 不属于项目代码。

转换前的本机临时备份：/tmp/lidar_object_tracker-python-before-cpp-20260911.tar.gz。
该文件是临时备份，不随仓库分发，也不保证长期保留。

## 行为与接口

- 保留参数名称和默认阈值、四种状态、完整原始动态点输出、JSON 字段及 RViz 显示。
- 保留精确纳秒配对、有界缓存、优先处理最新配对和单工作线程执行算法。
- 真机保留逐点时间检查、位置插值、四元数 SLERP、扫描末参考时刻及禁止外推。
- 仿真仍无逐点去畸变；C++ 使用有界最近时间一对一匹配，异步到达时配对选择
  可能与 Python ApproximateTimeSynchronizer 不同。
- PCL 最近邻使用 float；Eigen 地面拟合和 SVD 在数值细节上可能与 SciPy 不同。
  下面是已验证数据的结果，不宣称任意场景逐比特等价。

## C++ 测试

21 项核心单元测试，覆盖原有 17 项行为，并补充配置/缺失地面、全局关联、
多目标 ID 和带行填充的大端 Float64 点云读取。
两套 rostest 分别在隔离 ROS Master 上验证仿真和真机链路。
合成输入只发布到 /migration/raw、/migration/pose，不接触用户运行中的雷达和飞控。

## 同一录包对照

数据：/home/zhoubao/APS/M-detector_diagnostics/20260909-object-tracking/final_live.bag。
两种语言都使用本次迁移前的同一默认参数，按消息原时间戳离线重跑：

| 指标 | Python | C++ |
|---|---:|---:|
| 完整输入帧 | 270 | 270 |
| 排除启动阶段后的计分帧 | 255 | 255 |
| 计分帧 MOVING | 255 | 255 |
| 箱子 ID | 1 | 1 |
| 上部动态覆盖率 | 99.663356% | 99.663356% |
| 下部动态覆盖率 | 97.221597% | 97.221597% |
| ROI 外动态点均值 | 0 | 0 |
| 算法耗时中位数（本机本次回放） | 6.03 ms | 3.71 ms |

逐帧比较 270 帧的时间、对象数量、运动数量、箱子 ID/状态、输入点数、动态点数，
没有差异。速度最大绝对差约 1.67e-14 m/s。
耗时只代表当前 WSL 上本次离线运行，不代表机载机或端到端延迟。

--recorded 模式也对同一录包进行了两语言对照：语义指标一致，
时间分位数有浮点末位舍入差，全部在 1e-12 容差内。

本机本次结果目录（不随仓库分发）：
- /tmp/lidar-tracker-python-baseline-20260911
- /tmp/lidar-tracker-cpp-evaluation-20260911
- /tmp/lidar-tracker-python-recorded-20260911
- /tmp/lidar-tracker-cpp-recorded-20260911

## 限制

未重新执行真实飞行、真实运动目标或 RViz GUI 视觉检查。
原先纯旋转、紧贴/交叉物体、复杂遮挡、倾斜地面和时钟同步方面的限制继续存在。
