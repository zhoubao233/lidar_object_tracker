# 历史记录：迁移前的原实验

以下保留原始验证记录，路径和节点名属于旧工程；不是新包现场实测结果。

# 物体跟踪测试记录（2026-09-09）

正式代码与参数是本目录最终版本。测试过程没有修改原 M-detector 核心或向其历史图反馈新标签。
数据和逐帧 CSV 位于 `/home/zhoubao/APS/M-detector_diagnostics/20260909-object-tracking/`。

## 最终实际节点输出

启动本目录 `run.sh rviz:=true`，同时保持用户原 Gazebo、MAVROS、适配器和 M-detector 运行。
录制 30 秒墙钟时间，使用 `final_live.bag` 中实际发布的 tracks、dynamic 和原 frame_out 评分。
完成验证后已停止本次新增节点和 RViz，原节点保持运行。

- 匹配到 270 组输入与新节点输出，排除录制开头 1.5 秒后计分 255 组。
- 当前箱子目标 ID 保持为 1；这 255 帧均处于 MOVING 状态。
- 上部输入点的动态覆盖率 99.66%，下部 97.22%。
- 每帧算法耗时中位数 18.3 ms，P95 23.8 ms，最大 32.3 ms。
- 消息输出时数据年龄中位数 98 ms，P95 142.6 ms，最大 168 ms。
- skipped_pairs 最大为 0，未发生持续积压或跳过完整输入配对。

同时间戳、同一输入、同时具有两套输出的 253 帧对照：

| 点数指标 | 原 M-detector frame_out | 新物体跟踪 dynamic |
|---|---:|---:|
| 箱子上部覆盖率 | 44.66% | 99.67% |
| 箱子下部覆盖率 | 71.11% | 97.25% |

当前片段中，箱子扩展 ROI 以外的新动态点平均数为 0；这只说明此片段的结果，不能推广为复杂场景无误检。
所有被评分的新动态点均能在同时间戳输入中以 1 mm 内误差找到对应点。
MOVING 包含最长 0.8 秒沿用历史运动证据的状态，不能把 255/255 解读成每帧独立证明了运动。

## 已有录包复核

离线按原时间戳顺序执行最终 tracking_core，评分均排除开头 1.5 秒。

| 数据 | 总帧数 | 计分帧数 | MOVING / UNKNOWN | 上部覆盖率 | 目标 ID |
|---|---:|---:|---:|---:|---|
| 2 m/s：20260909-speed2-halves/live.bag | 213 | 197 | 197 / 0 | 99.45% | 1 |
| 1 m/s：20260909-rejection/holdout_live.bag | 326 | 311 | 296 / 15 | 94.77% | 1 |
| 1 m/s：20260909-detection/live.bag | 418 | 403 | 387 / 16 | 95.46% | 1 |

两个 1 m/s 片段仍有 UNKNOWN 帧，不承诺始终保持红色。

## 评分定义

只在评价脚本中使用 Gazebo iris 和 unit_box 位姿以及 1 m 箱体几何，算法本身不读取这些真值。
评价时考虑 iris 模型到 base_link 的 0.194923 m 高度差。
箱体 ROI 为每轴中心 ±0.54 m，排除箱底 0.15 m 以下区域，以箱底以上 0.5 m 划分上下部。
ROI 外动态点统计采用每轴 ±0.65 m 的容差区域。
覆盖率是“该区域输入点中被标为动态的比例”，并非语义检测准确率，也不是隐藏表面的完整率。

## 行为和接口检查

`python3 -m unittest test_tracking -v` 共 9 项通过：

1. 平移物体保持同一目标编号，输出覆盖整个可见采样表面；停止后恢复 STATIC。
2. 静止平面的可见范围变化不会触发 MOVING。
3. 观察者平移、旋转，经里程计补偿后静止物体仍保持静止。
4. 丢失后只输出短时预测框，不生成虚构点云，超过期限删除。
5. 仿真时间回退清空旧轨迹。
6. 明显位姿跳变清空旧运动证据。
7. 突然合并成明显更大的候选簇不会直接继承原运动分类。
8. 空点云读写可用。
9. 输入时间戳或坐标系不一致时拒绝处理。

已验证 Bash 语法、启动文件解析、实时 ROS 消息和 RViz 进程启动。FAST 输入模式只做启动参数解析检查。
本轮未验证纯旋转分类、接触/交叉物体、复杂遮挡、倾斜地面和真实扫描畸变。

## 复核命令

```bash
source /opt/ros/noetic/setup.bash
cd /home/zhoubao/APS/M-detector/tools/object_tracking_test
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 evaluate_bag.py \
  /home/zhoubao/APS/M-detector_diagnostics/20260909-object-tracking/final_live.bag \
  --recorded --output /tmp/object_tracker_review
```

以上是独立物体级跟踪的结果，不是 M-detector 原点级算法改动后的结果。
