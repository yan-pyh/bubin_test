---
tags:
  - 任务安排
  - RM
  - 自瞄
  - 移植
  - Phase3
  - 实现细节
created: 2026-09-02
status: 已完成 ✅
related:
  - "[[步兵自瞄移植任务安排]]"
  - "[[Phase0-依赖包作用说明]]"
  - "[[Phase1-移植清单]]"
  - "[[Phase2-自瞄实现]]"
  - "[[参数手册]]"
---

# Phase 3 实现细节：接入 solver，跑通角度闭环

> [!done] 阶段状态
> **✅ 已完成**：`识别 → 跟踪 → 预测 → 云台角度输出` 完整闭环跑通，用视频流替代真实云台输出 `yaw/pitch`。

---

## 一、阶段目标（回顾）

对应 [[步兵自瞄移植任务安排]] 的 **Phase 3**：

- 接入 `armor_solver`（EKF 多目标跟踪 + 预测 + 弹道补偿）
- 解决 TF 依赖（无电控 → 静态 TF 模拟云台位姿）
- 输出 `armor_solver/target`（跟踪状态）与 `armor_solver/cmd_gimbal`（yaw/pitch/distance/fire_advice）

---

## 二、核心实现（实现思路与关键结论）

### 1. 先读源码、摸清接口（未动手前）

> 在本地备份 `/Users/yan/Codes/bubin_code/src/rm_auto_aim/armor_solver/` 读完三份核心代码，明确接口边界。

| 文件 | 关键结论 |
|------|---------|
| `armor_solver_node.cpp` | 订阅 `armor_detector/armors`（带 **tf2 message_filter**）；发布 `target` / `cmd_gimbal` / `measurement` / `marker`；**250Hz timer 输出** |
| `armor_solver.cpp` | `solve()` 内部需要 **`odom → gimbal_link`** 的 TF 查云台姿态；弹道补偿参数 |
| `armor_tracker.cpp` | 状态机 `LOST → DETECTING → TRACKING / TEMP_LOST`；EKF 10 维状态 |

**关键认知 —— solver 不是收到 armor 就输出角度**，完整链路是：

```text
灯条颜色匹配 → 配对成装甲板 → PnP 出 3D 位姿（Phase 2 已通）
  → solver 把位姿从相机系 transform 到 odom 系
  → 异常过滤（abs(z) > 2）
  → tracker 连续 tracking_thres 帧匹配 → TRACKING
  → 才真正输出角度
```

> 任一环节断裂都会表现为 `tracking=false`，这是定位问题的总线索。

### 2. 配置 solver 参数（用默认值即可）

新建 `/home/nvidia/vision_ws/config/armor_solver_params.yaml`，直接参考 FYT2024 原版默认：

| 分组 | 参数（默认值） |
|------|----------------|
| **EKF** | `sigma2_q_x/y/z=0.05`、`sigma2_q_yaw=1.0`、`sigma2_q_r=80`、`r_x/y/z≈4e-4`、`r_yaw=5e-3` |
| **Tracker** | `max_match_distance=0.5`、`tracking_thres=2`、`lost_time_thres=1.0` |
| **Solver** | `bullet_speed=25`、`gravity=10`、`resistance=0.092`、`compensator_type="ideal"`、`iteration_times=20` |
| **其他** | `debug: true`、`target_frame: odom` |

> 参数调优方法见 [[参数手册]] B1 节；当前阶段用默认值即可跑通。

### 3. 搭建 TF 链路（最大的坑在这）

启动两条静态 TF：

```bash
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.05 0.0 0.0 0.0 odom gimbal_link
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.05 -1.5708 0.0 -1.5708 odom camera_optical_frame
```

> [!warning] 坑① 相机光学系旋转（最隐蔽）
> 只发平移不带旋转时，相机深度 **z≈2.7m 直接映射成 odom 的 z（高度）**，solver 内部 `abs(z)>2` 的异常过滤会把**所有**装甲板删光 → tracker 永远 `LOST`。
>
> 用 numpy 反解出正确 RPY = `(roll=-90°, pitch=0, yaw=-90°)`，使得坐标映射为：`cam_z→odom_x`、`cam_x→-odom_y`、`cam_y→-odom_z`。**这步不能省。**

### 4. 启动 solver 并验证

```bash
ros2 run armor_solver armor_solver_node --ros-args --params-file /home/nvidia/vision_ws/config/armor_solver_params.yaml
```

验证路径（每步都有对应话题证据）：

| # | 验证点 | 证据 |
|---|--------|------|
| 1 | `/armor_detector/armors` 非空 | 裸订阅验证 ✅ |
| 2 | TF `odom→camera` 对消息时间戳可用 | tf2 buffer 复现验证 ✅ |
| 3 | solver 日志进入跟踪 | `Tracker state: TRACKING 3` ✅ |
| 4 | `target` 状态 | `tracking=true, id='3', armors_num=4` ✅ |
| 5 | `cmd_gimbal` 角度 | `yaw≈7.9°, pitch≈-1.0°, distance≈2.94m` @250Hz ✅ |

### 5. 排查过程还原（两次关键诊断）

| 层 | 现象 | 动作 | 结论 |
|----|------|------|------|
| 现象 | solver 一直 `tracking=false`、`measurement` 零发布 | — | — |
| 第一层 | 怀疑视频没目标 | 用正确 QoS 直接订阅 detector | 检测其实正常（RED 模式 **658/679 帧非空**）→ 问题锁定在 solver 内部 |
| 第二层 | 怀疑消息/TF | 用 rclpy 复现 solver 的订阅 + tf2 transform | 全部成功 → 排除消息与 TF 传输 → 定位到 **solver 代码 `abs(z)>2` 过滤** → 最终锁定 **TF 缺旋转导致 z 坐标异常** |
| 附带发现 | — | — | detector 初始化**固定检测 RED**（`detect_color` 参数不生效），需用 `set_mode {mode:0}` 服务切换 |

### 6. 交付固化

- **一键脚本** `run_armor.sh {start|stop|status}`：起停 2 条静态 TF + video_player + detector + solver + 自动切 RED
- **配置**同步到本地备份 `/Users/yan/Codes/bubin_code/config/`
- **文档**更新主任务 Phase 3 为已完成并记录踩坑

### 一句话总结流程

> **读源码定接口 → 配默认参数 → 建 TF 链路（关键：相机光学系旋转）→ 启动验证 → 按"检测→TF→solver"三层逐级定位 `tracking=false` 根因 → 修复后闭环输出角度 → 固化脚本与文档**

---

## 三、部署流程（照着敲版）

### 前置：确认工作空间已编译

```bash
ls /home/nvidia/vision_ws/install/   # 应有 armor_detector, armor_solver, rm_video_player, rm_interfaces, rm_utils
```

每次新开终端都要 source：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/vision_ws/install/setup.bash
```

### 第 1 步：检查配置文件（4 个，缺一不可）

```bash
ls /home/nvidia/vision_ws/config/
```

| 文件 | 作用 | 关键点 |
|------|------|--------|
| `video_player_params.yaml` | 视频源 | `camera_info_url: file:///home/nvidia/vision_ws/config/camera_info.yaml`（必须 file:// 全路径，否则 PnP 内参空直接崩）、`path` 指向 input.avi、`keep_looping: true` |
| `camera_info.yaml` | 相机内参 | fx≈1807, fy≈1806, cx≈711, cy≈562，畸变全 0 |
| `armor_detector_params.yaml` | 检测参数 | `binary_thres: 90`、颜色相关、classifier 阈值等 |
| `armor_solver_params.yaml` | solver 参数 | EKF / tracker / `bullet_speed=25` 等（Phase 3 新建） |

### 第 2 步：发布静态 TF（两条，顺序无要求）

```bash
# ① odom → gimbal_link：云台在相机安装位（当前与相机同姿态）
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.05 0.0 0.0 0.0 odom gimbal_link

# ② odom → camera_optical_frame：必须带光学系旋转（-1.5708 = -90°）
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.05 -1.5708 0.0 -1.5708 odom camera_optical_frame
```

> ⚠️ **第 ② 条是最大坑**：`yaw pitch roll = -90° 0° -90°` 不能省（详见二.3 坑①）。

**验证：**

```bash
ros2 run tf2_ros tf2_echo odom camera_optical_frame
# 应看到 RPY (radian) [-1.571, -0.000, -1.571]，Translation [0.100, 0.000, 0.050]
```

### 第 3 步：启动 video_player（视频源）

```bash
ros2 run rm_video_player video_player_node \
  --ros-args --params-file /home/nvidia/vision_ws/config/video_player_params.yaml
```

**验证：**

```bash
ros2 topic hz /image_raw          # 应 ~27-30Hz
ros2 topic hz /camera_info        # 应 ~30Hz
```

### 第 4 步：启动 armor_detector（检测 + PnP）

```bash
ros2 run armor_detector armor_detector_node \
  --ros-args --params-file /home/nvidia/vision_ws/config/armor_detector_params.yaml
```

**验证：**

```bash
ros2 topic hz /armor_detector/armors        # ~30Hz
# 注意要用 best_effort QoS
ros2 topic echo /armor_detector/armors --qos-reliability best_effort --once
# 应看到 number: '3' 等字段（视频里是红色装甲板）
```

### 第 5 步：切到 RED 模式（关键，容易漏）

> [!warning]
> detector 初始化**固定检测红色**，`detect_color` 参数不生效，必须走服务切换。测试视频是红色装甲板，所以要切 RED：
> 漏了这步 → detector 一直检测不到（切错 BLUE 的话 45 秒 0 检测），solver 永远空转。

```bash
ros2 service call /armor_detector/set_mode rm_interfaces/srv/SetMode "{mode: 0}"
# mode: 0 = RED（AUTO_AIM_RED）, 1 = BLUE
```

**验证**：重看 armors 话题，应出现大量 `number: '3'` 的非空消息。

### 第 6 步：启动 armor_solver（跟踪 + 角度输出）

```bash
ros2 run armor_solver armor_solver_node \
  --ros-args --params-file /home/nvidia/vision_ws/config/armor_solver_params.yaml
```

### 第 7 步：完整验证（闭环是否真正跑通）

```bash
# ① 看 solver 日志，应出现跟踪状态切换
tail -f ~/fyt2024-log/<日期>/armor_solver_*.log.md
#   关键行: [DEBUG] Tracker state: TRACKING 3   （id 是识别到的数字）

# ② target：tracking 必须为 true
ros2 topic echo /armor_solver/target --once
#   tracking: true / id: '3' / armors_num: 4 / position(x,y,z)

# ③ cmd_gimbal：有真实角度（250Hz）
ros2 topic echo /armor_solver/cmd_gimbal --once
#   yaw ≈ 6~10° / pitch ≈ -1~-2° / distance ≈ 2.7~3m

# ④ measurement：EKF 持续更新（~16Hz，若 0 则 tracker 没跟上）
ros2 topic hz /armor_solver/measurement
```

**判定标准**：`tracking: true` + `cmd_gimbal` 的 `yaw/pitch/distance` 都是非零合理值 = Phase 3 闭环跑通。

### 可选：一键脚本（已固化）

以上启动步骤已封装进 `run_armor.sh`（内容 = 第 2~6 步的 nohup 后台化 + 自动 set_mode RED）：

```bash
bash /home/nvidia/vision_ws/run_armor.sh start    # 一键启动全部
bash /home/nvidia/vision_ws/run_armor.sh status   # 查进程+话题
bash /home/nvidia/vision_ws/run_armor.sh stop     # 全部停止
```

---

## 四、排查速查表（跑不通时按序查）

| 现象 | 原因 | 检查 |
|------|------|------|
| `armors` 一直空 | 没切 RED / 切错颜色 | 第 5 步 set_mode，`topic echo` 看非空 |
| `target` 一直 `tracking=false`、`measurement` 0 | TF 缺旋转 → z>2 被过滤 | 第 2 步 tf2_echo 看 RPY 是否 -90° |
| solver 日志大量 `dropping message queue full` | TF 未就绪时消息积压 | 确认两条静态 TF 已发布再起 solver |
| `cmd_gimbal` 全 0 / distance=-1 | 没进入 TRACKING | 先看 `target` |
| detector 崩溃 Mat 空 | camera_info URL 无效 | video_player_params 的 `file://` 全路径 |

---

## 五、关键经验总结

1. **最隐蔽的组合原因**：solver 的 `abs(z)>2` 异常过滤 + 相机系→世界系旋转缺失 —— 这是"检测正常但 solver 无输出"的典型根因。
2. **三层定位法**：遇到 `tracking=false` 按 **检测 → TF → solver** 顺序逐级排除，每层用独立工具验证（裸订阅 / tf2 buffer / 日志），别在猜测上浪费精力。
3. **detector 颜色是写死的**：`detect_color` 参数不生效，必须走 `set_mode` 服务，容易漏。
4. **QoS 陷阱**：读 detector 话题要加 `--qos-reliability best_effort`，否则可能订阅不到。
5. **TF 先于 solver 启动**：避免消息积压导致 `dropping message queue full`。

---

> [!summary] 关联
> - 上游任务：[[步兵自瞄移植任务安排]]
> - 前置阶段：[[Phase0-依赖包作用说明]] / [[Phase1-移植清单]] / [[Phase2-自瞄实现]]
> - 参数参考：[[参数手册]]（B1 solver 参数）
> - 下一阶段：Phase 4（接入真实相机，标定内参）
