---
tags:
  - 任务安排
  - RM
  - 自瞄
  - 移植
  - Phase1
created: 2026-08-28
related:
  - "[[步兵自瞄移植任务安排]]"
  - "[[Phase0-依赖包作用说明]]"
status: 已完成 ✅
---

# Phase 1 移植清单：最小包集提取与编译

> [!info] 本阶段目标
> 基于**实际源码验证**，把 FYT2024_vision 的 3 个包（rm_interfaces / rm_utils / rm_auto_aim）裁剪出最小集，拷入 `/home/nvidia/vision_ws/src`，先编译通过 `rm_interfaces` + `rm_utils`。
>
> 对应 [[步兵自瞄移植任务安排]] 的 **Phase 1**；环境依赖见 [[Phase0-依赖包作用说明]]。

---

## 一、需要移植的包（3 个）

### 1️⃣ rm_interfaces（消息定义）—— 裁剪到 10 个接口

原始 17 个接口，只保留自瞄闭环用到的 **10 个（9 msg + 1 srv）**：

| 接口 | 类型 | 用途 |
|------|------|------|
| Armor.msg | msg | 单块装甲板（编号/类型/距中心/3D 位姿） |
| Armors.msg | msg | 一帧检测结果（Header + Armor[]） |
| Target.msg | msg | 跟踪目标状态（位置/速度/yaw/半径/d_zc） |
| Measurement.msg | msg | EKF 测量（x/y/z/yaw） |
| GimbalCmd.msg | msg | 云台角度输出（pitch/yaw/distance/fire_advice） |
| DebugArmor.msg | msg | 调试：装甲板信息 |
| DebugArmors.msg | msg | 调试：装甲板数组 |
| DebugLight.msg | msg | 调试：灯条信息 |
| DebugLights.msg | msg | 调试：灯条数组 |
| SetMode.srv | srv | 切换模式（0 红自瞄 / 1 蓝自瞄 / 2-5 符） |

**裁剪掉（7 个，与自瞄无关）：**

- RuneTarget、Point2d、ChassisCmd、DebugRuneAngle、JudgeSystemData、OperatorCommand、SerialReceiveData

> [!warning] 数量修正
> 原方案写"裁剪 8 个"，按实际源码清单（10 保留 + 7 裁剪 = 17）应为 **7 个**。

**修改点：**

- `CMakeLists.txt`：`rosidl_generate_interfaces` 列表删掉上述 7 个
- `package.xml`：**不用改**（依赖 geometry_msgs + std_msgs 已满足）

### 2️⃣ rm_utils（工具库）—— 保留全部编译单元

> [!warning] 重要修正
> 原方案写"不搬 heartbeat / url_resolver"是**错的**。实测两个节点都调用 `HeartBeatPublisher::create()`，且 `armor_detector_node.cpp` 用 url_resolver 解析模型路径。**这俩必须留。**

需要（CMakeLists 编译单元 7 个 + logger 3 个）：

| 源文件 | 对应头文件 | 作用 |
|--------|-----------|------|
| src/math/utils.cpp | math/utils.hpp | 数学工具 |
| src/math/pnp_solver.cpp | math/pnp_solver.hpp | PnP 解算 |
| src/math/trajectory_compensator.cpp | math/trajectory_compensator.hpp | 弹道补偿 |
| src/math/manual_compensator.cpp | math/manual_compensator.hpp | 手动补偿 |
| src/math/extended_kalman_filter.cpp | math/extended_kalman_filter.hpp | EKF（ceres::Jet） |
| src/url_resolver.cpp | url_resolver.hpp | package:// 路径解析 |
| src/heartbeat.cpp | heartbeat.hpp | 心跳 |
| src/logger/{logger_impl,writer,logger_pool}.cpp | logger/ 全套 6 个头文件 | 日志 |
| 公共头 | common.hpp / assert.hpp | 公共定义 |

> 头文件路径均在 `include/rm_utils/` 下。

**不需要：**

- `particle_filter`（原本就不在 CMakeLists 编译列表里，`src/math/particle_filter.cpp` 及头文件**整个不搬**）

**CMakeLists 依赖**：`rclcpp`、`eigen3_cmake_module`、`Eigen3`、`geometry_msgs`、`OpenCV4`、`Ceres`、`rcpputils`、`fmt`

### 3️⃣ rm_auto_aim（主体，2 个子包全搬）

**armor_detector（识别 + PnP + BA）**

| 文件 | 说明 |
|------|------|
| src/armor_detector.cpp + .hpp | 灯条检测 + 配对主逻辑 |
| src/armor_detector_node.cpp + .hpp | ROS2 节点（订阅图像） |
| src/armor_pose_estimator.cpp + .hpp | PnP 解算 |
| src/light_corner_corrector.cpp + .hpp | PCA 角点矫正 |
| src/number_classifier.cpp + .hpp | LeNet-5 数字分类 |
| src/ba_solver.cpp + .hpp | G2O BA 优化 |
| src/graph_optimizer.cpp + .hpp | G2O 图优化构建 |
| include/armor_detector/types.hpp | 灯条/装甲板数据结构 |
| model/lenet.onnx + label.txt | 模型（**mlp.onnx 不用，不搬**） |
| cmake_modules/FindG2O.cmake | 已打补丁 ✅ |
| test/test_detector.cpp | 测试（可选，可留可删） |

**armor_solver（跟踪 + 预测 + 角度）**

| 文件 | 说明 |
|------|------|
| src/armor_solver.cpp + .hpp | 主逻辑（选板 + 弹道 + 角度计算） |
| src/armor_solver_node.cpp + .hpp | ROS2 节点 |
| src/armor_tracker.cpp + .hpp | EKF 多目标跟踪器 |
| include/armor_solver/motion_model.hpp | EKF 运动模型（ceres::Jet） |

---

## 二、依赖边界确认（已完成，无缺口）

**ROS 二进制依赖（都已装好 ✅）：**

- **公共**：rclcpp、rclcpp_components、geometry_msgs、std_msgs、message_filters、tf2、tf2_ros、tf2_geometry_msgs、visualization_msgs、angles、eigen、fmt、rcpputils
- **detector 特有**：sensor_msgs、cv_bridge、image_transport、image_transport_plugins、vision_opencv（apt 有包 3.2.1 ✅）、std_srvs

**系统依赖（Phase 0 已全装 ✅）：**

- OpenCV4、Ceres、G2O（/usr/local）、Sophus（/usr/local）、fmt、Eigen、TBB

> [!note] rosdep 不可用（raw.githubusercontent 被墙）→ 用手工依赖，上述已全部就绪

---

## 三、不移植的部分（明确排除）

- [ ] rm_interfaces 的 7 个无关消息
- [ ] rm_utils 的 particle_filter
- [ ] mlp.onnx（未使用）
- [ ] rune（符）、navigation（导航）、serial_driver（串口）、hardware_driver、robot_description、upstart、bringup
  - 说明：这些属于**运行/启动层**，Phase 1 只做构建层，启动配置留到 Phase 2/3 按需搬

---

## 四、Phase 1 执行步骤

> [!abstract] 确认后按序执行

- [ ] **1. 新建工作空间**
  ```bash
  mkdir -p /home/nvidia/vision_ws/src
  ```
- [ ] **2. 复制 rm_interfaces** → 删 7 个消息 + 改 CMakeLists
- [ ] **3. 复制 rm_utils**（全套编译单元）→ 删 particle_filter
- [ ] **4. 复制 rm_auto_aim**（两子包 + 模型 + FindG2O.cmake）
- [ ] **5. 编译 rm_interfaces + rm_utils**（只先编这两个）
  ```bash
  cd /home/nvidia/vision_ws
  colcon build --packages-select rm_interfaces rm_utils
  ```
- [ ] **6. 验收**（见下节）

> [!note] 说明
> `rm_auto_aim` 的**编译**属于 Phase 2（需先有 interfaces/utils 的 install 产物），Phase 1 只负责把它拷进 `src` 目录。

---

## 五、验收清单

- [x] `rm_interfaces`、`rm_utils` 两包 `colcon build` 编译通过，无报错
- [x] `ros2 interface show rm_interfaces/msg/GimbalCmd` 可用（能看到字段定义）
- [x] `source install/setup.bash` 后，`ros2 pkg prefix rm_interfaces` 与 `rm_utils` 能打印路径

---

## 六、与原方案对比（修正记录）

| 项 | 原方案 | 本次修正 |
|----|--------|----------|
| heartbeat / url_resolver | 标注"不搬" | ❌ 必须保留（两节点都在调用） |
| 裁剪接口数量 | "8 个" | ✅ 实际 7 个（10 保留 + 7 裁剪 = 17） |
| rm_auto_aim 搬入时机 | — | Phase 1 拷入 src，编译在 Phase 2 |
| particle_filter | 不搬 | 保持不搬（不在编译列表） |

---

> [!summary] 关联
> - 上游任务：[[步兵自瞄移植任务安排]]
> - 环境依赖：[[Phase0-依赖包作用说明]]
> - 下一阶段：Phase 2（编译自瞄节点，视频跑通识别）
