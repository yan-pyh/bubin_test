---
tags:
  - 任务安排
  - RM
  - 自瞄
  - 移植
  - Phase2
  - 实现细节
created: 2026-08-28
status: 已完成 ✅
related:
  - "[[步兵自瞄移植任务安排]]"
  - "[[Phase0-依赖包作用说明]]"
  - "[[Phase1-移植清单]]"
---

# Phase 2 实现细节：编译自瞄节点，视频跑通识别

> [!done] 阶段状态
> **✅ 已完成**：detector 已能在测试视频上画出装甲板框并持续发布 3D 位姿（`/armor_detector/armors` @30Hz）。

---

## 0. 阶段目标（回顾）

对应 [[步兵自瞄移植任务安排]] 的 **Phase 2**：

```text
colcon build 编译 armor_detector + armor_solver
   → 用 video_player 播放视频（模拟相机输入）
   → 验证 detector 识别出装甲板并发布 /armor_detector/armors
   → 查看 result_img 可视化
```

完成标准：**detector 能在视频上画出并发布装甲板位姿** —— 已达成 ✅

---

## 一、工作空间结构（完成态）

```text
/home/nvidia/vision_ws/
├── src/
│   ├── rm_interfaces/      # 10 个接口（保留子集）
│   ├── rm_utils/           # 裁剪后工具库
│   ├── rm_auto_aim/        # armor_detector + armor_solver + 元包
│   └── rm_video_player/    # 新增精简视频播放包（模拟相机）
├── config/
│   ├── camera_info.yaml
│   ├── video_player_params.yaml
│   └── armor_detector_params.yaml
├── launch/
│   └── test_armor.launch.py
├── input.avi               # 1280×1024, 351 帧, 30fps 测试视频
└── output_result.avi       # 265 帧识别结果录制
```

> [!note] 与方案差异
> 新增 `rm_video_player` 包：原方案写"用自带 video_player"，实际是从本地仓库的 `rm_camera_driver` 提取，独立成一个精简包。

---

## 二、代码来源与移植动作

> 来源仓库：`/home/nvidia/bubin_det/FYT2024_vision/`（**本地已 clone**，全程不依赖 github clone）。

### 2.1 rm_auto_aim（主体，整体复制）

| 子包 | 全量内容 |
|------|---------|
| **armor_detector** | Detector（预处理/灯条/配对）+ NumberClassifier（LeNet-5 ONNX）+ ArmorPoseEstimator（PnP）+ LightCornerCorrector（PCA）+ BaSolver（G2O BA） |
| **armor_solver** | Tracker（EKF 10 维）+ Solver（预测/选板/弹道补偿）—— 本阶段**只编译不启动** |

- `model/` 只保留 `lenet.onnx` + `label.txt`
- 新增补丁 `cmake_modules/FindG2O.cmake`：过滤 `-NOTFOUND` 条目。
  - 背景：CMake 的 `IF()` 会把以 `-NOTFOUND` 结尾的字符串判为 false，导致 `solver_cholmod` / `incremental` 缺失时链接失败；补丁后改为显式过滤修复。

### 2.2 rm_interfaces（消息子集）

- 保留 **10 个**：msg `Armor/Armors/Target/Measurement/GimbalCmd/DebugArmor(s)/DebugLight(s)` + srv `SetMode`
- 不搬：`RuneTarget`、`ChassisCmd`、`JudgeSystemData` 等无关接口
- `CMakeLists.txt` / `package.xml` 同步裁剪

### 2.3 rm_utils（工具裁剪）

- **删除** `particle_filter`
- **保留** `heartbeat`、`url_resolver` —— 重要：detector 加载 `package://armor_detector/model/lenet.onnx` 走 url_resolver，**不能删**
- **保留** `math/pnp_solver`、`math/extended_kalman_filter`、`math/trajectory_compensator`、`math/utils`、`logger/`

### 2.4 rm_video_player（新增精简包）

- 从原 `rm_camera_driver` 提取 `video_player.cpp`，注册节点 `fyt::camera_driver::VideoPlayerNode`
- 职责：按 30fps 循环发布 `image_raw`(BGR8) + `camera_info`，**模拟相机输入**，供无相机阶段调试

---

## 三、编译过程与关键依赖处理

| 项 | 处理 |
|----|------|
| ROS2 Humble | 镜像源安装（packages.ros.org SSL 证书不匹配 → 清华/阿里云镜像） |
| OpenCV 4.5.4 | 已带，含 dnn 模块 ✅ |
| g2o | apt 版缺失 → codeload.github.com 下载源码编译，`-DG2O_USE_OPENGL=OFF` |
| Sophus / Ceres | Sophus 源码编译；Ceres 用于 EKF 自动微分（Jet） |
| colcon / vcstool | `pip3 --user` 安装（`python3-colcon-common-extensions` 不在 Ubuntu 源） |
| empy | **必须 pin 到 `empy==3.3.4`**：pip 装 colcon 带入的 4.x 会破坏 rosidl（`AttributeError: module 'em' has no attribute 'BUFFERED_OPT'`） |
| ros-humble-camera-info-manager | apt 补装（否则 `camera_info_manager.hpp: No such file`），**装后须清 build 缓存重编** |

**编译命令**（在 `vision_ws` 下）：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm_interfaces rm_utils rm_auto_aim rm_video_player
```

---

## 四、运行链路与关键配置

### 4.1 数据流

```text
input.avi ─► video_player ─► /image_raw + /camera_info
                                  │
                                  ▼
                        armor_detector_node
                         │  lookupTransform(odom → camera_optical_frame)
                         ▼
                /armor_detector/armors（30Hz, 3D 位姿）
                         │
                         ▼
                 result_img（可视化，可录制）
```

### 4.2 TF 依赖（必配）

`armor_detector_node` 的 imageCallback 会先做 `lookupTransform(odom, frame_id)`，**失败直接 return**。无电控环境下用静态 TF 模拟：

```bash
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.05 0.0 0.0 0.0 odom camera_optical_frame
```

### 4.3 关键参数（config/armor_detector_params.yaml）

| 参数 | 值 | 说明 |
|------|----|------|
| binary_thres | 90 | 二值化阈值 |
| use_pca | true | PCA 角点矫正 |
| use_ba | true | G2O BA 优化 |
| detect_color | 0 | 红方装甲板 |
| classifier_threshold | 0.7 | 数字分类置信度 |
| ignore_classes | ["negative"] | 忽略非数字类别 |

### 4.4 camera_info 加载（关键坑）

> [!warning]
> `camera_info_url` **必须是全路径且带 `file:///` 前缀**：
> `file:///home/nvidia/vision_ws/config/camera_info.yaml`
>
> 纯路径不被 `validateURL` 接受 → 内参 K 为空 Mat → 构造 PnPSolver 时崩溃（`cv::Exception: total()==0`）。

### 4.5 启动方式（最终采用独立节点）

> [!note]
> 初版 `launch/test_armor.launch.py`（组件容器方式）中 armor_detector **静默加载失败**，排查后改为**独立节点**方式，稳定可靠。

```bash
source install/setup.bash
# 1) 静态 TF
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.05 0.0 0.0 0.0 odom camera_optical_frame &
# 2) 视频源
ros2 run rm_video_player video_player_node --ros-args \
  -p path:=/home/nvidia/vision_ws/input.avi \
  -p frame_rate:=30 \
  -p camera_info_url:=file:///home/nvidia/vision_ws/config/camera_info.yaml
# 3) 检测
ros2 run armor_detector armor_detector_node
```

---

## 五、验证结果（已达成 ✅）

### 5.1 功能验证

- `/armor_detector/armors` 以 **30Hz** 持续发布，内容含装甲板 **3D 位姿**（识别到 `number='3'` 小装甲板）
- `result_img` 绘制正常：
  - 逐帧像素差异验证：**9/9 帧有标注**，检出 **19948 个标注像素**
  - 输出录制视频 `output_result.avi`（**265 帧**）

### 5.2 性能实测（完整 detector 链路）

> 链路包含：分类 + PnP + 绘制 + 发布（不含追踪）

| 项 | 值 |
|----|----|
| 分辨率/帧率 | 1280×1024 @30fps |
| 处理延迟（中位 / 平均 / 最大） | **14.3ms / 14.7ms / 23.2ms** |
| 折算处理能力 | ≈68fps（30fps 输入下有富余 ✅） |
| 纯像素基准（灰度+二值+轮廓） | 仅 1.89ms |

> [!tip] 结论
> 耗时大头在**逻辑/分类/PnP**，而非像素操作 → 后续优化方向是分类器与 PnP，而非图像预处理。

---

## 六、踩坑记录（调试备忘）

| 问题 | 根因 | 修复 |
|------|------|------|
| `cv::Exception total()==0` 崩溃 | `camera_info_url` 纯路径不被 validateURL 接受，内参 K 空 | 加 `file:///` 前缀 |
| TF 缺失刷屏 | 无 `odom → camera_optical_frame` | 静态 TF 发布 |
| launch 组件容器 detector 不加载 | 容器方式静默失败 | 改 `ros2 run` 独立节点 |
| empy 编译炸 | empy 4.x 不兼容 rosidl | `pip3 install empy==3.3.4` |
| camera_info_manager 缺头文件 | 包未装 | apt 安装后清缓存重编 |
| github git 协议 TLS 中断 | 网络受限 | codeload tarball / apt 镜像 |

---

## 七、关键经验总结

1. **相机内参必须完整**：`camera_info_url` 带 `file:///` 前缀，否则内参 K 空、PnP 直接崩溃 —— 这是最隐蔽的坑。
2. **TF 是硬依赖**：detector 的 imageCallback 以 `lookupTransform` 为前置条件，无电控先用静态 TF 顶替。
3. **组件容器 vs 独立节点**：组件容器静默失败难排查，独立节点方式更稳，Phase 3 沿用。
4. **empy 版本锁死**：ROS2 Humble + colcon 环境必须 `empy==3.3.4`，装 4.x 必炸 rosidl。
5. **性能余量充足**：68fps 处理能力远超 30fps 输入，给 Phase 4 真实相机调参留了空间。

---

> [!summary] 关联
> - 上游任务：[[步兵自瞄移植任务安排]]
> - 前置：[[Phase0-依赖包作用说明]] → [[Phase1-移植清单]]
> - 下一阶段：Phase 3（接 solver + 静态 TF，输出角度数据）
