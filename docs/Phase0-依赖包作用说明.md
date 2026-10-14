---
tags:
  - 任务安排
  - RM
  - 自瞄
  - 移植
  - 依赖说明
  - Phase0
created: 2026-08-26
related:
  - "[[步兵自瞄移植任务安排]]"
status: 已完成 ✅
---

# Phase 0 依赖安装与作用说明

> [!info] 适用场景
> 本文对应 [[步兵自瞄移植任务安排]] 的 **Phase 0 — 环境准备**。目标是让你明白**每个依赖是干什么的、怎么装、装完怎么确认**。全部完成后即可进入 Phase 1 编译工作空间。

---

## 0. 总览：装什么、按什么顺序

```text
基础工具（4 个）→ 数学/优化库（6 个）→ ROS2 本体 → ROS 图像/坐标包（4 个）
```

> [!warning] 前提
> 目标机器为 **Ubuntu 22.04 (jammy) / aarch64**。以下命令均需 `sudo`。ROS2 相关包必须在 ROS2 源（清华/阿里云镜像）配置好之后才能装。

---

## 一、基础工具链（环境骨架，4 个）

| 工具                 | 作用（在自瞄里的角色）                             | 安装                                           | 检查                 |
| ------------------ | --------------------------------------- | -------------------------------------------- | ------------------ |
| **cmake**          | C++ 构建工具，编译自瞄核心节点（detector/solver）的必备工具 | `sudo apt install cmake`                     | `cmake --version`  |
| **pip3**           | Python 包管理器，给相机驱动/辅助脚本装 Python 依赖       | `sudo apt install python3-pip`               | `pip3 --version`   |
| **curl**           | 命令行下载工具，下载源码/模型、测网络连通性                  | `sudo apt install curl`                      | `curl --version`   |
| **python3-rosdep** | ROS 依赖解析器，`rosdep install` 自动补齐编译时缺失的依赖 | 配好 ROS2 源后：`sudo apt install python3-rosdep` | `rosdep --version` |

**rosdep 首次初始化**（装完立刻做）：

```bash
sudo rosdep init
rosdep update
```

---

## 二、核心数学/优化库（算法的"大脑"，6 个）

| 依赖                | 作用（在自瞄里的角色）                                    | 安装                               | 检查                             |
| ----------------- | ---------------------------------------------- | -------------------------------- | ------------------------------ |
| **libeigen3-dev** | 线性代数模板库，所有矩阵运算/几何变换的基础，几乎每个位姿计算都要用它            | `sudo apt install libeigen3-dev` | `ls /usr/include/eigen3/Eigen` |
| **libsophus-dev** | 李群/李代数库（SE(3)/SO(3)），云台角解算、位姿插值，写旋转比裸 Eigen 优雅 | `sudo apt install libsophus-dev` | `ls /usr/include/sophus`       |
| **libceres-dev**  | 非线性优化库，PnP 重投影误差最小化、多帧融合位姿平滑                   | `sudo apt install libceres-dev`  | `ls /usr/include/ceres`        |
| **libg2o-dev**    | 图优化库，多观测点融合位姿估计（BA 优化用）                        | `sudo apt install libg2o-dev`    | `ls /usr/include/g2o`          |
| **libtbb-dev**    | Intel 并行计算库，加速图像处理循环（多帧并行、像素级操作）               | `sudo apt install libtbb-dev`    | `ls /usr/include/oneapi/tbb`   |
| **libfmt-dev**    | 现代化字符串格式化库，负责日志输出与调试信息组装                       | `sudo apt install libfmt-dev`    | `ls /usr/include/fmt`          |

> [!note] 一句记忆
> Eigen 算矩阵、Sophus 转旋转、Ceres/G2O 做优化、TBB 加速图像、fmt 输出日志。Ceres 与 G2O 功能有重叠，可二选一，但都装不冲突。

---

## 三、ROS 图像/坐标包（数据搬运工，4 个）

> [!note] 这些包通常在 `ros-humble-desktop` 里已带，单独列出是确保齐全；也可用一条命令补装。

| ROS 包 | 作用（在自瞄里的角色） | 安装 | 检查 |
|--------|------|------|------|
| **cv_bridge** | ROS Image 消息 ↔ OpenCV Mat 互转（相机回调 → 算法处理） | `sudo apt install ros-humble-cv-bridge` | `ros2 pkg prefix cv_bridge` |
| **image_transport** | 图像压缩/解压传输，省带宽（常用 compressed 订阅） | `sudo apt install ros-humble-image-transport` | `ros2 pkg prefix image_transport` |
| **tf2** | 坐标变换管理（相机→云台→发射机构，以及世界坐标系目标跟踪） | `sudo apt install ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs` | `ros2 pkg prefix tf2` |
| **message_filters** | 多传感器时间戳同步（相机 + 云台反馈精确对齐，避免瞄准偏差） | `sudo apt install ros-humble-message-filters` | `ros2 pkg prefix message_filters` |

**ROS2 本体**（在配好镜像源后安装）：

```bash
sudo apt install ros-humble-desktop
```

---

## 四、备选方案：源码编译（sophus / g2o 若 apt 装不上）

> [!warning] 备注
> Ubuntu 官方源不一定带 `libsophus-dev` / `libg2o-dev`。若 `apt install` 报 "Unable to locate package"，走源码编译：

```bash
# Sophus
sudo apt install build-essential cmake libeigen3-dev
git clone https://github.com/strasdat/Sophus.git /tmp/Sophus
cd /tmp/Sophus && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install

# g2o（如需 BA 优化；也可临时用 use_ba_=false 绕过）
git clone https://github.com/RainerKuemmerle/g2o.git /tmp/g2o
cd /tmp/g2o && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

> 若 ROS2 源整体不通（SSL/被墙）：切清华/阿里云镜像 → 仍不行用离线 `.deb`（[ROS2 官网](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 手动 `dpkg -i`）→ 最后备选 conda（注意 ABI 兼容问题，能不用就不用）。

---

## 五、最后检查安装（验收）

全部装完后，复制下面命令逐条执行，**全部通过才算 Phase 0 完成**：

```bash
# ── 1. 基础工具 ─────────────────────────────
cmake --version          # ≥3.22
pip3 --version
curl --version
rosdep --version

# ── 2. ROS2 本体 ────────────────────────────
ros2 --version           # 显示 Humble
colcon --version

# ── 3. 数学/优化库头文件存在 ────────────────
ls /usr/include/eigen3/Eigen
ls /usr/include/sophus
ls /usr/include/ceres
ls /usr/include/g2o
ls /usr/include/oneapi/tbb
ls /usr/include/fmt

# ── 4. ROS 包可被找到 ───────────────────────
ros2 pkg prefix cv_bridge
ros2 pkg prefix image_transport
ros2 pkg prefix tf2
ros2 pkg prefix message_filters
```

> [!tip] 检查要点
> - 版本类命令：有版本号输出即 OK（尤其 `ros2 --version` 要显示 Humble）。
> - `ls` 头文件类：列出文件即 OK，报 "No such file" 说明该包没装上。
> - `ros2 pkg prefix`：能打印路径即 OK。

---

> [!summary] 关联
> - 上游任务：[[步兵自瞄移植任务安排]]
> - 本说明对应主任务笔记 **Phase 0** 的所有安装项，验收通过后再进入 Phase 1。
