# bubin_test — 步兵自瞄移植工作区备份

Jetson Orin NX 上 `~/vision_ws` 工作区的备份（源码 + 配置 + 阶段文档 + 调试日志）。

## 来源与血缘

代码源自开源项目 **FYT2024_vision**
（[CSU-FYT-Vision/FYT2024_vision](https://github.com/CSU-FYT-Vision/FYT2024_vision)，Apache-2.0），
其自瞄算法又基于 **rm_vision**（Chen Jun / FaterYU，MIT）。

本工作区在 FYT2024_vision 基础上做了裁剪：只保留自瞄链路
（`armor_detector` / `armor_solver` / `rm_utils` / `rm_interfaces`），
新增 `rm_video_player` 用于视频仿真，移除了打符、串口、导航、机器人描述等包。

## 目录

| 路径 | 说明 |
|---|---|
| `src/` | 5 个 ROS2 功能包 |
| `config/` | 节点参数与相机内参 |
| `launch/` | 组件容器启动文件（已弃用，现用 run_armor.sh） |
| `calib/` | 相机标定采集/计算脚本（calib_imgs 为空，标定未完成） |
| `docs/` | 阶段文档（Phase0-3、参数手册、电控接口草案） |
| `logs/` | 调试探针日志 |
| `run_armor.sh` | 一键启停脚本 |

## 运行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash run_armor.sh start    # 起 TF + video_player + detector + solver
bash run_armor.sh status
bash run_armor.sh stop
```

## 环境

Jetson Orin NX / L4T R36.4.4 (JetPack 6.2) / Ubuntu 22.04 / ROS2 Humble
依赖：OpenCV 4.5.4、Eigen3、Sophus、Ceres、g2o(/usr/local)、fmt、TBB

## 说明

本备份**不含测试视频**（`input.avi` / `output_result.avi` / `cam_recording.mp4`）
与 `build/`、`install/`、`log/` 等 colcon 构建产物；视频在设备本地 `~/vision_ws/` 下，构建产物可随时重新编译生成。
