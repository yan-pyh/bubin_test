#!/bin/bash
# RoboMaster auto-aim full pipeline launcher (Phase 0-3)
# Usage: ./run_armor.sh [start|stop|status]
set +u
WS=/home/nvidia/vision_ws
LOG=$WS/logs
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
mkdir -p $LOG

start() {
  echo "Starting auto-aim pipeline..."
  # Static TF: odom -> gimbal_link (gimbal at camera mount, same pose as camera for now)
  setsid nohup ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.05 0.0 0.0 0.0 odom gimbal_link \
    > $LOG/tf_gimbal.log 2>&1 < /dev/null &
  # Static TF: odom -> camera_optical_frame (with correct optical rotation yaw=-90, roll=-90)
  setsid nohup ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.05 -1.5708 0.0 -1.5708 odom camera_optical_frame \
    > $LOG/tf_camera.log 2>&1 < /dev/null &
  sleep 1
  setsid nohup ros2 run rm_video_player video_player_node --ros-args --params-file $WS/config/video_player_params.yaml \
    > $LOG/video_player.log 2>&1 < /dev/null &
  setsid nohup ros2 run armor_detector armor_detector_node --ros-args --params-file $WS/config/armor_detector_params.yaml \
    > $LOG/armor_detector.log 2>&1 < /dev/null &
  setsid nohup ros2 run armor_solver armor_solver_node --ros-args --params-file $WS/config/armor_solver_params.yaml \
    > $LOG/armor_solver.log 2>&1 < /dev/null &
  sleep 3
  # Switch detector to RED mode (video contains red armor plates)
  ros2 service call /armor_detector/set_mode rm_interfaces/srv/SetMode "{mode: 0}" > /dev/null 2>&1 || true
  echo "Started. Use ./run_armor.sh status to check."
}

stop() {
  echo "Stopping auto-aim pipeline..."
  pkill -f "install/rm_video_player" 2>/dev/null
  pkill -f "install/armor_detector" 2>/dev/null
  pkill -f "install/armor_solver" 2>/dev/null
  pkill -f "ros2 run tf2_ros static_transform_publisher" 2>/dev/null
  pkill -f "ros2 run rm_video_player" 2>/dev/null
  pkill -f "ros2 run armor_detector" 2>/dev/null
  pkill -f "ros2 run armor_solver" 2>/dev/null
  echo "Stopped."
}

status() {
  echo "=== Processes ==="
  ps aux | grep -E "vision_ws/(install|lib)|tf2_ros static" | grep -v grep | awk '{print $2, $11}' | head -12
  echo "=== Topics ==="
  source /opt/ros/humble/setup.bash; source $WS/install/setup.bash
  timeout 4 ros2 topic hz /armor_detector/armors 2>&1 | tail -1
  timeout 4 ros2 topic hz /armor_solver/target 2>&1 | tail -1
  timeout 4 ros2 topic hz /armor_solver/cmd_gimbal 2>&1 | tail -1
}

case "${1:-}" in
  start) start ;;
  stop) stop ;;
  status) status ;;
  *) echo "Usage: $0 {start|stop|status}" ;;
esac
