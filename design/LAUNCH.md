# 启动顺序

以下命令假定 ROS 2 Humble 已安装、`agv_chassis_demo` 与 `legacy_grasp_ws` 已各自完成 `colcon build`。每条命令在独立终端执行；所有检测输入均为 **1280×720**。先启动相机，再启动深度服务，再启动检测，最后启动主程序。

```bash
source /opt/ros/humble/setup.bash
source ~/legacy_grasp_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=fixed camera_name:=camera enable_color:=true enable_depth:=true align_depth.enable:=true color_width:=1280 color_height:=720 depth_width:=1280 depth_height:=720
```

```bash
source /opt/ros/humble/setup.bash
source ~/legacy_grasp_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=wrist camera_name:=camera enable_color:=true enable_depth:=true align_depth.enable:=true color_width:=1280 color_height:=720 depth_width:=1280 depth_height:=720
```

固定相机感知：

```bash
source /opt/ros/humble/setup.bash
source ~/legacy_grasp_ws/install/setup.bash
ros2 run gather_depth gather_depth
ros2 run yolov8_detector_node yolov8_detector_node --ros-args -p detections_topic:=/yolov8/detections
```

腕部相机感知。深度服务代码使用相对名 `gather_depth`，因此用指定的命名空间重映射后服务为 `/wrist/gather_depth`；腕部检测输出显式设为 `/wrist/detections`：

```bash
source /opt/ros/humble/setup.bash
source ~/legacy_grasp_ws/install/setup.bash
ros2 run gather_depth gather_depth --ros-args -r __ns:=/wrist
ros2 run yolov8_detector_node yolov8_detector_node --ros-args -p detections_topic:=/wrist/detections
```

最后启动本工程：

```bash
source /opt/ros/humble/setup.bash
source ~/agv_chassis_demo/install/setup.bash
ros2 run agv_chassis_demo agv_demo
```

CameraInfo 配置（两个阶段各自使用，均为具体 ROS 话题）：

- 固定相机：`/fixed/camera/color/camera_info` 配到 `vision.camera_info_topic`；它供 `VisionGraspNode` 在 S2/S4 调用 `transform_xyz` 前完成表面→中心深度补偿。
- 腕部相机：`/wrist/camera/color/camera_info` 配到 `grasp_mission.wrist_camera_info_topic`；它供 `GraspMission` 在 S6 反投影前完成同一补偿。

启动后先以 `grasp_mission.execute_enabled=false` 运行 `mission_start`，只检查输出坐标；此 provider-only 干跑必须仍能产出目标。先在此条件、空载下验收 `approach_angle_sign` 为 `+1` 还是 `-1`，不得以调大 `max_approach_step_mm` 规避方向错误。依次打开 `execute_chassis`、`execute_arm`、`execute_gripper` 后重启程序再验证。每次启动前应确认：

```bash
ros2 topic echo --once /wrist/camera/color/camera_info
ros2 service list | grep gather_depth
ros2 topic hz /yolov8/detections
ros2 topic hz /wrist/detections
```

`gather_depth`、`yolov8_detector_node` 的 ROS 包名/可执行名在本仓库未出现；这里按用户给定组件名写出 `ros2 run <包名> <可执行名>`。若 legacy 工作区实际包名不同，先运行 `ros2 pkg executables | grep -E 'gather_depth|yolov8_detector_node'`，将命令的前两个标识替换为该输出的真实二元组；命名空间和参数必须保持不变。

**抓手张开值尚未验收：** `hand_open_cmd` 中的 `--distance=100` 只是占位，未经实测。任何抓取执行前，必须使用 `hardware_tests/arm` 的 `raw` 命令从小行程逐步试出最大开口，再把实测值写回配置。

`wrist_camera_intrinsics` 的默认值同样只是**在 CameraInfo 不可用时的应急值**；正常运行应以 `/wrist/camera/color/camera_info` 为准，并在启用机械臂前实测确认该兜底参数。
