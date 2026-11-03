# AGV 底盘 + 机械臂视觉抓取（ROS 2）

AGV 移动底盘与机械臂的集成控制程序，实现「导航到作业点 → 视觉定位 → 底盘接近 → 机械臂抓取 → 退回释放」的完整闭环，同时提供底盘、机械臂、夹爪、相机的独立硬件测试工具。

- **框架**：ROS 2 Humble、C++17、CMake / ament_cmake
- **底盘**：AGV COR-A100SV，HTTP 接口（`/nd-mgr`）
- **机械臂**：贞实 taiKeU058T，RPC 网关 + TCP 直连（端口 5868）
- **夹爪**：Inspire 4B4C 二指夹爪
- **视觉**：YOLOv8 检测输出（`vision_msgs`）+ 深度服务，固定相机与腕部相机双阶段
- **任务调度**：HTTP 调度接口下发任务与站点列表

## 抓取状态机

主流程由 `GraspMission` 驱动，失败统一走 `abort`：

| 阶段 | 状态 | 说明 |
| --- | --- | --- |
| S1 | `NAV_TO_SITE` | 底盘导航到作业站点 |
| S2 | `COARSE_DETECT` | 固定相机粗定位目标 |
| S3 | `CHASSIS_APPROACH` | 底盘纯平移接近，迭代至死区 |
| S4 | `COARSE_RECHECK` | 固定相机复测 |
| S5 | `ARM_PREGRASP` | 机械臂运动到预抓点 |
| S6 | `FINE_DETECT` | 腕部相机精定位与深度补偿 |
| S7 | `ARM_GRASP` | 机械臂运动到抓取点 |
| S8 | `GRIP_CLOSE` | 合爪 |
| S9 | `RETREAT_RELEASE` | 退回并释放 |
| — | `COMPLETED` | 任务完成 |

详细的状态推进公式、坐标变换与验收条件见 [`design/STATE_MACHINE.md`](design/STATE_MACHINE.md)。

## 目录结构

```
├── CMakeLists.txt              # 主工程（agv_chassis_demo）
├── package.xml
├── config/                     # 运行时配置
│   ├── config.json             # 底盘/机械臂地址、速度、视觉与抓取参数
│   ├── task_list.json          # 巡检任务与站点列表
│   └── waypoints.json          # 路径点
├── include/                    # 头文件
│   ├── arm/                    # 机械臂控制（RPC / TCP 直连）
│   ├── chassis/                # 底盘控制
│   ├── common/                 # HTTP 工具
│   ├── dispatch/               # 调度客户端
│   ├── task/                   # 任务管理
│   └── vision/                 # 视觉抓取
├── src/                        # 实现
├── design/                     # 设计文档
│   ├── STATE_MACHINE.md        # 状态机与坐标变换
│   ├── CHANGES.md              # 逐文件实施清单
│   ├── RISKS.md                # 已知风险与未验收项
│   ├── LAUNCH.md               # 启动顺序
│   ├── config_schema.json      # 配置项 schema
│   └── grasp_mission.h         # GraspMission 头文件
├── hardware_tests/             # 各硬件独立测试
│   ├── arm/                    # 机械臂 RPC 命令与日志查看
│   ├── camera/                 # Orbbec 相机采集、标定、YOLO 检测
│   ├── chassis/                # 底盘运动/回零/恢复测试
│   └── gripper/                # 夹爪控制
└── third_party/nlohmann/       # json.hpp（MIT）
```

## 构建

```bash
source /opt/ros/humble/setup.bash

# 依赖：libcurl、nlohmann-json3、vision_msgs、service_interfaces
cd ~/agv_chassis_demo
colcon build
source install/setup.bash
```

## 运行

完整启动顺序（相机 → 深度服务 → 检测 → 主程序）见 [`design/LAUNCH.md`](design/LAUNCH.md)。要点：

```bash
ros2 run agv_chassis_demo agv_demo
```

启动前建议先确认各个 ROS 话题与服务就位：

```bash
ros2 topic echo --once /wrist/camera/color/camera_info
ros2 service list | grep gather_depth
ros2 topic hz /yolov8/detections
ros2 topic hz /wrist/detections
```

## 配置

`config/config.json` 中的关键项：

| 配置项 | 说明 |
| --- | --- |
| `chassis_url` / `dispatch_url` | 底盘与调度服务地址 |
| `arm_ip` / `arm_port` | 机械臂 RPC 地址与端口 |
| `robot_id` / `map_id` | 机器人编号与地图编号 |
| `move_speed` / `rot_speed` | 底盘线速度 / 角速度 |
| `vision.*` | 检测话题、类别、置信度、画面尺寸、工作空间限位 |
| `vision.execute_enabled` | **运动总开关，默认 `false`**（只计算不执行） |
| `vision.grasp.*` | 预抓距离、抓取姿态、安全位关节角、夹爪开关命令 |

## ⚠️ 使用前必读

本仓库保留了原项目的完整设计文档，其中 [`design/RISKS.md`](design/RISKS.md) 记录了**尚未验收**的项，务必先读。几个关键点：

1. **`execute_enabled` 默认为 `false`**。首次调试必须先在只计算坐标、不驱动机器的模式下确认输出，再依次打开 `execute_chassis`、`execute_arm`、`execute_gripper` 并重启程序。
2. **`hand_open_cmd` 中的 `--distance=100` 是占位值，未经实测**。抓取执行前必须用 `hardware_tests/arm` 的 `raw` 命令从小行程逐步试出真实最大开口，再写回配置。
3. **`main.cpp` 中配置路径为硬编码**（`/home/robot/robot_ws/config/*.json`）。部署到其他机器时需确保该路径存在，或改为启动参数。
4. 机械臂工作空间限位、接近方向符号（`approach_angle_sign` 为 `+1` 还是 `-1`）必须在空载条件下验收，不得靠调大 `max_approach_step_mm` 规避。

现场作业前请确认机械臂周围无人、急停可达。

## 关于本仓库

- **标定照片未包含**：`hardware_tests/camera` 下的标定图像（含实验室实景）未纳入版本库，仅保留了标定角点数据 `pairs_all/*/board.json` 与对应 TCP 位姿 `tcp.txt`，足以复现标定计算。
- **示例配置中的内网地址**（`192.168.30.x`）为原作者实验环境，均属私有网段，请按现场实际情况修改。
- **第三方依赖**：`third_party/nlohmann/json.hpp` 遵循 MIT 许可，版权归原作者所有。
