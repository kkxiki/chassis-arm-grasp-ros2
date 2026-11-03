# AGV 底盘 + 机械臂复合视觉抓取（ROS 2）

<p>
<img alt="ROS 2" src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white">
<img alt="C++" src="https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B&logoColor=white">
<img alt="Python" src="https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white">
<img alt="YOLOv8" src="https://img.shields.io/badge/Detection-YOLOv8-00FFFF?logo=yolo&logoColor=black">
<img alt="Platform" src="https://img.shields.io/badge/Platform-AGV%20%2B%20Arm-informational">
<a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-MIT-yellow.svg"></a>
</p>

AGV 移动底盘与六轴机械臂的复合抓取系统。核心是一个 **9 阶段 `GraspMission` 状态机**：底盘导航到位、固定相机粗定位、底盘纯平移接近、腕部相机精定位、机械臂抓取、退回释放，全流程带分级执行总闸与失败中止路径。

与常见的"固定工位抓取"不同，本项目要解决的是**移动底盘 + 机械臂**的复合位姿问题：相机装在腕部、底盘会动、机械臂基座随底盘一起平移，粗定位坐标系与执行坐标系不在同一个参考系里。

## 技术指标

| 指标 | 数值 |
| --- | --- |
| 状态机阶段 | 9 个主阶段 + `ABORTED` 统一中止路径 |
| 视觉输入 | 固定相机（臂基座系）+ 腕部相机（像素 + 深度）双路 |
| 图像基准 | 1280 × 720，固定工位与腕部一致 |
| 底盘接近单步上限 | 400 mm（`max_approach_step_mm`） |
| 底盘接近迭代上限 | 3 次（含单步截断与测量噪声两种情况） |
| 精定位迭代上限 | 2 次，带 60 mm 修正范数守卫 |
| 合爪稳定等待 | 900 ms（`hand_wait_ms`） |
| 执行分级 | `execute_enabled` 总闸 + `execute_chassis` / `execute_arm` / `execute_gripper` 分闸 |

## 系统架构

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          GraspMission（独占任务线程）                      │
│                                                                          │
│  S1 NAV_TO_SITE ──▶ S2 COARSE_DETECT ──▶ S3 CHASSIS_APPROACH ──┐        │
│                                               ▲                 │        │
│                                               │                 ▼        │
│                                        S4 COARSE_RECHECK ◀──────┘        │
│                                               │                          │
│                                               ▼                          │
│  S9 RETREAT_RELEASE ◀── S8 GRIP_CLOSE ◀── S7 ARM_GRASP ◀── S6 FINE_DETECT│
│         │                                        ▲                │       │
│         ▼                                        └── S5 ARM_PREGRASP ◀────┘
│     COMPLETED                                                             │
│                                                                          │
│  任一阶段失败 ──────────────────────────▶ ABORTED(reason)                  │
└──────────────────────────────────────────────────────────────────────────┘
        │                    │                    │                │
        ▼                    ▼                    ▼                ▼
  ChassisController   ArmDirectController   Gripper (RPC)   VisionGraspNode ×2
  translateDistance   addCartPoint          hand_close_cmd  固定 / 腕部 (provider)
  goToPose            moveCartL                             仅产出目标，不执行
```

## 核心技术难点

### 1. 底盘接近为什么只平移、且几何上一步到位

S3 是整个系统最容易写错的地方。朴素做法是"转到目标方向再往前走"，但一旦调用 `rotateAngle`，机械臂基座跟着旋转，粗目标在臂基座系下的坐标 `Δ_b` 当场失效，问题退化成没有解析解的迭代。

本项目做法是**只发一次纯平移**。设臂基座系 B 与底盘系 C 之间只差固定旋转 `Rz(ψ)`（ψ 由安装外参给出），目标在世界系静止，则：

```
p_b − Rz(ψ)ᵀ · T_c = s_b
        ⟹  T_c = Rz(ψ) · (p_b − s_b) = Rz(ψ) · Δ_b
```

也就是说 `Δ_c = Rz(ψ) · Δ_b` **本身就是精确解，几何上一步到位、没有残差**。这条结论只在"不旋转"前提下成立——这就是 S3 禁止旋转的根本原因，而不是风格选择。

推导与失效判据见 [`design/STATE_MACHINE.md`](design/STATE_MACHINE.md)。

### 2. 迭代次数不是收敛机制，而是失效判据

既然 S3 一步精确，`max_approach_iterations=3` 就**不应该**被当成收敛循环。需要第二步只可能有两个正当原因：

1. `d > max_approach_step_mm`，单步被 400 mm 截断；
2. 粗定位测量噪声。

于是加入显式的**有效性判据**：

```
shrink_expected = d_cmd
shrink_actual   = d_before − d_after

若 shrink_actual < approach_step_check_ratio · shrink_expected
   → 立即中止："底盘接近无效：安装外参或方向符号可能错误"
```

否则重复迭代只会把同一个系统性错误犯三遍。`approach_angle_sign ∈ {+1, −1}` 用于吸收控制器"前方右侧为正"与数学 `atan2` 逆时针为正的符号差异，必须在空载、总闸关闭条件下验收确定——**不允许靠调大 `max_approach_step_mm` 绕过方向错误**。

### 3. 双相机两段式：为什么粗定位和精定位必须分开

固定相机的标定残差决定了它只能做`厘米级`粗定位；抓取需要的精度远高于此。所以系统设计成两段：

- **固定相机**（臂基座系）：粗目标，驱动底盘接近到容差内；
- **腕部相机**（像素 + 深度）：到了预抓点之后做精定位修正 `G_new = P_pos + R_P · (p_tool − o_tool)`，最多 2 次，带 60 mm 修正范数守卫（首轮以粗定位为 `G_old`，迭代后以上一轮为准）。

两个 provider 都跑 `provider_only=true`——只产出目标，不执行动作，执行权全部集中在 `GraspMission`，避免视觉节点和任务线程抢控制权。

### 4. 目标半径补偿必须两侧一致

YOLO 给的是物体**表面**深度，抓取点要的是**中心**。系统用 `object_radius_mm` + CameraInfo 做补偿（抓 500 ml 瓶应填 `32.0 mm`，填 `0` 会系统性偏近一个半径）。

关键约束：**S2/S4 与 S6 必须复用同一个 `compensateDepthForRadius` 和同一个 `object_radius_mm`**。只补一侧会让粗、精定位引入不同偏差，`correction` 守卫就失去意义了。内参未就绪时只 WARN 并原样使用深度，绝不以 `m≈1` 兜底猜测。

### 5. 分级执行总闸

调试期间最危险的是"改了一处配置，机器人突然动了"。系统把执行权拆成四级：

| 开关 | 作用 |
| --- | --- |
| `vision.execute_enabled` | 总闸，默认 `false` |
| `execute_chassis` | 允许底盘移动 |
| `execute_arm` | 允许机械臂运动 |
| `execute_gripper` | 允许夹爪动作 |

全部关闭时，`provider_only` 仍会照常做完坐标变换并写入 `target_`（干跑闸只抑制 `executeGrasp`），因此可以**在完全不动的条件下验收坐标输出**——这是新环境调试的第一步。

## 状态机明细

| 阶段 | 状态 | 动作 | 失败转移 |
| --- | --- | --- | --- |
| S1 | `NAV_TO_SITE` | 导航到作业站点并等待底盘停稳 | `abort` |
| S2 | `COARSE_DETECT` | 从固定相机 provider 原子取走一条确认目标 | 超时 → `abort` |
| S3 | `CHASSIS_APPROACH` | 先同步回安全位，再一次纯平移接近 | 指令失败 / 超迭代 → `abort` |
| S4 | `COARSE_RECHECK` | 复用固定相机重测目标 | 超时 → `abort` |
| S5 | `ARM_PREGRASP` | 由粗抓取点与 `pregrasp_dist_mm` 计算预抓点并 `moveCartL` | 臂未连接 → `abort` |
| S6 | `FINE_DETECT` | 腕部目标反投影 + 工具系外参换算，最多 2 次修正 | 无目标 / 内参无效 / 超守卫 → `abort` |
| S7 | `ARM_GRASP` | `moveCartL` 到修正后的抓取点 | 调用失败 → `abort` |
| S8 | `GRIP_CLOSE` | `sendRawCmd(hand_close_cmd)`，等待 900 ms 后置 `holding_` | RPC 失败 → `abort` |
| S9 | `RETREAT_RELEASE` | 回预抓点 → 释放关节位 → 张爪 → 配置安全位 | 任一步失败 → `abort` |

## 目录结构

```
├── CMakeLists.txt              # 主工程 agv_chassis_demo
├── package.xml
├── config/
│   ├── config.json             # 底盘/机械臂地址、速度、视觉与抓取参数
│   ├── task_list.json          # 巡检任务与站点列表
│   └── waypoints.json          # 路径点
├── include/
│   ├── arm/                    # 机械臂控制（RPC 网关 / TCP 直连）
│   ├── chassis/                # 底盘运动与回零
│   ├── common/                 # HTTP 工具
│   ├── dispatch/               # 调度客户端
│   ├── task/                   # 任务管理与状态机
│   └── vision/                 # 视觉抓取 provider
├── src/                        # 对应实现
├── design/
│   ├── STATE_MACHINE.md        # S1–S9 状态机、S3 推导、失效判据
│   ├── RISKS.md                # 10 条已知限制与实施注意事项
│   ├── CHANGES.md              # 逐文件实施清单
│   ├── LAUNCH.md               # 启动顺序与验收步骤
│   ├── config_schema.json      # 配置项 schema
│   └── grasp_mission.h
├── hardware_tests/             # 各硬件独立测试（arm / camera / chassis / gripper）
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

完整启动顺序（相机 → 深度服务 → 检测 → 主程序）见 [`design/LAUNCH.md`](design/LAUNCH.md)。

```bash
ros2 run agv_chassis_demo agv_demo
```

启动后自检：

```bash
ros2 topic echo --once /wrist/camera/color/camera_info
ros2 service list | grep gather_depth
ros2 topic hz /yolov8/detections
ros2 topic hz /wrist/detections
```

### 推荐调试顺序

1. 全部执行闸关闭 → 运行 `mission_start`，只检查坐标输出是否合理；
2. 空载条件下验收 `approach_angle_sign` 是 `+1` 还是 `−1`；
3. 依次打开 `execute_chassis` → `execute_arm` → `execute_gripper`，每次打开后重启程序验证。

## ⚠️ 使用前必读

[`design/RISKS.md`](design/RISKS.md) 记录了 10 条已知限制，务必先读。其中三条直接影响安全：

1. **控制器无位姿回读、无运动完成反馈**。`moveCartL` 只按传入的 `wait_ms` 阻塞后返回，不能从控制器确认真实走完。`move_wait_ms` 必须现场校准为所有相关轨迹耗时的**上界**。
2. **`hand_open_cmd` 中的 `--distance=100` 是未经实测的占位值**。抓取前必须用 `hardware_tests/arm` 的 `raw` 命令从小行程逐步试出真实最大开口，再写回配置。
3. **`main.cpp` 将配置路径硬编码**为 `/home/robot/robot_ws/config/*.json`。部署到其他机器需确保该路径存在，或改为启动参数。

现场作业前请确认机械臂周围无人、急停可达。

## Roadmap

以下为**规划中、尚未实现**的改进（多数来自 `design/RISKS.md` 中已识别的限制）：

- [x] 9 阶段复合状态机与统一 `abort` 路径
- [x] 底盘接近的单步有效性判据
- [x] 分级执行总闸与干跑验收
- [x] 目标半径深度补偿（粗/精两侧一致）
- [ ] 控制器位姿回读与运动完成状态接口（替代当前的等待窗口）
- [ ] 配置路径改为启动参数，去掉 `main.cpp` 硬编码
- [ ] `object_radius_mm` 按物体尺寸自动量取，不再手工填
- [ ] 腕部外参在线漂移检测（用重投影残差实时判断）
- [ ] 抓取失败自动重试（当前失败即中止）
- [ ] 多类别目标识别（当前仅单一类别）

## 关于本仓库

- **标定照片未包含**：`hardware_tests/camera` 下的标定图像未纳入版本库，仅保留角点数据 `pairs_all/*/board.json` 与对应 TCP 位姿 `tcp.txt`，足以复现标定计算。
- **示例配置中的内网地址**（`192.168.30.x`）为原作者实验环境，均属私有网段，请按现场实际修改。
- **第三方依赖**：`third_party/nlohmann/json.hpp` 遵循 MIT 许可，版权归原作者所有。

## 许可

本项目基于 [MIT License](LICENSE) 开源。
`src/chassis_driver` 沿用其 `package.xml` 中声明的 Apache-2.0。
