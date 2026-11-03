# 逐文件实施清单

下列行号基于当前工作树；实施时以函数名定位。除列出的新增文件外，不改动其他文件。

## 新增 `include/task/grasp_mission.h`、`src/task/grasp_mission.cpp`

头文件的完整内容见 [grasp_mission.h](grasp_mission.h)。实现新增 `loadGraspMissionConfig` 和 `GraspMission`，并严格使用下列已核实接口：`ChassisController::goToPose(float,float,float,bool)`（S1，仅米/度）、`translateDistance(float,int,float)`（S3 纯平移，度、mm、m/s）、`ArmDirectController::addCartPoint(name,x,y,z,rx,ry,rz)`（mm/度）、`addJointPoint(name,j1,j2,j3,j4,j5,j6)`、`moveCartL(name,wait_ms,retry)`、`moveToPoint(name,wait_ms,retry)`、`sendRawCmd(cmd,timeout,retry)`。S3 不使用 `rotateAngle`；任务不调用 `goSafeHome()`，因为它会忽略配置。

`run()` 的完整状态推进骨架如下（各 `step...` 按 `STATE_MACHINE.md` 的公式实现，失败统一调用 `abort`）：

```cpp
void GraspMission::run() {
    VisionGraspNode::ProviderTarget coarse;
    state_ = State::NAV_TO_SITE;
    if (cfg_.execute_enabled && cfg_.execute_chassis &&
        chassis_.goToPose(static_cast<float>(cfg_.site_x_m),
                          static_cast<float>(cfg_.site_y_m),
                          static_cast<float>(cfg_.site_yaw_deg), true) != CHASSIS_OK) {
        return abort("S1 导航到作业点失败");
    }
    state_ = State::COARSE_DETECT;
    if (!takeCoarseTarget(coarse)) return abort("S2 未取得固定相机目标");
    for (;;) {
        state_ = State::CHASSIS_APPROACH;
        if (!performApproach(coarse)) return abort("S3 底盘接近失败或超出迭代次数");
        if (approach_iterations_ == -1) break;  // 已在死区内
        state_ = State::COARSE_RECHECK;
        if (!takeCoarseTarget(coarse)) return abort("S4 未取得复测目标");
    }
    state_ = State::ARM_PREGRASP;
    if (!movePregrasp()) return abort("S5 预抓点运动失败");
    state_ = State::FINE_DETECT;
    if (!fineCorrect()) return abort("S6 腕部精定位失败");
    state_ = State::ARM_GRASP;
    if (!moveGrasp()) return abort("S7 抓取点运动失败");
    state_ = State::GRIP_CLOSE;
    if (!closeGripper()) return abort("S8 合爪失败");
    state_ = State::RETREAT_RELEASE;
    if (!retreatRelease()) return abort("S9 退回释放失败");
    state_ = State::COMPLETED;
    running_ = false;
    state_ = State::IDLE;
}

void GraspMission::abort(const std::string& reason) {
    state_ = State::ABORTED;
    { std::lock_guard<std::mutex> lock(mutex_); abort_reason_ = reason; }
    // 调用点均在阻塞 MoveL/MoveJ 返回后；此处绝不调用 arm_->stop()。
    if (arm_ && arm_->isConnected()) {
        if (!holding_ && cfg_.execute_enabled && cfg_.execute_gripper)
            arm_->sendRawCmd(cfg_.hand_open_cmd);
        if (cfg_.execute_enabled && cfg_.execute_arm) moveMissionSafeHome();
    }
    if (arm_working_) arm_working_->store(false);
    holding_ = false; running_ = false; state_ = State::IDLE;
}
```

`moveMissionSafeHome()` 是任务内唯一的安全位实现，禁止转调 `ArmDirectController::goSafeHome()`：

```cpp
bool GraspMission::moveMissionSafeHome() {
    if (!arm_ || !arm_->isConnected()) return false;
    const auto& j = cfg_.safe_home_joints;
    if (!arm_->addJointPoint("mission_safe_home", j[0], j[1], j[2],
                             j[3], j[4], j[5])) return false;
    return arm_->moveToPoint("mission_safe_home", cfg_.move_wait_ms, 0);
}
```

`loadGraspMissionConfig()` 读取 `object_radius_mm`（默认 `0.0 mm`）和 `approach_step_check_ratio`（默认 `0.6`、无单位）。`object_radius_mm` 的配置注释必须保留“取 0 意味着系统性偏近约一个物体半径；抓 500ml 瓶填 32.0”。删除 `GraspMission` 私有 `CameraIntrinsics`，其缓存和 `currentIntrinsics()` 返回类型统一为 `PinholeIntrinsics`。

`performApproach()` 是 S3 单步校验的实现位置。执行实际平移前记录 `d_before` 与 `d_cmd`；S4 重测、再次算出距离后在进入下一轮前执行：

```cpp
const double shrink_expected = last_approach_cmd_mm_;
const double shrink_actual = last_approach_distance_before_mm_ - d_after;
if (shrink_actual < cfg_.approach_step_check_ratio * shrink_expected) {
    return abort("底盘接近无效：安装外参或方向符号可能错误");
}
```

首次 S3 或因 `d > max_approach_step_mm` 而被截断时才允许进入这项检查后的下一步；这不是收敛迭代。每次实际 S3 平移仍必须先要求总闸、底盘闸和臂闸均开启，同步调用 `moveMissionSafeHome()` 且成功返回，再检查 `arm_working->load()==false`；任一条件不满足即中止，绝不下发平移。全关干跑只记录该计划动作。随后只调用一次 `translateDistance(cfg_.approach_angle_sign * alpha_math, lround(d_cmd), cfg_.approach_speed_mps)`；绝不调用 `rotateAngle`。`fineCorrect()` 在反投影**之前**调用 `compensateDepthForRadius(target.depth_m * 1000.0, target.pixel_u, target.pixel_v, intr, cfg_.object_radius_mm)`，再用返回的毫米深度反投影；之后按 `STATE_MACHINE.md` 直接计算 `G_new=P_pos+R_P·(p_tool-o_tool)`，并以 `||G_new-G_old||₂` 作 60 mm 守卫，首次 `G_old` 是粗定位点、后续为上一轮 `G_new`。

原因：将 S1–S9、`holding_`、中止语义和 ROS `CameraInfo` 缓存从 651 行的 `main.cpp` 隔离。

## 修改 `include/vision/vision_grasp.h`

位置：`VisionConfig`（24 行）、`VisionGraspNode` 公有区（72 行）、`GraspTarget`（91 行）。新增字段和完整目标消费接口：

```cpp
// VisionConfig 新增；transform_enabled 默认 true，保证旧路径不变。
bool provider_only = false;
bool transform_enabled = true;
std::string depth_service = "gather_depth";
std::string transform_service = "transform_xyz";
std::string service_client_node_name = "vision_grasp_fixed_service_client";
// 空字符串表示不订阅、不补偿；固定相机应配置实际 CameraInfo 话题。
std::string camera_info_topic = "";
// 运行时由 GraspMissionConfig::object_radius_mm 镜像，mm；不另设独立配置源。
double object_radius_mm = 0.0;

struct ProviderTarget {
    double x_mm{0.0}, y_mm{0.0}, z_mm{0.0};
    int pixel_u{0}, pixel_v{0};
    double depth_m{0.0};
    std::chrono::steady_clock::time_point confirmed_at{};
    bool has_base_point{false};
};

// 成功时在同一互斥锁内复制并清空；一次确认目标只能被消费一次。
bool takeProviderTarget(ProviderTarget& out);
```

并在 `namespace agv` 的头文件公共区（`VisionConfig` 之前）增加下面唯一共享定义；并在头文件增加 `#include <chrono>`、`#include <mutex>`、`#include "sensor_msgs/msg/camera_info.hpp"`：

```cpp
// 针孔内参
struct PinholeIntrinsics {
    double fx{0.0}, fy{0.0}, cx{0.0}, cy{0.0};
    bool valid{false};
};

// 沿视线方向把表面点前推一个物体半径。
// 推导见 design/STATE_MACHINE.md：反投影对 z 线性，故等价于只调整深度标量。
//   m      = ‖[(u-cx)/fx, (v-cy)/fy, 1]‖₂
//   z_comp = depth_mm + radius_mm / m
// radius_mm <= 0 或内参无效时原样返回 depth_mm。
double compensateDepthForRadius(double depth_mm, double u, double v,
                                const PinholeIntrinsics& intr, double radius_mm);
```

`VisionGraspNode` 还须声明 `void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);` 和 `PinholeIntrinsics currentIntrinsics() const;`，并持有 `mutable std::mutex intrinsics_mutex_`、`PinholeIntrinsics camera_intrinsics_` 与 `rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_`。原私有 `GraspTarget` 扩展为 `ProviderTarget provider;`，或直接以 `ProviderTarget` 取代，避免两份坐标定义。

## 修改 `src/vision/vision_grasp.cpp`

位置：`loadVisionConfig`（74–110 行）、构造函数（112–149 行）、`processCandidate`（250–301 行）、`graspLoop`（445–470 行）。

`loadVisionConfig` 追加：

```cpp
out.provider_only = jsonBool(cfg, "provider_only", out.provider_only);
out.transform_enabled = jsonBool(cfg, "transform_enabled", out.transform_enabled);
out.depth_service = jsonString(cfg, "depth_service", out.depth_service);
out.transform_service = jsonString(cfg, "transform_service", out.transform_service);
out.service_client_node_name = jsonString(cfg, "service_client_node_name", out.service_client_node_name);
out.camera_info_topic = jsonString(cfg, "camera_info_topic", out.camera_info_topic);
```

构造函数中将固定字符串替换为配置字段，创建 CameraInfo 订阅（仅在 `camera_info_topic` 非空时），且仅在旧路径创建抓取线程：

```cpp
service_node_ = rclcpp::Node::make_shared(cfg_.service_client_node_name);
depth_client_ = service_node_->create_client<service_interfaces::srv::GatherDepth>(cfg_.depth_service);
if (cfg_.transform_enabled)
    transform_client_ = service_node_->create_client<service_interfaces::srv::TransformXyz>(cfg_.transform_service);
if (!cfg_.camera_info_topic.empty()) {
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cfg_.camera_info_topic, rclcpp::SensorDataQoS(),
        std::bind(&VisionGraspNode::cameraInfoCallback, this, std::placeholders::_1));
}
if (!cfg_.provider_only)
    grasp_thread_ = std::thread(&VisionGraspNode::graspLoop, this);
```

`cameraInfoCallback()` 仅在 `msg->k[0] > 0.0 && msg->k[4] > 0.0` 时，在 `intrinsics_mutex_` 保护下缓存 `{msg->k[0], msg->k[4], msg->k[2], msg->k[5], true}`；否则将缓存标为无效。`currentIntrinsics()` 在同一锁下复制缓存。

在 `vision_grasp.cpp`（`namespace agv` 内）实现共享函数；两个调用点只能调用此实现：

```cpp
double compensateDepthForRadius(double depth_mm, double u, double v,
                                const PinholeIntrinsics& intr, double radius_mm) {
    if (radius_mm <= 0.0 || !intr.valid || intr.fx <= 0.0 || intr.fy <= 0.0)
        return depth_mm;
    const double x = (u - intr.cx) / intr.fx;
    const double y = (v - intr.cy) / intr.fy;
    const double m = std::sqrt(x * x + y * y + 1.0);
    return depth_mm + radius_mm / m;
}
```

在 `processCandidate` 中，深度服务成功后保存 `depth_m=depth_mm/1000.0`、像素和 `confirmed_at=steady_clock::now()`；仅 `transform_enabled=true` 时调用现有 `callTransformXyz` 和工作区检查。调用 `transform_xyz` 前先计算深度；内参未就绪时不得猜测，必须 WARN 并原样传入 `depth_mm`：

```cpp
double depth_for_transform_mm = depth_mm;
if (cfg_.object_radius_mm > 0.0) {
    const PinholeIntrinsics intr = currentIntrinsics();
    if (!intr.valid) {
        RCLCPP_WARN(get_logger(),
                    "CameraInfo not ready; skip coarse surface-to-center compensation");
    } else {
        depth_for_transform_mm = compensateDepthForRadius(
            depth_mm, u, v, intr, cfg_.object_radius_mm);
    }
}
```

`VisionConfig` 也须有 `double object_radius_mm{0.0};`，由任务装配时设为与 `GraspMissionConfig::object_radius_mm` 完全相同的值；这保证两处调用使用同一参数。目标通过全部检查后必须先无条件写入 `target_`；随后仅当 `!provider_only && execute_enabled` 才调用 `executeGrasp`。不得在写入前因 `execute_enabled=false` 早退。确认后的完整分支如下：

```cpp
ProviderTarget p;
p.pixel_u = u; p.pixel_v = v; p.depth_m = depth_mm / 1000.0;
p.confirmed_at = std::chrono::steady_clock::now();
if (cfg_.transform_enabled) {
    if (!callTransformXyz(u, v, depth_for_transform_mm, p.x_mm, p.y_mm, p.z_mm) ||
        !withinWorkspace(p.x_mm, p.y_mm, p.z_mm)) return;
    p.has_base_point = true;
}
latched = true; stable_count = 0;
{
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!target_.is_exist && !grasp_busy_) {
        target_.is_exist = true;
        target_.provider = p;  // provider_only/干跑同样产出目标
    }
}
if (!cfg_.provider_only && cfg_.execute_enabled)
    executeGrasp();
else if (!cfg_.provider_only)
    RCLCPP_WARN(get_logger(), "DRY RUN: controller command suppressed");
```

新增消费实现：

```cpp
bool VisionGraspNode::takeProviderTarget(ProviderTarget& out) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!cfg_.provider_only || !target_.is_exist || grasp_busy_) return false;
    out = target_.provider;
    target_.is_exist = false;
    latched_ = false;
    return true;
}
```

旧模式仍以原 `execute_enabled` 和 `graspLoop/executeGrasp` 执行，故行为不变；provider 模式不启动 `graspLoop`，也不会调用 `executeGrasp`。

## 修改 `src/main.cpp`

位置：包含区 19 行、`AppConfig` 53–66 行、`loadAppConfig` 71–101 行、`consoleInputThread` 208 行、`main` 532–639 行。包含 `task/grasp_mission.h`；`AppConfig` 新增 `agv::GraspMissionConfig grasp_mission;`，加载：

```cpp
agv::loadGraspMissionConfig(cfg.contains("grasp_mission") ? cfg["grasp_mission"] : json(),
                             out_config.grasp_mission);
```

新增 `mission_start`、`mission_abort` 命令。`mission_start` 必须拒绝已有导航/调度/视觉抓取任务；启动 ROS 后构造两份 provider：固定相机使用 `vision.provider_only=true, transform_enabled=true, service_client_node_name="vision_grasp_fixed_service_client"`，腕部使用 `wrist_vision.provider_only=true, transform_enabled=false, depth_service="/wrist/gather_depth", service_client_node_name="vision_grasp_wrist_service_client"`。固定相机必须显式继承同一个任务半径，腕部 provider 不负责 S6 补偿：

```cpp
VisionConfig fixed_cfg = app_config.vision;
fixed_cfg.provider_only = true;
fixed_cfg.transform_enabled = true;
fixed_cfg.object_radius_mm = app_config.grasp_mission.object_radius_mm;

VisionConfig wrist_cfg = app_config.wrist_vision;
wrist_cfg.provider_only = true;
wrist_cfg.transform_enabled = false;
// S6 由 GraspMission 以 grasp_mission.object_radius_mm 调用同一共享函数。
```

节点名从配置读取，且同一进程内必须唯一。随后构造 `GraspMission` 并调用 `start()`。将 `mission->cameraInfoNode()` 也加入现有 `MultiThreadedExecutor`，否则 S6 的 `CameraInfo` 回调不会执行。`mission_abort` 调用 `requestAbort("操作员命令")`。退出路径在 `stopVisionGrasp()` 前调用 `mission->requestAbort("程序退出")` 并等待任务线程结束。

原因：主程序只负责装配、互斥和命令，不承载状态机。

## 修改 `CMakeLists.txt`

位置：ROS `find_package` 10–14 行、源文件清单 17–27 行、`ament_target_dependencies` 44–48 行。完整增量：

```cmake
find_package(sensor_msgs REQUIRED)
# add_executable 源文件列表中新增：
src/task/grasp_mission.cpp
# ament_target_dependencies(agv_demo ...) 中新增：
sensor_msgs
```

同时修改 `package.xml`，在第 11 行后加入 `<depend>sensor_msgs</depend>`。原因：`CameraInfo` 是 S6 的真实 ROS 消息类型。
