// ============================================================================
// vision_grasp.cpp
// 视觉抓取模块（移植自 legacy_grasp_ws 的 yolo_result_subscriber.cpp）
//
// 整体流程：
//   1. 订阅 YOLO 检测结果话题（默认 /yolov8/detections）；
//   2. 回调内筛选目标类别与置信度，取本帧最高置信度的目标；
//   3. 连续稳定帧确认 + 目标消失释放帧，防止误检与重复抓取；
//   4. 调用 gather_depth 服务取深度（米→毫米）、transform_xyz 服务换算
//      机械臂基座坐标系坐标（毫米）；
//   5. 检查工作区后，把抓取任务写入共享状态，交给抓取线程；
//   6. 抓取线程通过 ArmDirectController 执行：
//      回安全位 → 直线到预抓点 → 直线到抓取点 → 合手(可选) →
//      直线退回预抓点 → 关节到释放位 → 放手(可选) → 回安全位。
//
// 与备份工程的区别：
//   - 备份是双臂（左右臂各一条抓取线程），本工程是单臂，只保留一条抓取线程；
//   - 备份用厂商 cpp_rpc SDK 发指令，本工程用 ArmDirectController(TCP)；
//   - 备份用四元数描述抓取姿态，本工程用 RPY（度），配置在 config.json。
// ============================================================================
#include "vision/vision_grasp.h"

#include <cmath>
#include <cstdio>
#include <thread>
#include <chrono>
#include <functional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace agv {

namespace {
// 从 json 中取 double，缺省时返回默认值
double jsonDouble(const nlohmann::json& j, const char* key, double def)
{
    if (j.is_object() && j.contains(key) && j[key].is_number())
    {
        return j[key].get<double>();
    }
    return def;
}
int jsonInt(const nlohmann::json& j, const char* key, int def)
{
    if (j.is_object() && j.contains(key) && j[key].is_number())
    {
        return j[key].get<int>();
    }
    return def;
}
std::string jsonString(const nlohmann::json& j, const char* key, const std::string& def)
{
    if (j.is_object() && j.contains(key) && j[key].is_string())
    {
        return j[key].get<std::string>();
    }
    return def;
}
bool jsonBool(const nlohmann::json& j, const char* key, bool def)
{
    if (j.is_object() && j.contains(key) && j[key].is_boolean())
    {
        return j[key].get<bool>();
    }
    return def;
}
std::vector<double> jsonDoubleArray(const nlohmann::json& j, const char* key,
                                    const std::vector<double>& def)
{
    if (j.is_object() && j.contains(key) && j[key].is_array())
    {
        std::vector<double> v;
        for (const auto& item : j[key])
        {
            if (item.is_number()) v.push_back(item.get<double>());
        }
        if (!v.empty()) return v;
    }
    return def;
}
}  // namespace

bool loadVisionConfig(const nlohmann::json& cfg, VisionConfig& out)
{
    if (cfg.is_null() || !cfg.is_object())
    {
        return false;
    }
    out.detections_topic  = jsonString(cfg, "detections_topic", out.detections_topic);
    out.target_class_id   = jsonString(cfg, "target_class_id", out.target_class_id);
    out.min_confidence    = jsonDouble(cfg, "min_confidence", out.min_confidence);
    out.image_width       = jsonInt(cfg, "image_width", out.image_width);
    out.image_height      = jsonInt(cfg, "image_height", out.image_height);
    out.arm_type          = jsonInt(cfg, "arm_type", out.arm_type);
    out.center_deadband_px = jsonDouble(cfg, "center_deadband_px", out.center_deadband_px);
    out.stable_frames     = jsonInt(cfg, "stable_frames", out.stable_frames);
    out.release_frames    = jsonInt(cfg, "release_frames", out.release_frames);
    out.max_center_jump_px = jsonDouble(cfg, "max_center_jump_px", out.max_center_jump_px);
    out.execute_enabled   = jsonBool(cfg, "execute_enabled", out.execute_enabled);
    out.workspace_x_min   = jsonDouble(cfg, "workspace_x_min", out.workspace_x_min);
    out.workspace_x_max   = jsonDouble(cfg, "workspace_x_max", out.workspace_x_max);
    out.workspace_y_min   = jsonDouble(cfg, "workspace_y_min", out.workspace_y_min);
    out.workspace_y_max   = jsonDouble(cfg, "workspace_y_max", out.workspace_y_max);
    out.workspace_z_min   = jsonDouble(cfg, "workspace_z_min", out.workspace_z_min);
    out.workspace_z_max   = jsonDouble(cfg, "workspace_z_max", out.workspace_z_max);

    if (cfg.contains("grasp") && cfg["grasp"].is_object())
    {
        const auto& g = cfg["grasp"];
        out.pregrasp_dist_mm = jsonDouble(g, "pregrasp_dist_mm", out.pregrasp_dist_mm);
        out.grasp_rpy_deg = jsonDoubleArray(g, "grasp_rpy_deg", out.grasp_rpy_deg);
        out.safe_home_joints = jsonDoubleArray(g, "safe_home_joints", out.safe_home_joints);
        out.release_joints   = jsonDoubleArray(g, "release_joints", out.release_joints);
        out.hand_close_cmd   = jsonString(g, "hand_close_cmd", out.hand_close_cmd);
        out.hand_open_cmd    = jsonString(g, "hand_open_cmd", out.hand_open_cmd);
        out.move_wait_ms     = jsonInt(g, "move_wait_ms", out.move_wait_ms);
    }
    return true;
}

VisionGraspNode::VisionGraspNode(const VisionConfig& cfg,
                                 ArmDirectController* arm,
                                 std::atomic<bool>* arm_working)
    : Node("vision_grasp"), cfg_(cfg), arm_(arm), arm_working_(arm_working)
{
    if (cfg_.image_width <= 0 || cfg_.image_height <= 0 ||
        cfg_.stable_frames < 1 || cfg_.release_frames < 1 ||
        cfg_.center_deadband_px < 0.0)
    {
        RCLCPP_ERROR(get_logger(), "vision 配置非法（图像尺寸/稳定帧/死区），请检查 config.json");
    }

    // 创建订阅：每收到一帧 YOLO 检测结果就回调一次
    sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        cfg_.detections_topic, 10,
        std::bind(&VisionGraspNode::detectionCallback, this, std::placeholders::_1));

    // 服务客户端使用独立节点：回调里调用 spin_until_future_complete 时
    // 不会和订阅节点所在执行器互相干扰（与备份工程一致）
    service_node_ = rclcpp::Node::make_shared("vision_grasp_service_client");
    depth_client_ = service_node_->create_client<service_interfaces::srv::GatherDepth>("gather_depth");
    transform_client_ = service_node_->create_client<service_interfaces::srv::TransformXyz>("transform_xyz");

    // 启动抓取线程
    grasp_thread_ = std::thread(&VisionGraspNode::graspLoop, this);

    RCLCPP_INFO(get_logger(),
                "YOLO grasp subscriber ready: topic=%s class=%s confidence>=%.2f "
                "arm_type=%d center_deadband=%.1fpx",
                cfg_.detections_topic.c_str(), cfg_.target_class_id.c_str(),
                cfg_.min_confidence, cfg_.arm_type, cfg_.center_deadband_px);
    if (cfg_.execute_enabled)
    {
        RCLCPP_WARN(get_logger(), "EXECUTION ENABLED: valid stable targets can command the arm.");
    }
    else
    {
        RCLCPP_WARN(get_logger(), "DRY-RUN MODE: coordinates are calculated but no arm command is sent.");
    }
}

VisionGraspNode::~VisionGraspNode()
{
    stop_requested_ = true;
    if (grasp_thread_.joinable())
    {
        grasp_thread_.join();
    }
}

void VisionGraspNode::requestStop()
{
    stop_requested_ = true;
}

bool VisionGraspNode::withinWorkspace(double x, double y, double z) const
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           x >= cfg_.workspace_x_min && x <= cfg_.workspace_x_max &&
           y >= cfg_.workspace_y_min && y <= cfg_.workspace_y_max &&
           z >= cfg_.workspace_z_min && z <= cfg_.workspace_z_max;
}

bool VisionGraspNode::callGatherDepth(int u, int v, int& depth_mm)
{
    if (!depth_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(get_logger(), "Service 'gather_depth' unavailable after 5s");
        return false;
    }
    auto request = std::make_shared<service_interfaces::srv::GatherDepth::Request>();
    request->pixel_u = u;
    request->pixel_v = v;

    auto future = depth_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
            service_node_, future, std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        if (!response->ok)
        {
            RCLCPP_WARN(get_logger(), "Depth request rejected: %s", response->msg.c_str());
            return false;
        }
        RCLCPP_INFO(get_logger(), "Depth response: depth=%.2f", response->depth);
        depth_mm = static_cast<int>(std::lround(response->depth * 1000.0));  // 米 → 毫米
        return true;
    }
    RCLCPP_ERROR(get_logger(), "Failed to call service 'gather_depth'!");
    return false;
}

bool VisionGraspNode::callTransformXyz(int u, int v, int depth_mm,
                                       double& x, double& y, double& z)
{
    if (!transform_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(get_logger(), "Service 'transform_xyz' unavailable after 5s");
        return false;
    }
    auto request = std::make_shared<service_interfaces::srv::TransformXyz::Request>();
    request->type = cfg_.arm_type;   // 单臂工程：0=左臂标定，1=右臂标定
    request->pixel_u = u;
    request->pixel_v = v;
    request->depth = depth_mm;

    auto future = transform_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
            service_node_, future, std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        if (!response->ok)
        {
            RCLCPP_WARN(get_logger(), "Transform request rejected: %s", response->msg.c_str());
            return false;
        }
        RCLCPP_INFO(get_logger(), "Transform response: x=%.2f, y=%.2f, z=%.2f",
                    response->end_x, response->end_y, response->end_z);
        x = response->end_x;
        y = response->end_y;
        z = response->end_z;
        return true;
    }
    RCLCPP_ERROR(get_logger(), "Failed to call service 'transform_xyz'!");
    return false;
}

void VisionGraspNode::processCandidate(const Candidate& candidate,
                                       int& stable_count, double& last_x,
                                       double& last_y, bool& latched)
{
    // 本帧没有合格目标：清空稳定计数
    if (!candidate.valid)
    {
        stable_count = 0;
        return;
    }

    // 计算目标相对上一帧的跳动距离，判断是否同一个目标
    const double jump = std::hypot(candidate.center_x - last_x,
                                   candidate.center_y - last_y);
    stable_count = (stable_count > 0 && jump <= cfg_.max_center_jump_px)
                       ? stable_count + 1
                       : 1;
    last_x = candidate.center_x;
    last_y = candidate.center_y;
    if (stable_count < cfg_.stable_frames || latched)
    {
        return;
    }

    // ---- 目标稳定，开始算坐标并转换为抓取任务 ----
    const int u = static_cast<int>(std::lround(candidate.center_x));
    const int v = static_cast<int>(std::lround(candidate.center_y));
    int depth_mm = 0;
    if (!callGatherDepth(u, v, depth_mm))
    {
        return;
    }
    double wx = 0.0, wy = 0.0, wz = 0.0;
    if (!callTransformXyz(u, v, depth_mm, wx, wy, wz))
    {
        return;
    }
    if (!withinWorkspace(wx, wy, wz))
    {
        RCLCPP_ERROR(get_logger(),
                     "target rejected outside workspace: (%.1f, %.1f, %.1f)mm",
                     wx, wy, wz);
        return;
    }

    // 目标合格：锁存并清空稳定计数，防止重复触发
    latched = true;
    stable_count = 0;
    RCLCPP_INFO(get_logger(),
                "grasp candidate: pixel=(%d,%d) depth=%dmm base=(%.1f,%.1f,%.1f)mm confidence=%.3f",
                u, v, depth_mm, wx, wy, wz, candidate.confidence);

    if (!cfg_.execute_enabled)
    {
        RCLCPP_WARN(get_logger(), "DRY RUN: controller command suppressed");
        return;
    }

    // ---- 真实执行模式：把目标写入共享状态，交给抓取线程 ----
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!grasp_busy_ && !target_.is_exist)
    {
        target_.is_exist = true;
        target_.x = wx;
        target_.y = wy;
        target_.z = wz;
    }
}

void VisionGraspNode::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    // 本帧候选目标（单臂只保留一个）
    Candidate selected;
    for (const auto& detection : msg->detections)
    {
        if (detection.results.empty())
        {
            continue;
        }
        const auto& hypothesis = detection.results[0].hypothesis;
        if (hypothesis.class_id != cfg_.target_class_id ||
            hypothesis.score < cfg_.min_confidence)
        {
            continue;
        }
        // 归一化中心坐标 × 图像宽高 → 像素坐标
        const double center_x = detection.bbox.center.position.x * cfg_.image_width;
        const double center_y = detection.bbox.center.position.y * cfg_.image_height;
        if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
            center_x < 0.0 || center_x >= cfg_.image_width ||
            center_y < 0.0 || center_y >= cfg_.image_height)
        {
            continue;
        }
        // 同一帧出现多个目标时保留置信度最高的
        if (!selected.valid || hypothesis.score > selected.confidence)
        {
            selected = {true, hypothesis.score, center_x, center_y};
        }
    }

    // 维护"目标消失帧数"，超限复位锁存，允许再次触发
    if (selected.valid)
    {
        missed_frames_ = 0;
    }
    else if (++missed_frames_ >= cfg_.release_frames)
    {
        latched_ = false;
    }

    processCandidate(selected, stable_count_, last_x_, last_y_, latched_);
}

void VisionGraspNode::computeApproach(double roll_deg, double pitch_deg, double yaw_deg,
                                      double& ax, double& ay, double& az) const
{
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)，工具接近轴 = R * [0,0,1]
    const double r = roll_deg * M_PI / 180.0;
    const double p = pitch_deg * M_PI / 180.0;
    const double y = yaw_deg * M_PI / 180.0;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    ax = cy * sp * cr + sy * sr;
    ay = sy * sp * cr - cy * sr;
    az = cp * cr;
}

bool VisionGraspNode::executeGrasp(const GraspTarget& target)
{
    if (arm_working_)
    {
        arm_working_->store(true);  // 上锁：机械臂作业中，禁止底盘移动
    }

    bool ok = true;
    if (!arm_ || !arm_->isConnected())
    {
        RCLCPP_ERROR(get_logger(), "arm not connected, grasp aborted");
        ok = false;
    }

    const auto& rpy = cfg_.grasp_rpy_deg;
    double r = rpy.size() > 0 ? rpy[0] : 0.0;
    double p = rpy.size() > 1 ? rpy[1] : 0.0;
    double y = rpy.size() > 2 ? rpy[2] : 0.0;

    // 预抓点 = 抓取点 - 工具接近方向 × 距离
    double ax = 0.0, ay = 0.0, az = 0.0;
    computeApproach(r, p, y, ax, ay, az);
    const double pg_x = target.x - ax * cfg_.pregrasp_dist_mm;
    const double pg_y = target.y - ay * cfg_.pregrasp_dist_mm;
    const double pg_z = target.z - az * cfg_.pregrasp_dist_mm;

    RCLCPP_INFO(get_logger(),
                "grasp target=(%.1f,%.1f,%.1f)mm pregrasp=(%.1f,%.1f,%.1f)mm rpy=(%.1f,%.1f,%.1f)deg",
                target.x, target.y, target.z, pg_x, pg_y, pg_z, r, p, y);

    // Step-1 回安全位
    if (ok)
    {
        ok = arm_->goSafeHome(cfg_.move_wait_ms);
    }
    // 预张开手（可选）
    if (ok && !cfg_.hand_open_cmd.empty())
    {
        ok = arm_->sendRawCmd(cfg_.hand_open_cmd);
    }
    // Step-2 直线到预抓点 → 直线到抓取点 → 合手（可选）
    if (ok)
    {
        ok = arm_->addCartPoint("vg_pregrasp", pg_x, pg_y, pg_z, r, p, y) &&
             arm_->moveCartL("vg_pregrasp", cfg_.move_wait_ms);
    }
    if (ok)
    {
        ok = arm_->addCartPoint("vg_grasp", target.x, target.y, target.z, r, p, y) &&
             arm_->moveCartL("vg_grasp", cfg_.move_wait_ms);
    }
    if (ok && !cfg_.hand_close_cmd.empty())
    {
        ok = arm_->sendRawCmd(cfg_.hand_close_cmd);
    }
    // Step-3 沿接近轴直线退回预抓点
    if (ok)
    {
        ok = arm_->moveCartL("vg_pregrasp", cfg_.move_wait_ms);
    }
    // Step-4 关节运动到释放位，放手（可选）
    if (ok && cfg_.release_joints.size() >= 6)
    {
        ok = arm_->addJointPoint("vg_release",
                                 cfg_.release_joints[0], cfg_.release_joints[1],
                                 cfg_.release_joints[2], cfg_.release_joints[3],
                                 cfg_.release_joints[4], cfg_.release_joints[5]) &&
             arm_->moveToPoint("vg_release", cfg_.move_wait_ms);
    }
    if (ok && !cfg_.hand_open_cmd.empty())
    {
        ok = arm_->sendRawCmd(cfg_.hand_open_cmd);
    }
    // Step-5 回安全位
    arm_->goSafeHome(cfg_.move_wait_ms);

    if (arm_working_)
    {
        arm_working_->store(false);  // 解锁：允许底盘行驶
    }
    return ok;
}

void VisionGraspNode::graspLoop()
{
    while (!stop_requested_ && rclcpp::ok())
    {
        GraspTarget t;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            if (target_.is_exist && !grasp_busy_)
            {
                grasp_busy_ = true;
                t = target_;
            }
        }
        if (t.is_exist)
        {
            bool success = executeGrasp(t);
            RCLCPP_INFO(get_logger(), "grasp finished: %s", success ? "OK" : "FAILED");
            {
                std::lock_guard<std::mutex> lock(target_mutex_);
                target_.is_exist = false;
            }
            grasp_busy_ = false;
            latched_ = false;  // 复位锁存，目标再次可见时可重新触发
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

}  // namespace agv
