#ifndef VISION_GRASP_H
#define VISION_GRASP_H

#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "service_interfaces/srv/gather_depth.hpp"
#include "service_interfaces/srv/transform_xyz.hpp"
#include "nlohmann/json.hpp"
#include "arm/arm_direct_controller.h"

namespace agv {

// ============================================================
// 视觉抓取配置（对应 config.json 中的 "vision" 段）
// 数值含义与备份工程 legacy_grasp_ws 的 pipeline_dry_run.yaml 一致
// ============================================================
struct VisionConfig {
    // 订阅与检测
    std::string detections_topic = "/yolov8/detections";  // YOLO 检测结果话题
    std::string target_class_id = "39";                   // 要抓的目标类别（COCO 水瓶=39）
    double min_confidence = 0.40;                         // 置信度阈值
    int image_width = 1280;                               // 图像宽（像素）
    int image_height = 720;                               // 图像高（像素）

    // 单臂工程：固定使用一套手眼标定（0=左臂标定，1=右臂标定）
    int arm_type = 0;
    double center_deadband_px = 40.0;   // 图像中线死区（像素）

    // 稳定帧 / 释放帧逻辑（防止误检与重复抓取）
    int stable_frames = 5;              // 连续稳定 N 帧才触发抓取
    int release_frames = 10;            // 目标消失 N 帧后才允许再次触发
    double max_center_jump_px = 15.0;   // 两帧间中心跳动阈值（像素）

    // 是否真实执行机械臂动作（false=干跑：只计算坐标不发运动指令）
    bool execute_enabled = false;

    // 工作区范围（毫米）：换算坐标超出范围的目标被拒绝
    double workspace_x_min = 0.0,   workspace_x_max = 1000.0;
    double workspace_y_min = -700.0, workspace_y_max = 700.0;
    double workspace_z_min = -300.0, workspace_z_max = 500.0;

    // ========== 抓取动作参数 ==========
    double pregrasp_dist_mm = 200.0;                    // 预抓点距抓取点的距离（沿工具接近轴）
    std::vector<double> grasp_rpy_deg = {-102.7, -39.0, -78.6}; // 抓取姿态 RPY（度），现场需重新示教确认
    std::vector<double> safe_home_joints = {0.0, -90.0, 0.0, -90.0, 0.0, 0.0};  // 安全位（启动/结束回此位）
    std::vector<double> release_joints  = {0.0, -90.0, 0.0, -90.0, 0.0, 0.0};   // 释放位（放下水瓶）
    std::string hand_close_cmd;     // 可选：抓取后发送的抓手指令（空=跳过，如 {SetHandState --state=0 --hand_id=0}）
    std::string hand_open_cmd;      // 可选：释放前发送的抓手指令（空=跳过）
    int move_wait_ms = 3000;        // 单步运动等待时间（毫秒）
};

// 从 JSON 对象加载视觉配置（缺失字段使用默认值）
bool loadVisionConfig(const nlohmann::json& cfg, VisionConfig& out);

// ============================================================
// 视觉抓取节点（移植自 legacy_grasp_ws 的 yolo_result_subscriber.cpp）
// 流程：
//   1. 订阅 /yolov8/detections，回调内筛选目标类别与置信度；
//   2. 连续稳定帧确认 + 目标消失释放帧防重复；
//   3. 调用 gather_depth 服务取深度、transform_xyz 服务换算机械臂基座坐标；
//   4. 检查工作区后，把抓取任务交给抓取线程；
//   5. 抓取线程通过 ArmDirectController 执行：预抓→抓取→退回→释放→回零。
// ============================================================
class VisionGraspNode : public rclcpp::Node {
public:
    VisionGraspNode(const VisionConfig& cfg,
                    ArmDirectController* arm,
                    std::atomic<bool>* arm_working = nullptr);
    ~VisionGraspNode() override;

    bool execute_enabled() const { return cfg_.execute_enabled; }
    // 请求停止抓取循环（线程安全）
    void requestStop();

private:
    // 单帧候选目标
    struct Candidate {
        bool valid{false};
        double confidence{0.0};
        double center_x{0.0};
        double center_y{0.0};
    };
    // 待抓目标（机械臂基座坐标，毫米）
    struct GraspTarget {
        bool is_exist{false};
        double x{0.0}, y{0.0}, z{0.0};
    };

    // 是否在合法工作区内
    bool withinWorkspace(double x, double y, double z) const;
    // 查询像素深度（毫米）
    bool callGatherDepth(int u, int v, int& depth_mm);
    // 像素+深度 → 机械臂基座坐标（毫米）
    bool callTransformXyz(int u, int v, int depth_mm, double& x, double& y, double& z);
    // 稳定确认并派发抓取任务
    void processCandidate(const Candidate& cand,
                          int& stable_count, double& last_x, double& last_y, bool& latched);
    // YOLO 检测结果回调
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    // 抓取线程主循环（移植自 robot_larm_loop）
    void graspLoop();
    // 执行一次完整抓取序列
    bool executeGrasp(const GraspTarget& target);
    // 由 RPY(度) 计算工具接近方向单位向量（R = Rz(yaw)*Ry(pitch)*Rx(roll)）
    void computeApproach(double roll_deg, double pitch_deg, double yaw_deg,
                         double& ax, double& ay, double& az) const;

    VisionConfig cfg_;
    ArmDirectController* arm_;
    std::atomic<bool>* arm_working_;  // 底盘互斥标志（可为空）

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
    // 服务客户端使用独立节点，避免在回调内自旋执行器造成重入
    rclcpp::Node::SharedPtr service_node_;
    rclcpp::Client<service_interfaces::srv::GatherDepth>::SharedPtr depth_client_;
    rclcpp::Client<service_interfaces::srv::TransformXyz>::SharedPtr transform_client_;

    // 稳定性状态
    int stable_count_{0};
    int missed_frames_{0};
    double last_x_{0.0}, last_y_{0.0};
    bool latched_{false};

    // 抓取任务共享状态
    std::mutex target_mutex_;
    GraspTarget target_;
    std::atomic<bool> grasp_busy_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread grasp_thread_;
};

}  // namespace agv

#endif  // VISION_GRASP_H
