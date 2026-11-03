#ifndef AGV_GRASP_MISSION_H
#define AGV_GRASP_MISSION_H

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "arm/arm_direct_controller.h"
#include "chassis/chassis_controller.h"
#include "vision/vision_grasp.h"

namespace agv {

struct GraspMissionConfig {
    bool execute_enabled{false};
    bool execute_chassis{false};
    bool execute_arm{false};
    bool execute_gripper{false};
    double site_x_m{0.0};
    double site_y_m{0.0};
    double site_yaw_deg{0.0};

    double base_to_chassis_x_mm{0.0};
    double base_to_chassis_y_mm{0.0};
    double base_to_chassis_yaw_deg{0.0};
    double approach_sweet_x_mm{500.0};
    double approach_sweet_y_mm{0.0};
    int max_approach_iterations{3};
    double max_approach_step_mm{400.0};
    double approach_tolerance_mm{30.0};
    float approach_speed_mps{0.1f};
    int approach_angle_sign{1};  // 只能为 +1 或 -1；空载验收后确定
    double approach_step_check_ratio{0.6};  // S3 单步有效性判据阈值，无单位

    int fine_iterations{2};
    double max_fine_correction_mm{60.0};
    // 表面→中心补偿半径，mm。取 0 意味着系统性偏近约一个物体半径；抓 500ml 瓶填 32.0。
    double object_radius_mm{0.0};
    std::string wrist_camera_info_topic{"/wrist/camera/color/camera_info"};
    std::array<double, 4> wrist_camera_intrinsics{615.0, 615.0, 640.0, 360.0};
    std::array<double, 3> gripper_to_camera_translation_mm{0.0, 0.0, 0.0};
    std::array<double, 3> gripper_to_camera_rpy_deg{0.0, 0.0, 0.0};
    std::array<double, 3> tool_grasp_offset_mm{0.0, 0.0, 0.0};

    double pregrasp_dist_mm{200.0};
    std::array<double, 3> grasp_rpy_deg{-102.7, -39.0, -78.6};
    std::array<double, 6> release_joints{0.0, -90.0, 0.0, -90.0, 0.0, 0.0};
    std::array<double, 6> safe_home_joints{0.0, -90.0, 0.0, -90.0, 0.0, 0.0};
    std::string hand_close_cmd{"{MoveTwoFingersGripper --distance=0 --interval=25}"};
    std::string hand_open_cmd{"{MoveTwoFingersGripper --distance=100 --interval=25}"};
    int move_wait_ms{3000};
    int hand_wait_ms{900};
};

bool loadGraspMissionConfig(const nlohmann::json& cfg, GraspMissionConfig& out);

class GraspMission {
public:
    enum class State {
        IDLE, NAV_TO_SITE, COARSE_DETECT, CHASSIS_APPROACH, COARSE_RECHECK,
        ARM_PREGRASP, FINE_DETECT, ARM_GRASP, GRIP_CLOSE, RETREAT_RELEASE,
        ABORTED, COMPLETED
    };

    GraspMission(const GraspMissionConfig& cfg,
                 ChassisController& chassis,
                 ArmDirectController* arm,
                 VisionGraspNode& coarse_provider,
                 VisionGraspNode& wrist_provider,
                 std::atomic<bool>* arm_working);
    ~GraspMission();

    bool start();
    void requestAbort(const std::string& reason);
    State state() const;
    std::string lastReason() const;
    bool isRunning() const;
    rclcpp::Node::SharedPtr cameraInfoNode() const;

private:
    struct PoseMmDeg {
        double x{0.0}, y{0.0}, z{0.0};
        double roll{0.0}, pitch{0.0}, yaw{0.0};
    };
    void run();
    void abort(const std::string& reason);
    bool takeCoarseTarget(VisionGraspNode::ProviderTarget& out);
    bool takeWristTarget(VisionGraspNode::ProviderTarget& out);
    bool performApproach(const VisionGraspNode::ProviderTarget& target);
    bool movePregrasp();
    bool fineCorrect();
    bool moveGrasp();
    bool closeGripper();
    bool retreatRelease();
    // 不调用 ArmDirectController::goSafeHome()；该接口忽略配置。
    bool moveMissionSafeHome();
    bool commandMoveL(const std::string& name, const PoseMmDeg& pose);
    bool chassisMayMove() const;
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    PinholeIntrinsics currentIntrinsics() const;
    static void rpyToMatrix(double roll_deg, double pitch_deg, double yaw_deg,
                            std::array<std::array<double, 3>, 3>& out);

    GraspMissionConfig cfg_;
    ChassisController& chassis_;
    ArmDirectController* arm_;
    VisionGraspNode& coarse_provider_;
    VisionGraspNode& wrist_provider_;
    std::atomic<bool>* arm_working_;
    std::atomic<bool> running_{false};
    std::atomic<bool> abort_requested_{false};
    std::atomic<State> state_{State::IDLE};
    mutable std::mutex mutex_;
    std::string abort_reason_;
    bool holding_{false};
    int approach_iterations_{0};
    double last_approach_distance_before_mm_{0.0};
    double last_approach_cmd_mm_{0.0};
    PoseMmDeg pregrasp_pose_;
    PoseMmDeg grasp_pose_;
    PinholeIntrinsics camera_intrinsics_;
    rclcpp::Node::SharedPtr camera_info_node_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::thread worker_;
};

}  // namespace agv

#endif  // AGV_GRASP_MISSION_H
