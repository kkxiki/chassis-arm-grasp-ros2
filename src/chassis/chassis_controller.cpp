#include "chassis/chassis_controller.h"
#include "common/http_utils.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <curl/curl.h>

// ============================================================
// 构造函数：初始化参数
// ============================================================
ChassisController::ChassisController(const std::string& chassis_url, DispatchClient& dispatch_client)
    : base_url_(chassis_url)
    , dispatch_client_(dispatch_client)
    , move_speed_(0.3f)
    , rot_speed_(30.0f)
    , retry_count_(2)
    , cruising_(false)    // 原子变量初始化
    , pause_flag_(false)
    , cancel_flag_(false)
{
    
}
    

// ============================================================
// 析构函数
// ============================================================
ChassisController::~ChassisController() {
    // 退出时确保停止运动
    if (cruising_) {
        cancelCruise();
    }
}

// ============================================================
// 静态工具函数：从JSON文件加载航点列表
// ============================================================
bool ChassisController::loadWaypointsFromFile(const std::string& file_path, std::vector<Waypoint>& out_points)
{
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "[loadWaypointsFromFile] 无法打开文件: " << file_path << std::endl;
        return false;
    }
    try {
        nlohmann::json root;
        file >> root;
        out_points.clear();

        if (!root.contains("points") || !root["points"].is_array()) {
            std::cerr << "[loadWaypointsFromFile] 文件格式错误，缺少points数组" << std::endl;
            return false;
        }

        for (const auto& p : root["points"]) {
            Waypoint wp;
            wp.x = p.value("x", 0.0f);
            wp.y = p.value("y", 0.0f);
            wp.angle = p.value("angle", 0.0f);
            wp.stay_ms = p.value("stay_ms", 0);
            wp.action_points.clear();

            // 解析动作点位列表
            if (p.contains("action_points") && p["action_points"].is_array()) {
                for (const auto& ap : p["action_points"]) {
                    wp.action_points.push_back(ap.get<std::string>());
                }
            }
            out_points.push_back(wp);
        }
        std::cout << "[loadWaypointsFromFile] 加载成功，共" << out_points.size() << "个航点" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[loadWaypointsFromFile] JSON解析异常: " << e.what() << std::endl;
        return false;
    }
}


// ============================================================
// 设置运动速度参数
// ============================================================
void ChassisController::setSpeed(float move_speed, float rot_speed) {
    std::lock_guard<std::mutex> lock(cruise_mutex_);
    move_speed_ = move_speed;
    rot_speed_ = rot_speed;
}

// ============================================================
// 设置指令失败重试次数
// ============================================================
void ChassisController::setRetryCount(int count) {
    std::lock_guard<std::mutex> lock(cruise_mutex_);
    retry_count_ = count;
}

// ============================================================
// 注册航点到达回调函数
// ============================================================
void ChassisController::setWaypointArrivedCallback(WaypointArrivedCallback callback) {
    std::lock_guard<std::mutex> lock(cruise_mutex_);
    arrived_callback_ = callback;
}

// ============================================================
// 查询是否正在巡航（线程安全）
// ============================================================
bool ChassisController::isCruising() const {
    // 原子读取，无需加锁
    return cruising_.load();
}

// ============================================================
// 获取当前机器人位姿
// ============================================================
int ChassisController::getCurrentPose(float& x, float& y, float& angle) {
    bool is_fault = false;
    if (!dispatch_client_.getRobotPoseSafe(x, y, angle, is_fault)) {
        return CHASSIS_ERR_POSE_FAILED;
    }
    if (is_fault) {
        return CHASSIS_ERR_FAULT;
    }
    return CHASSIS_OK;
}

// ============================================================
// 调度模式：底盘原地旋转
// ============================================================
int ChassisController::rotateAngle(float angle, float speed) {
    std::vector<std::string> params;
    params.push_back("speed:" + std::to_string(speed));
    params.push_back("angle:" + std::to_string(angle));
    return dispatch_client_.startActionByType(13, params);
}

// ============================================================
// 调度模式：底盘相对平移
// ============================================================
int ChassisController::translateDistance(float angle, int distance_mm, float speed) {
    std::vector<std::string> params;
    params.push_back("speed:" + std::to_string(speed));
    params.push_back("angle:" + std::to_string(angle));
    params.push_back("distance:" + std::to_string(distance_mm));
    return dispatch_client_.startActionByType(8, params);
}

// ============================================================
// 【新增】直控模式：启动移动到目标点
// 对应文档接口：/startMoveController
// ============================================================
int ChassisController::startMoveTo(float target_x, float target_y, float target_angle,
                                   float speed, int way_type) {
    // 1. 获取当前位置作为起点
    float start_x, start_y, start_angle;
    int ret = getCurrentPose(start_x, start_y, start_angle);
    if (ret != CHASSIS_OK) {
        std::cerr << "[startMoveTo] 获取当前位姿失败" << std::endl;
        return CHASSIS_ERR_SEND_FAILED;
    }

    // 2. 构造请求体，严格按照接口文档字段
    json body;
    body["px"] = target_x;          // 目标点x坐标
    body["py"] = target_y;          // 目标点y坐标
    body["angle"] = target_angle;   // 目标点角度
    body["speed"] = speed;          // 移动速度
    body["wayType"] = way_type;     // 通道类型
    body["navScoreLow"] = 0.2f;     // 定位置信度阈值
    body["type"] = 0;               // 目标点类型：0=作业点
    body["isContinue"] = false;     // 是否继续上次任务

    // 目标点编号与ID（用简单自增或随机值，保证唯一即可）
    static int target_seq = 0;
    target_seq++;
    body["targetCode"] = "tar_" + std::to_string(target_seq);
    body["targetId"] = target_seq;

    // 起点信息
    body["startId"] = 0;            // 路中启动，起点ID填0
    body["startX"] = start_x;       // 起点x坐标
    body["startY"] = start_y;       // 起点y坐标

    // 途径点：暂无，填空数组
    body["path"] = json::array();
    body["pathIndex"] = -1;         // 无途径点填-1

    // 3. 发送请求
    json resp = chassisHttpPost("/startMoveController", body);
    if (!checkChassisResponse(resp)) {
        std::cerr << "[startMoveTo] 启动移动失败: " << resp.value("msg", "unknown") << std::endl;
        return CHASSIS_ERR_SEND_FAILED;
    }

    return CHASSIS_OK;
}

// ============================================================
// 【新增】直控模式：停止移动
// 对应文档接口：/stopMoveController
// ============================================================
int ChassisController::stopMove() {
    json resp = chassisHttpPost("/stopMoveController", json::object());
    if (!checkChassisResponse(resp)) {
        std::cerr << "[stopMove] 停止移动失败" << std::endl;
        return CHASSIS_ERR_SEND_FAILED;
    }
    return CHASSIS_OK;
}

// ============================================================
// 【新增】直控模式：查询移动是否启动
// 对应文档接口：/isMoveStarted
// ============================================================
int ChassisController::isMoveStarted(bool& out_started) {
    json resp = chassisHttpGet("/isMoveStarted");
    if (!checkChassisResponse(resp)) {
        return CHASSIS_ERR_SEND_FAILED;
    }

    // data字段返回字符串"true"或"false"
    std::string data_str = resp.value("data", "false");
    out_started = (data_str == "true");
    return CHASSIS_OK;
}

// ============================================================
// 【新增】直控模式：启动旋转
// 对应文档接口：/startRotateRobot
// ============================================================
int ChassisController::startRotate(float angle, bool is_nav_angle, float speed) {
    json body;
    body["angle"] = angle;
    body["isNavAngle"] = is_nav_angle;
    body["speed"] = speed;

    json resp = chassisHttpPost("/startRotateRobot", body);
    if (!checkChassisResponse(resp)) {
        std::cerr << "[startRotate] 启动旋转失败" << std::endl;
        return CHASSIS_ERR_SEND_FAILED;
    }
    return CHASSIS_OK;
}

// ============================================================
// 【新增】直控模式：停止旋转
// ============================================================
int ChassisController::stopRotate() {
    json resp = chassisHttpPost("/stopRotateRobot", json::object());
    return checkChassisResponse(resp) ? CHASSIS_OK : CHASSIS_ERR_SEND_FAILED;
}

// ============================================================
// 【新增】直控模式：暂停旋转
// ============================================================
int ChassisController::pauseRotate() {
    json resp = chassisHttpPost("/pauseRotateRobot", json::object());
    return checkChassisResponse(resp) ? CHASSIS_OK : CHASSIS_ERR_SEND_FAILED;
}

// ============================================================
// 【新增】直控模式：继续旋转
// ============================================================
int ChassisController::resumeRotate() {
    json resp = chassisHttpPost("/resumeRotateRobot", json::object());
    return checkChassisResponse(resp) ? CHASSIS_OK : CHASSIS_ERR_SEND_FAILED;
}

// ============================================================
// 移动到单个指定位姿（内部调用巡航实现）
// ============================================================
int ChassisController::goToPose(float x, float y, float angle, bool check_angle) {
    std::vector<Waypoint> single_wp;
    Waypoint wp;
    wp.x = x;
    wp.y = y;
    wp.angle = angle;
    wp.stay_ms = 0;
    single_wp.push_back(wp);
    return runCruise(single_wp, check_angle);
}

// ============================================================
// 返回原点位置
// ============================================================
int ChassisController::goHome(float home_x, float home_y, float home_angle, bool check_angle) {
    return goToPose(home_x, home_y, home_angle, check_angle);
}

// ============================================================
// 暂停巡航任务
// ============================================================
int ChassisController::pauseCruise() {
    if (!cruising_.load()) {
        return CHASSIS_ERR_STATE_INVALID;
    }
    // 原子操作：如果已经是暂停状态，直接返回成功，不重复调用stopMove
    if (pause_flag_.exchange(true)) {
        return CHASSIS_OK;
    }
    // 暂停时停止底盘移动
    stopMove();
    return CHASSIS_OK;
}

// ============================================================
// 恢复巡航任务
// ============================================================
int ChassisController::resumeCruise() {
    if (!cruising_.load() || !pause_flag_.load()) {
        return CHASSIS_ERR_STATE_INVALID;
    }
    pause_flag_ = false;
    return CHASSIS_OK;
}

// ============================================================
// 取消巡航任务
// ============================================================
int ChassisController::cancelCruise() {
    cancel_flag_ = true;
    pause_flag_ = false;
    // 取消时停止底盘移动
    stopMove();
    return CHASSIS_OK;
}

// ============================================================
// 执行多航点巡航（核心阻塞函数，直控模式）
// 逻辑：逐航点下发移动指令 → 轮询位姿判断到位 → 停止 → 停留 → 回调 → 下一航点
// ============================================================
int ChassisController::runCruise(const std::vector<Waypoint>& waypoints, bool check_angle) {
    // 前置校验
    if (waypoints.empty()) {
        std::cerr << "[runCruise] 航点列表为空" << std::endl;
        return CHASSIS_ERR_STATE_INVALID;
    }

    // 设置巡航运行状态（原子操作，防止重复启动）
    bool expected = false;
    if (!cruising_.compare_exchange_strong(expected, true)) {
        return CHASSIS_ERR_STATE_INVALID;
    }
    pause_flag_ = false;
    cancel_flag_ = false;

    std::cout << "=== 直控巡航开始，共" << waypoints.size() << "个航点 ===" << std::endl;

    // 依次执行每个航点
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const Waypoint& target_wp = waypoints[i];
        std::cout << "\n--- 前往第" << i+1 << "个航点: (" 
                  << target_wp.x << ", " << target_wp.y << ", " << target_wp.angle << "°) ---" << std::endl;

        // ========== 1. 下发移动指令 ==========
        int move_ret = startMoveTo(target_wp.x, target_wp.y, target_wp.angle, move_speed_);
        if (move_ret != CHASSIS_OK) {
            std::cerr << "[runCruise] 下发移动指令失败，终止巡航" << std::endl;
            cruising_ = false;
            return CHASSIS_ERR_SEND_FAILED;
        }

                // ========== 2. 轮询等待到位 ==========
        bool arrived = false;
        int timeout_count = 0;
        const int check_interval_ms = 200;

        while (!arrived) {
            // ========== 第一步：检查是否进入暂停状态 ==========
            if (pause_flag_.load()) {
                std::cout << "  进入暂停等待状态" << std::endl;
                // 暂停循环：一直等到暂停标志被清除或任务被取消
                while (pause_flag_.load()) {
                    // 暂停中也要响应取消指令
                    if (cancel_flag_.load()) {
                        stopMove();
                        cruising_ = false;
                        std::cout << "[runCruise] 巡航被取消" << std::endl;
                        return CHASSIS_ERR_CANCELED;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
                }

                // ========== 核心修复：退出暂停循环=恢复运行，立刻重发移动指令 ==========
                // 不需要依赖状态变量判断，只要从暂停里出来就必然要重新启动底盘
                std::cout << "  检测到暂停恢复，重新下发移动指令..." << std::endl;
                int move_ret = startMoveTo(target_wp.x, target_wp.y, target_wp.angle, move_speed_);
                if (move_ret != CHASSIS_OK) {
                    std::cerr << "  ❌ 恢复移动失败，错误码: " << move_ret << std::endl;
                } else {
                    std::cout << "  ✅ 恢复移动指令下发成功" << std::endl;
                }
            }

            // ========== 第二步：非暂停状态也要检查取消指令 ==========
            if (cancel_flag_.load()) {
                stopMove();
                cruising_ = false;
                std::cout << "[runCruise] 巡航被取消" << std::endl;
                return CHASSIS_ERR_CANCELED;
            }

            // ========== 第三步：正常获取位姿、判断到位 ==========
        
            // 获取当前位姿
            float curr_x, curr_y, curr_angle;
            bool is_fault = false;
            if (!dispatch_client_.getRobotPoseSafe(curr_x, curr_y, curr_angle, is_fault)) {
                timeout_count++;
                if (timeout_count > 50) {
                    stopMove();
                    cruising_ = false;
                    std::cerr << "[runCruise] 连续获取位姿失败" << std::endl;
                    return CHASSIS_ERR_POSE_FAILED;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
                continue;
            }

            // 检查底盘故障/急停
            if (is_fault) {
                stopMove();
                cruising_ = false;
                std::cerr << "[runCruise] 底盘触发急停/故障" << std::endl;
                return CHASSIS_ERR_FAULT;
            }

            // 计算位置误差
            float pos_error = std::sqrt(std::pow(curr_x - target_wp.x, 2) + std::pow(curr_y - target_wp.y, 2));
            
            // 计算角度误差（归一化）
            float ang_error = std::abs(curr_angle - target_wp.angle);
            if (ang_error > 180.0f) {
                ang_error = 360.0f - ang_error;
            }

            // 判断到位
            bool pos_ok = pos_error < position_threshold_;
            bool ang_ok = !check_angle || ang_error < angle_threshold_;
            if (pos_ok && ang_ok) {
                arrived = true;
                // 到位后立即停止底盘
                stopMove();
                std::cout << "  精准到位，位置误差:" << pos_error << "m 角度误差:" << ang_error << "°" << std::endl;
                break;
            }

            timeout_count++;
            // 超时判断
            if (timeout_count > total_timeout_s_ * 1000 / check_interval_ms) {
                stopMove();
                cruising_ = false;
                std::cerr << "[runCruise] 运动超时，当前位置误差:" << pos_error << "m" << std::endl;
                return CHASSIS_ERR_TIMEOUT;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
        }

        // ========== 3. 到位后处理 ==========
        // 触发回调
        if (arrived_callback_) {
            arrived_callback_(i, target_wp);
        }

        // 停留指定时间
        if (target_wp.stay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(target_wp.stay_ms));
        }
    }

    // 全部航点完成
    cruising_ = false;
    std::cout << "\n=== 全部航点执行完成 ===" << std::endl;
    return CHASSIS_OK;
}

// ============================================================
// 底盘控制专用 HTTP GET 请求
// ============================================================
json ChassisController::chassisHttpGet(const std::string& path) {
    std::string url = base_url_ + path;
    return HttpUtils::httpGet(url, connect_timeout_s_, total_timeout_s_);
}

// ============================================================
// 底盘控制专用 HTTP POST 请求
// ============================================================
json ChassisController::chassisHttpPost(const std::string& path, const json& body) {
    std::string url = base_url_ + path;
    return HttpUtils::httpPost(url, body, connect_timeout_s_, total_timeout_s_);
}

// ============================================================
// 检查直控接口返回是否成功
// 注意：直控接口成功码为2000，与调度接口的0不同
// ============================================================
bool ChassisController::checkChassisResponse(const json& resp) const {
    int code = resp.value("code", -1);
    return code == 2000;
}