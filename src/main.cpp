#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <csignal>
#include <fstream>
#include <string>
#include <algorithm>
#include <atomic>
#include <sstream>
#include <map>
#include "nlohmann/json.hpp"
#include "common/http_utils.h"
#include "dispatch/dispatch_client.h"
#include "chassis/chassis_controller.h"
#include "arm/arm_controller.h"
#include "task/task_manager.h"
#include "arm/arm_direct_controller.h"
#include "vision/vision_grasp.h"

using json = nlohmann::json;

// ==================== 全局状态标志 ====================
volatile bool g_should_quit = false;          // 程序退出信号（Ctrl+C / q命令）
std::atomic<bool> g_input_thread_exit = false; // 输入线程退出标志
std::atomic<bool> g_direct_cruise_running = false; // 直控巡航是否正在运行
std::atomic<bool> g_schedule_task_running = false; // 调度任务是否正在运行
std::shared_ptr<ArmDirectController> g_arm_direct = nullptr;

// ========== 新增：底盘-机械臂互斥保护标志 ==========
// true = 机械臂作业中，禁止底盘任何移动；false = 空闲，允许底盘行驶
std::atomic<bool> g_arm_working{false};

// ========== 视觉抓取（rclcpp 节点）全局状态 ==========
std::atomic<bool> g_vision_started{false};  // 视觉节点是否已启动
std::shared_ptr<agv::VisionGraspNode> g_vision_node = nullptr;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_vision_executor = nullptr;

/**
 * @brief Ctrl+C 信号处理函数
 */
void signalHandler(int signum) {
    std::cout << "\n检测到中断信号，正在停止所有运动并退出..." << std::endl;
    g_should_quit = true;
}

/**
 * @brief 字符串转小写
 */
std::string toLowerString(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    return str;
}

/**
 * @brief 全局配置结构体
 */
struct AppConfig {
    std::string dispatch_url = "http://192.168.30.200:8008";
    std::string chassis_url  = "http://192.168.30.200:8767";
    int robot_id = 227;
    float move_speed = 0.3f;
    float rot_speed  = 30.0f;
    int retry_count  = 2;
    float home_x     = 0.0f;
    float home_y     = 0.0f;
    float home_angle = 0.0f;
    std::string task_file = "/home/robot/robot_ws/config/task_list.json";
    std::string waypoint_file = "/home/robot/robot_ws/config/waypoints.json";
    agv::VisionConfig vision;  // 视觉抓取配置
};

/**
 * @brief 加载全局配置
 */
bool loadAppConfig(const std::string& file_path, AppConfig& out_config) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "[配置加载] 无法打开配置文件: " << file_path << "，使用默认参数" << std::endl;
        return false;
    }
    try {
        json cfg;
        file >> cfg;
        out_config.dispatch_url = cfg.value("dispatch_url", out_config.dispatch_url);
        out_config.chassis_url  = cfg.value("chassis_url", out_config.chassis_url);
        out_config.robot_id     = cfg.value("robot_id", out_config.robot_id);
        out_config.move_speed   = cfg.value("move_speed", out_config.move_speed);
        out_config.rot_speed    = cfg.value("rot_speed", out_config.rot_speed);
        out_config.retry_count  = cfg.value("retry_count", out_config.retry_count);
        if (cfg.contains("home_pose") && cfg["home_pose"].is_object()) {
            const auto& home = cfg["home_pose"];
            out_config.home_x     = home.value("x", out_config.home_x);
            out_config.home_y     = home.value("y", out_config.home_y);
            out_config.home_angle = home.value("angle", out_config.home_angle);
        }
        // 视觉抓取配置（vision 段缺失时使用默认值）
        agv::loadVisionConfig(cfg.contains("vision") ? cfg["vision"] : json(), out_config.vision);
        std::cout << "[配置加载] 成功加载配置文件: " << file_path << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[配置加载] 解析失败: " << e.what() << "，使用默认参数" << std::endl;
        return false;
    }
}

/**
 * @brief 启动视觉抓取（rclcpp 节点 + 独立执行器线程）
 * @note 仅允许启动一次（rclcpp::init 只能调用一次）；停止后再启动需重启程序
 */
void startVisionGrasp(const agv::VisionConfig& vcfg) {
    if (g_vision_started.load()) {
        std::cout << "[视觉] ❌ 视觉抓取已在运行中" << std::endl;
        return;
    }
    // rclcpp::init 整个进程只能调用一次（shutdown 后不可再次 init）
    static bool rclcpp_initialized = false;
    if (rclcpp_initialized) {
        std::cout << "[视觉] ❌ rclcpp 已初始化过，停止后需重启程序才能再次启动视觉抓取" << std::endl;
        return;
    }
    if (g_schedule_task_running.load() || g_direct_cruise_running.load()) {
        std::cout << "[视觉] ❌ 有调度/直控任务正在运行，请先停止" << std::endl;
        return;
    }
    if (!g_arm_direct || !g_arm_direct->isConnected()) {
        std::cout << "[视觉] ❌ TCP直连机械臂未连接，无法启动视觉抓取" << std::endl;
        return;
    }

    try {
        rclcpp::init(0, nullptr);
        rclcpp_initialized = true;
    } catch (const std::exception& e) {
        std::cout << "[视觉] ❌ rclcpp 初始化失败: " << e.what() << std::endl;
        return;
    }
    g_vision_node = std::make_shared<agv::VisionGraspNode>(vcfg, g_arm_direct.get(), &g_arm_working);
    g_vision_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    g_vision_executor->add_node(g_vision_node);
    g_vision_started.store(true);

    std::thread([executor = g_vision_executor]() {
        executor->spin();
    }).detach();

    std::cout << "[视觉] ✅ 视觉抓取已启动，订阅话题: " << vcfg.detections_topic
              << "  模式: " << (vcfg.execute_enabled ? "真实执行" : "干跑(只算坐标)") << std::endl;
}

/**
 * @brief 停止视觉抓取
 */
void stopVisionGrasp() {
    if (!g_vision_started.load()) return;

    if (g_vision_node) g_vision_node->requestStop();
    if (g_vision_executor) {
        g_vision_executor->cancel();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    // 析构会 join 抓取线程
    g_vision_node.reset();
    g_vision_executor.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    g_vision_started.store(false);
    std::cout << "[视觉] ⏹️ 视觉抓取已停止" << std::endl;
}

/**
 * @brief 直控巡航后台线程函数
 */
void directCruiseThread(ChassisController& chassis, const std::vector<Waypoint>& waypoints) {
    // ========== 新增：互斥检查 ==========
    if (g_arm_working.load()) {
        std::cerr << "[直控] ❌ 机械臂正在作业，禁止底盘移动" << std::endl;
        g_direct_cruise_running = false;
        return;
    }

    g_direct_cruise_running = true;
    std::cout << "\n[直控模式] 开始执行多航点巡航，共" << waypoints.size() << "个航点" << std::endl;
    
    int ret = chassis.runCruise(waypoints, true);
    
    if (ret == CHASSIS_OK) {
        std::cout << "[直控模式] ✅ 巡航执行完成" << std::endl;
    } else if (ret == CHASSIS_ERR_CANCELED) {
        std::cout << "[直控模式] ⏹️ 巡航被手动停止" << std::endl;
    } else {
        std::cerr << "[直控模式] ❌ 巡航执行失败，错误码: " << ret << std::endl;
    }
    g_direct_cruise_running = false;
}

/**
 * @brief 调度任务后台线程函数
 */
void scheduleTaskThread(TaskManager& task_manager, int map_id, 
                        const std::vector<int>& site_ids, 
                        const std::vector<int>& action_ids) {
    g_schedule_task_running = true;
    std::cout << "\n[调度模式] 任务开始执行，共" << site_ids.size() << "个站点" << std::endl;
    
    task_manager.runTask(map_id, site_ids, action_ids);
    
    std::cout << "\n[调度模式] 任务已结束" << std::endl;
    g_schedule_task_running = false;
}

/**
 * @brief 控制台输入监听线程函数
 */
void consoleInputThread(TaskManager& task_manager, 
                        ChassisController& chassis, 
                        DispatchClient& dispatch_client,
                        const AppConfig& app_config,
                        int map_id,
                        const std::vector<int>& site_ids,
                        const std::vector<int>& action_ids){
    std::cout << "\n========== 终端控制命令 ==========" << std::endl;
    std::cout << "===== 调度任务模式 =====" << std::endl;
    std::cout << "  start           → 启动调度巡检任务" << std::endl;
    std::cout << "  p / pause       → 暂停调度航行" << std::endl;
    std::cout << "  r / resume      → 恢复调度航行" << std::endl;
    std::cout << "  s / stop        → 停止调度任务（不退出程序）" << std::endl;
    std::cout << "===== 直控测试模式 =====" << std::endl;
    std::cout << "  goto x y angle  → 直控移动到指定坐标(米/度)" << std::endl;
    std::cout << "                   示例: goto -2.6 -1.7 80" << std::endl;
    std::cout << "  cruise          → 加载航点文件执行多航点巡航" << std::endl;
    std::cout << "  pause_move      → 暂停直控巡航" << std::endl;
    std::cout << "  resume_move     → 恢复直控巡航" << std::endl;
    std::cout << "  stop_move       → 停止直控移动/巡航" << std::endl;
    std::cout << "  home            → 直控返回原点" << std::endl;
    std::cout << "  arm_cruise      → 带机械臂动作的直控坐标巡航" << std::endl;
    std::cout << "===== 视觉抓取模式 =====" << std::endl;
    std::cout << "  vision          → 启动视觉抓取（订阅 YOLO 检测结果，模式由配置决定）" << std::endl;
    std::cout << "  vision_stop     → 停止视觉抓取" << std::endl;
    std::cout << "===== 通用命令 =====" << std::endl;
    std::cout << "  h / help        → 查看帮助" << std::endl;
    std::cout << "  q / quit        → 停止所有运动并退出程序" << std::endl;
    std::cout << "==================================\n" << std::endl;
    std::cout << "💡 提示：程序启动后默认空闲，输入 start 启动调度任务" << std::endl;
    std::cout << "💡 测直控建议先不要启动调度，终端无刷屏，输入清晰可见\n" << std::endl;

    std::string input_line;
    while (!g_input_thread_exit) {
        std::getline(std::cin, input_line);
        std::string cmd_line = toLowerString(input_line);
        cmd_line.erase(0, cmd_line.find_first_not_of(" \t\r\n"));
        cmd_line.erase(cmd_line.find_last_not_of(" \t\r\n") + 1);
        if (cmd_line.empty()) continue;

        std::istringstream iss(cmd_line);
        std::string cmd;
        iss >> cmd;

        // ==================== 调度模式命令 ====================
        if (cmd == "start") {
            if (g_schedule_task_running) {
                std::cout << "[调度] ❌ 调度任务已在运行中" << std::endl;
                continue;
            }
            if (g_vision_started.load()) {
                std::cout << "[调度] ❌ 视觉抓取正在运行，请先执行 vision_stop" << std::endl;
                continue;
            }
            // 启动调度任务线程
            std::thread(scheduleTaskThread, 
                        std::ref(task_manager), 
                        map_id, 
                        std::cref(site_ids), 
                        std::cref(action_ids)).detach();
            std::cout << "[调度] ✅ 已启动调度任务" << std::endl;
        }
        else if (cmd == "p" || cmd == "pause") {
            if (!g_schedule_task_running) {
                std::cout << "[调度] ❌ 当前没有运行中的调度任务" << std::endl;
                continue;
            }
            int ret = task_manager.pauseTask();
            std::cout << (ret == 0 ? "[调度] ✅ 已暂停" : "[调度] ❌ 暂停失败") << std::endl;
        }
        else if (cmd == "r" || cmd == "resume") {
            if (!g_schedule_task_running) {
                std::cout << "[调度] ❌ 当前没有运行中的调度任务" << std::endl;
                continue;
            }
            int ret = task_manager.resumeTask();
            std::cout << (ret == 0 ? "[调度] ✅ 已恢复" : "[调度] ❌ 恢复失败") << std::endl;
        }
        else if (cmd == "s" || cmd == "stop") {
            if (!g_schedule_task_running) {
                std::cout << "[调度] 当前没有运行中的调度任务" << std::endl;
                continue;
            }
            task_manager.stop();
            std::cout << "[调度] ⏹️ 已停止调度任务，程序继续运行" << std::endl;
        }

        // ==================== 直控模式命令 ====================
        else if (cmd == "goto") {
            float x, y, angle;
            if (!(iss >> x >> y >> angle)) {
                std::cout << "[直控] ❌ 参数错误，格式: goto x y angle" << std::endl;
                continue;
            }
            if (g_direct_cruise_running) {
                std::cout << "[直控] ❌ 已有直控任务正在运行，请先停止" << std::endl;
                continue;
            }
            // ========== 新增：互斥检查 ==========
            if (g_arm_working.load()) {
                std::cout << "[直控] ❌ 机械臂正在作业，禁止底盘移动" << std::endl;
                continue;
            }

            std::cout << "[直控] 移动到坐标: (" << x << ", " << y << ", " << angle << "°)" << std::endl;
            // 后台线程执行，不阻塞输入
            std::thread([&chassis, x, y, angle]() {
                g_direct_cruise_running = true;
                int ret = chassis.goToPose(x, y, angle, true);
                if (ret == CHASSIS_OK) {
                    std::cout << "\n[直控] ✅ 到达目标点" << std::endl;
                } else {
                    std::cerr << "\n[直控] ❌ 移动失败，错误码: " << ret << std::endl;
                }
                g_direct_cruise_running = false;
            }).detach();
        }
        else if (cmd == "cruise")
        {
            if (g_direct_cruise_running)
            {
                std::cout << "[直控] ❌ 已有直控任务正在运行，请先停止" << std::endl;
                continue;
            }
            // ========== 新增：互斥检查 ==========
            if (g_arm_working.load()) {
                std::cout << "[直控] ❌ 机械臂正在作业，禁止底盘移动" << std::endl;
                continue;
            }

            // 1. 从调度系统查询当前地图所有站点的坐标
            std::vector<json> all_nodes;
            int ret = dispatch_client.getTaskNodeList(map_id, all_nodes);
            if (ret != 0)
            {
                std::cerr << "[直控] ❌ 获取站点坐标失败，无法生成航点" << std::endl;
                continue;
            }
            // 2. 匹配 task_list 中的站点ID，生成航点列表
            std::vector<Waypoint> waypoints;
            int miss_count = 0;
            for (int sid : site_ids)
            {
                bool found = false;
                for (const auto &node : all_nodes)
                {
                    if (node.value("id", -1) == sid)
                    {
                        Waypoint wp;
                        wp.x = node.value("x", 0.0f);
                        wp.y = node.value("y", 0.0f);
                        wp.angle = 0.0f;
                        wp.stay_ms = 1000;
                        waypoints.push_back(wp);
                        found = true;
                        break;
                    }
                }
                if (!found)
                    miss_count++;
            }
            if (waypoints.empty())
            {
                std::cerr << "[直控] ❌ 未匹配到任何有效站点坐标" << std::endl;
                continue;
            }
            if (miss_count > 0)
            {
                std::cout << "[直控] ⚠️  有 " << miss_count << " 个站点未找到坐标，已跳过" << std::endl;
            }
            std::cout << "[直控] 生成航点 " << waypoints.size() << " 个，开始巡航..." << std::endl;
            // 3. 启动后台巡航线程
            std::thread(directCruiseThread, std::ref(chassis), waypoints).detach();
        }
        else if (cmd == "pause_move") {
            if (!g_direct_cruise_running) {
                std::cout << "[直控] ❌ 当前没有运行中的直控任务" << std::endl;
                continue;
            }
            int ret = chassis.pauseCruise();
            std::cout << (ret == CHASSIS_OK ? "[直控] ✅ 已暂停巡航" : "[直控] ❌ 暂停失败") << std::endl;
        }
        else if (cmd == "resume_move") {
            if (!g_direct_cruise_running) {
                std::cout << "[直控] ❌ 当前没有运行中的直控任务" << std::endl;
                continue;
            }
            int ret = chassis.resumeCruise();
            std::cout << (ret == CHASSIS_OK ? "[直控] ✅ 已恢复巡航" : "[直控] ❌ 恢复失败") << std::endl;
        }
        else if (cmd == "stop_move") {
            // 无论是否在巡航，都强制停止底盘，安全兜底
            chassis.cancelCruise();
            chassis.stopMove();
            std::cout << "[直控] ⏹️ 已下发停止移动指令" << std::endl;
        }
        else if (cmd == "arm_cruise")
        {
            if (g_direct_cruise_running)
            {
                std::cout << "[直控] 巡航任务正在运行中，请先停止" << std::endl;
                continue;
            }
            // ========== 新增：互斥检查 ==========
            if (g_arm_working.load()) {
                std::cout << "[直控] ❌ 机械臂正在作业，禁止底盘移动" << std::endl;
                continue;
            }

            std::vector<Waypoint> waypoints;
            if (!chassis.loadWaypointsFromFile(app_config.waypoint_file, waypoints))
            {
                std::cerr << "[直控] 航点文件加载失败" << std::endl;
                continue;
            }
            std::cout << "[直控] 带机械臂动作的巡航启动，共" << waypoints.size() << "个航点" << std::endl;
            g_direct_cruise_running = true;
            std::thread([&chassis, waypoints]()
                        {
                for (size_t i = 0; i < waypoints.size() && g_direct_cruise_running; i++)
                {
                    const auto& wp = waypoints[i];
                    std::cout << "\n[直控] 前往第" << i+1 << "个航点 (" << wp.x << ", " << wp.y << ")" << std::endl;
                    
                    int ret = chassis.goToPose(wp.x, wp.y, wp.angle, true);
                    if (ret != 0)
                    {
                        std::cerr << "[直控] 导航失败，终止巡航" << std::endl;
                        break;
                    }
                    std::cout << "[直控] 到达第" << i+1 << "个航点" << std::endl;

                    // ========== 执行绑定的机械臂动作序列 + 互斥保护 ==========
                    if (!wp.action_points.empty() && g_arm_direct && g_arm_direct->isConnected())
                    {
                        // 上锁：标记机械臂作业中，禁止底盘移动
                        g_arm_working.store(true);
                        std::cout << "[互斥保护] 机械臂作业开始，底盘已锁定" << std::endl;
                        
                        g_arm_direct->executePointList(wp.action_points, 500);
                        
                        // 解锁：作业完成，允许底盘行驶
                        g_arm_working.store(false);
                        std::cout << "[互斥保护] 机械臂作业结束，底盘已解锁" << std::endl;
                    }

                    // 基础停留
                    if (wp.stay_ms > 0)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(wp.stay_ms));
                    }
                }
                std::cout << "\n[直控] 巡航任务执行完毕" << std::endl;
                g_direct_cruise_running = false; })
                .detach();
        }
        else if (cmd == "home") {
            if (g_direct_cruise_running) {
                std::cout << "[直控] ❌ 已有直控任务正在运行，请先停止" << std::endl;
                continue;
            }
            // ========== 新增：互斥检查 ==========
            if (g_arm_working.load()) {
                std::cout << "[直控] ❌ 机械臂正在作业，禁止底盘移动" << std::endl;
                continue;
            }

            std::cout << "[直控] 返回原点: (" << app_config.home_x 
                      << ", " << app_config.home_y 
                      << ", " << app_config.home_angle << "°)" << std::endl;
            std::thread([&chassis, &app_config]() {
                g_direct_cruise_running = true;
                int ret = chassis.goHome(app_config.home_x, app_config.home_y, app_config.home_angle, true);
                std::cout << (ret == CHASSIS_OK ? "\n[直控] ✅ 已回到原点" : "\n[直控] ❌ 回原点失败") << std::endl;
                g_direct_cruise_running = false;
            }).detach();
        }

        // ==================== 视觉抓取模式命令 ====================
        else if (cmd == "vision") {
            startVisionGrasp(app_config.vision);
        }
        else if (cmd == "vision_stop") {
            stopVisionGrasp();
        }

        // ==================== 通用命令 ====================
        else if (cmd == "h" || cmd == "help") {
            std::cout << "\n========== 命令帮助 ==========" << std::endl;
            std::cout << "【调度任务模式】" << std::endl;
            std::cout << "  start           启动调度巡检任务" << std::endl;
            std::cout << "  p / pause       暂停调度航行任务" << std::endl;
            std::cout << "  r / resume      恢复调度航行任务" << std::endl;
            std::cout << "  s / stop        停止调度任务（不退出程序）" << std::endl;
            std::cout << "【直控测试模式】" << std::endl;
            std::cout << "  goto x y angle  直控移动到指定坐标(米, 度)" << std::endl;
            std::cout << "  cruise          加载航点文件执行多航点巡航" << std::endl;
            std::cout << "  pause_move      暂停直控巡航" << std::endl;
            std::cout << "  resume_move     恢复直控巡航" << std::endl;
            std::cout << "  stop_move       停止直控移动/巡航" << std::endl;
            std::cout << "  home            直控返回配置的原点" << std::endl;
            std::cout << "  arm_cruise      带机械臂动作的直控坐标巡航" << std::endl;
            std::cout << "【视觉抓取模式】" << std::endl;
            std::cout << "  vision          启动视觉抓取（订阅 YOLO 检测结果）" << std::endl;
            std::cout << "  vision_stop     停止视觉抓取" << std::endl;
            std::cout << "【通用】" << std::endl;
            std::cout << "  h / help        显示帮助" << std::endl;
            std::cout << "  q / quit        停止所有运动并退出程序" << std::endl;
            std::cout << "==============================\n" << std::endl;
        }
        else if (cmd == "q" || cmd == "quit") {
            std::cout << "正在停止所有运动并退出程序..." << std::endl;
            g_should_quit = true;
            // 停止所有运动
            stopVisionGrasp();
            task_manager.stop();
            chassis.cancelCruise();
            chassis.stopMove();
            break;
        }
        else {
            std::cout << "❓ 未知命令: " << cmd << "，输入 h 查看帮助" << std::endl;
        }
    }
}

int main() {
    signal(SIGINT, signalHandler);

    // ==================== 1. 加载全局配置 ====================
    AppConfig app_config;
    loadAppConfig("/home/robot/robot_ws/config/config.json", app_config);

    // ==================== 2. 初始化模块 ====================
    DispatchClient dispatch_client(app_config.dispatch_url, app_config.robot_id);
    ChassisController chassis(app_config.chassis_url, dispatch_client);
    chassis.setSpeed(app_config.move_speed, app_config.rot_speed);
    chassis.setRetryCount(app_config.retry_count);
    ArmController arm(dispatch_client);

    // ========== 初始化TCP直连机械臂 + 全局预注册所有示教点位 ==========
    g_arm_direct = std::make_shared<ArmDirectController>();
    if (g_arm_direct->connect("192.168.30.11", 5868)) {
        std::cout << "✅ TCP直连机械臂连接成功" << std::endl;
        
        // 初始化阶段上锁保护
        g_arm_working.store(true);
        g_arm_direct->enable();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        g_arm_direct->goSafeHome();
        g_arm_working.store(false);

        std::cout << "✅ 机械臂已回安全基准位" << std::endl;

        // 全局注册所有示教点位（仅执行一次）
        g_arm_direct->addJointPoint("catch", 86.3, -133.9, -42.7, -94.7, 90.9, 8.3);
        g_arm_direct->addJointPoint("move",  86.3, -90.4,  -88.0, -94.7, 90.9, 8.3);
    } else {
        std::cerr << "⚠️  TCP直连机械臂连接失败，将跳过机械臂动作" << std::endl;
    }

    // ==================== 3. 加载调度任务配置 ====================
    std::vector<int> site_ids;
    std::vector<int> action_ids;
    int map_id = 63;
    try {
        std::ifstream f(app_config.task_file);
        if (!f.is_open()) throw std::runtime_error("无法打开任务文件: " + app_config.task_file);
        json cfg = json::parse(f);
        map_id = cfg.value("map_id", 63);
        if (cfg.contains("tasks") && cfg["tasks"].is_array()) {
            const auto& one_task = cfg["tasks"][0];
            if (one_task.contains("site_list") && one_task["site_list"].is_array()) {
                for (const auto& point : one_task["site_list"]) {
                    if (point.contains("site_id")) {
                        site_ids.push_back(point["site_id"].get<int>());
                        action_ids.push_back(point.value("action_id", 0));
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "加载任务配置失败: " << e.what() << "，使用默认任务" << std::endl;
        site_ids = {234, 226, 227, 228, 229, 231, 230, 191, 232, 234};
        action_ids = {0, 0, 101, 0, 0, 0, 0, 0, 0, 0};
    }

    if (site_ids.empty()) {
        std::cerr << "错误：无有效站点，程序退出" << std::endl;
        return -1;
    }

    std::cout << "\n========== 复合机器人系统启动 ==========" << std::endl;
    std::cout << "机器人ID: " << app_config.robot_id << std::endl;
    std::cout << "调度服务: " << app_config.dispatch_url << std::endl;
    std::cout << "直控服务: " << app_config.chassis_url << std::endl;
    std::cout << "地图ID: " << map_id << "  站点数: " << site_ids.size() << std::endl;
    std::cout << "输入 h 查看所有命令\n" << std::endl;

    // ==================== 4. 创建任务管理器 ====================
    TaskManager task_manager(dispatch_client, chassis, arm, g_arm_direct.get());
    
    // 配置站点 → 机械臂点位序列 映射
    std::map<int, std::vector<std::string>> site_action_map;
    site_action_map[226] = {"catch", "move"};
    site_action_map[231] = {"catch", "move"};
    task_manager.setSiteActionMap(site_action_map);

    // ==================== 5. 启动控制台输入线程 ====================
    std::thread input_thread([&]()
                             { consoleInputThread(task_manager, chassis, dispatch_client, app_config, map_id, site_ids, action_ids); });

    // ==================== 6. 主循环：常驻运行，直到收到退出指令 ====================
    while (!g_should_quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // ==================== 7. 退出清理 ====================
    stopVisionGrasp();
    task_manager.stop();
    chassis.cancelCruise();
    chassis.stopMove();

    // 机械臂回零
    if (g_arm_direct && g_arm_direct->isConnected()) {
        g_arm_working.store(true);
        g_arm_direct->goSafeHome();
        g_arm_working.store(false);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "\n所有运动已停止，按回车键退出程序..." << std::endl;
    g_input_thread_exit = true;
    input_thread.join();
    std::cout << "程序安全退出" << std::endl;
    return 0;
}
