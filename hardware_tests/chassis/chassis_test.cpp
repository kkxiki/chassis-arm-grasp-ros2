// ============================================================
// 底盘单独测试程序
// 功能：验证调度服务(8008)与直控服务(8767)连通性，
//       测试位姿查询、直控移动/旋转、巡航回位等底盘动作。
// 配置：chassis_config.json（同目录），缺省时使用默认值。
// 用法：本程序在 Linux(虚拟机) 上编译运行，见 README.md
// ============================================================
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <sstream>
#include <fstream>
#include <algorithm>
#include "nlohmann/json.hpp"
#include "common/http_utils.h"
#include "dispatch/dispatch_client.h"
#include "chassis/chassis_controller.h"

using json = nlohmann::json;

// ==================== 全局状态 ====================
std::atomic<bool> g_quit{false};
std::atomic<bool> g_cruise_running{false};

// ==================== 测试配置 ====================
struct TestConfig {
    std::string chassis_url  = "http://192.168.30.200:8767";
    std::string dispatch_url = "http://192.168.30.200:8008";
    int robot_id = 227;
    int map_id = 63;
    float move_speed = 0.3f;
    float rot_speed  = 30.0f;
    float home_x = -3.1355f;
    float home_y = -2.1429f;
    float home_angle = -90.2697f;
};

bool loadConfig(const std::string& file_path, TestConfig& cfg) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "[配置] 无法打开 " << file_path << "，使用默认参数" << std::endl;
        return false;
    }
    try {
        json j;
        file >> j;
        cfg.chassis_url  = j.value("chassis_url", cfg.chassis_url);
        cfg.dispatch_url = j.value("dispatch_url", cfg.dispatch_url);
        cfg.robot_id     = j.value("robot_id", cfg.robot_id);
        cfg.map_id       = j.value("map_id", cfg.map_id);
        cfg.move_speed   = j.value("move_speed", cfg.move_speed);
        cfg.rot_speed    = j.value("rot_speed", cfg.rot_speed);
        if (j.contains("home_pose") && j["home_pose"].is_object()) {
            const auto& h = j["home_pose"];
            cfg.home_x     = h.value("x", cfg.home_x);
            cfg.home_y     = h.value("y", cfg.home_y);
            cfg.home_angle = h.value("angle", cfg.home_angle);
        }
        std::cout << "[配置] 加载成功: " << file_path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[配置] 解析失败: " << e.what() << std::endl;
        return false;
    }
}

// ==================== 打印调度返回的机器人状态 ====================
// 字段与调度接口 RobotStatus 定义对齐：robotId/x/y/angle/taskStatus/battery/isCharging/isForceStop
void printRobotStatus(const std::vector<json>& list) {
    if (list.empty()) {
        std::cout << "  (无机器人状态返回)" << std::endl;
        return;
    }
    for (const auto& r : list) {
        std::cout << "  robotId=" << r.value("robotId", -1)
                  << " 坐标=(" << r.value("x", 0.0f) << ", " << r.value("y", 0.0f)
                  << ", " << r.value("angle", 0.0f) << "°)"
                  << " 任务状态=" << r.value("taskStatus", -1)
                  << " 电量=" << r.value("battery", -1) << "%"
                  << " 充电中=" << r.value("isCharging", false)
                  << " 急停=" << r.value("isForceStop", false)
                  << " 地图=" << r.value("mapName", "")
                  << std::endl;
    }
}

// ==================== 打印地图节点 ====================
void printNodes(const std::vector<json>& nodes) {
    if (nodes.empty()) {
        std::cout << "  (无作业点)" << std::endl;
        return;
    }
    for (const auto& n : nodes) {
        std::cout << "  id=" << n.value("id", -1)
                  << " 名称=" << n.value("name", "")
                  << " 坐标=(" << n.value("x", 0.0f) << ", " << n.value("y", 0.0f) << ")"
                  << std::endl;
    }
}

void showHelp() {
    std::cout << "\n========== 底盘测试命令 ==========" << std::endl;
    std::cout << "  status          查询所有机器人状态(调度)" << std::endl;
    std::cout << "  pose            查询本机当前位姿" << std::endl;
    std::cout << "  nodes [map_id]  查询地图作业点列表" << std::endl;
    std::cout << "  move x y angle [speed]  直控移动到目标点(米/度/可选m/s)" << std::endl;
    std::cout << "  stop            停止直控移动" << std::endl;
    std::cout << "  rot angle [speed]       直控原地旋转(度/可选度每秒)" << std::endl;
    std::cout << "  stoprot         停止直控旋转" << std::endl;
    std::cout << "  goto x y angle  直控巡航到目标点(阻塞轮询到位)" << std::endl;
    std::cout << "  home            直控返回配置的原点" << std::endl;
    std::cout << "  speed mv rs     设置速度(直线m/s 旋转度/s)" << std::endl;
    std::cout << "  h / help        帮助" << std::endl;
    std::cout << "  q / quit        退出" << std::endl;
    std::cout << "====================================\n" << std::endl;
}

int main() {
    HttpUtils::init();
    TestConfig cfg;
    loadConfig("chassis_config.json", cfg);

    // ==================== 初始化 ====================
    DispatchClient dispatch(cfg.dispatch_url, cfg.robot_id);
    ChassisController chassis(cfg.chassis_url, dispatch);
    chassis.setSpeed(cfg.move_speed, cfg.rot_speed);

    std::cout << "\n========== 底盘单独测试 ==========" << std::endl;
    std::cout << "调度服务: " << cfg.dispatch_url << std::endl;
    std::cout << "直控服务: " << cfg.chassis_url << std::endl;
    std::cout << "机器人ID: " << cfg.robot_id << "  地图ID: " << cfg.map_id << std::endl;
    showHelp();

    std::string line;
    while (!g_quit && std::getline(std::cin, line)) {
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if (cmd == "status") {
            std::vector<json> list;
            int ret = dispatch.getAllRobotStatus(list);
            std::cout << (ret == 0 ? "[底盘] ✅ 获取机器人状态成功:" : "[底盘] ❌ 获取失败") << std::endl;
            printRobotStatus(list);
        }
        else if (cmd == "pose") {
            float x, y, angle;
            int ret = chassis.getCurrentPose(x, y, angle);
            if (ret == CHASSIS_OK) {
                std::cout << "[底盘] ✅ 当前位置: x=" << x << " y=" << y << " angle=" << angle << "°" << std::endl;
            } else {
                std::cout << "[底盘] ❌ 获取位姿失败，错误码=" << ret << std::endl;
            }
        }
        else if (cmd == "nodes") {
            int map_id = cfg.map_id;
            iss >> map_id;
            std::vector<json> nodes;
            int ret = dispatch.getTaskNodeList(map_id, nodes);
            std::cout << (ret == 0 ? "[底盘] ✅ 地图" + std::to_string(map_id) + "作业点:" : "[底盘] ❌ 查询失败") << std::endl;
            printNodes(nodes);
        }
        else if (cmd == "move") {
            float x, y, angle, speed = cfg.move_speed;
            if (!(iss >> x >> y >> angle)) {
                std::cout << "[底盘] ❌ 参数错误，格式: move x y angle [speed]" << std::endl;
                continue;
            }
            iss >> speed;
            int ret = chassis.startMoveTo(x, y, angle, speed);
            std::cout << (ret == CHASSIS_OK ? "[底盘] ✅ 移动指令已下发" : "[底盘] ❌ 移动指令下发失败") << std::endl;
        }
        else if (cmd == "stop") {
            int ret = chassis.stopMove();
            std::cout << (ret == CHASSIS_OK ? "[底盘] ✅ 已停止移动" : "[底盘] ❌ 停止失败") << std::endl;
        }
        else if (cmd == "rot") {
            float angle, speed = cfg.rot_speed;
            if (!(iss >> angle)) {
                std::cout << "[底盘] ❌ 参数错误，格式: rot angle [speed]" << std::endl;
                continue;
            }
            iss >> speed;
            int ret = chassis.startRotate(angle, false, speed);
            std::cout << (ret == CHASSIS_OK ? "[底盘] ✅ 旋转指令已下发" : "[底盘] ❌ 旋转指令下发失败") << std::endl;
        }
        else if (cmd == "stoprot") {
            int ret = chassis.stopRotate();
            std::cout << (ret == CHASSIS_OK ? "[底盘] ✅ 已停止旋转" : "[底盘] ❌ 停止旋转失败") << std::endl;
        }
        else if (cmd == "goto") {
            float x, y, angle;
            if (!(iss >> x >> y >> angle)) {
                std::cout << "[底盘] ❌ 参数错误，格式: goto x y angle" << std::endl;
                continue;
            }
            if (g_cruise_running.load()) {
                std::cout << "[底盘] ❌ 已有巡航任务运行中" << std::endl;
                continue;
            }
            g_cruise_running.store(true);
            std::thread([&chassis, x, y, angle]() {
                int ret = chassis.goToPose(x, y, angle, true);
                if (ret == CHASSIS_OK)      std::cout << "\n[底盘] ✅ 到达目标点" << std::endl;
                else if (ret == CHASSIS_ERR_CANCELED) std::cout << "\n[底盘] ⏹️ 巡航被取消" << std::endl;
                else                        std::cout << "\n[底盘] ❌ 巡航失败，错误码=" << ret << std::endl;
                g_cruise_running.store(false);
            }).detach();
        }
        else if (cmd == "home") {
            if (g_cruise_running.load()) {
                std::cout << "[底盘] ❌ 已有巡航任务运行中" << std::endl;
                continue;
            }
            g_cruise_running.store(true);
            std::thread([&chassis, &cfg]() {
                int ret = chassis.goHome(cfg.home_x, cfg.home_y, cfg.home_angle, true);
                std::cout << (ret == CHASSIS_OK ? "\n[底盘] ✅ 已回到原点" : "\n[底盘] ❌ 回原点失败，错误码=" + std::to_string(ret)) << std::endl;
                g_cruise_running.store(false);
            }).detach();
        }
        else if (cmd == "speed") {
            float mv, rs;
            if (!(iss >> mv >> rs)) {
                std::cout << "[底盘] ❌ 参数错误，格式: speed mv rs" << std::endl;
                continue;
            }
            chassis.setSpeed(mv, rs);
            std::cout << "[底盘] ✅ 速度已设置: 直线=" << mv << "m/s 旋转=" << rs << "°/s" << std::endl;
        }
        else if (cmd == "h" || cmd == "help") {
            showHelp();
        }
        else if (cmd == "q" || cmd == "quit") {
            std::cout << "正在停止底盘并退出..." << std::endl;
            chassis.cancelCruise();
            chassis.stopMove();
            chassis.stopRotate();
            g_quit = true;
            break;
        }
        else {
            std::cout << "❓ 未知命令: " << cmd << "，输入 h 查看帮助" << std::endl;
        }
    }

    // ==================== 退出清理 ====================
    chassis.cancelCruise();
    chassis.stopMove();
    chassis.stopRotate();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    HttpUtils::cleanup();
    std::cout << "底盘测试程序已退出" << std::endl;
    return 0;
}
