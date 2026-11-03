// ============================================================
// 机械臂单独测试程序
// 功能：验证 TCP 直连机械臂控制器(192.168.30.11:5868)，
//       测试连接/使能/回零/关节点位/笛卡尔点位/原始指令下发。
// 配置：arm_config.json（同目录），缺省时使用默认值。
// 用法：本程序在 Linux(虚拟机) 上编译运行，见 README.md
// ============================================================
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <memory>
#include "nlohmann/json.hpp"
#include "arm/arm_direct_controller.h"

using json = nlohmann::json;

struct TestConfig {
    std::string arm_ip = "192.168.30.11";
    int arm_port = 5868;
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
        cfg.arm_ip   = j.value("arm_ip", cfg.arm_ip);
        cfg.arm_port = j.value("arm_port", cfg.arm_port);
        std::cout << "[配置] 加载成功: " << file_path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[配置] 解析失败: " << e.what() << std::endl;
        return false;
    }
}

void showHelp() {
    std::cout << "\n========== 机械臂测试命令 ==========" << std::endl;
    std::cout << "  connect [ip] [port]  连接机械臂控制器" << std::endl;
    std::cout << "  clear                清除故障" << std::endl;
    std::cout << "  enable               上电使能" << std::endl;
    std::cout << "  stop                 停止(急停)" << std::endl;
    std::cout << "  home                 回安全零位(6轴全0°)" << std::endl;
    std::cout << "  addj name j1 j2 j3 j4 j5 j6  添加关节点位(度,自动限位校验)" << std::endl;
    std::cout << "  move name [wait_ms]  运动到指定关节点位" << std::endl;
    std::cout << "  addc name x y z rx ry rz     添加笛卡尔点位(mm/度)" << std::endl;
    std::cout << "  movej name [wait_ms] 笛卡尔关节插补(MoveJ)" << std::endl;
    std::cout << "  movel name [wait_ms] 笛卡尔直线插补(MoveL)" << std::endl;
    std::cout << "  speed ratio          设置全局速度倍率 1~100%" << std::endl;
    std::cout << "  raw cmd...           发送原始指令(如 {SetHandState --state=0 --hand_id=0})" << std::endl;
    std::cout << "  h / help             帮助" << std::endl;
    std::cout << "  q / quit             退出" << std::endl;
    std::cout << "=======================================\n" << std::endl;
}

int main() {
    TestConfig cfg;
    loadConfig("arm_config.json", cfg);

    auto arm = std::make_shared<ArmDirectController>();

    std::cout << "\n========== 机械臂单独测试 ==========" << std::endl;
    std::cout << "默认控制器: " << cfg.arm_ip << ":" << cfg.arm_port << std::endl;
    showHelp();

    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if (cmd == "connect") {
            std::string ip = cfg.arm_ip;
            int port = cfg.arm_port;
            iss >> ip >> port;
            if (arm->isConnected()) {
                std::cout << "[机械臂] 已连接，如需重连请先退出程序" << std::endl;
                continue;
            }
            bool ok = arm->connect(ip, port);
            std::cout << (ok ? "[机械臂] ✅ 连接成功" : "[机械臂] ❌ 连接失败") << std::endl;
        }
        else if (cmd == "clear") {
            bool ok = arm->clearFault();
            std::cout << (ok ? "[机械臂] ✅ 清除故障成功" : "[机械臂] ❌ 清除故障失败(可能未连接)") << std::endl;
        }
        else if (cmd == "enable") {
            bool ok = arm->enable();
            std::cout << (ok ? "[机械臂] ✅ 使能成功" : "[机械臂] ❌ 使能失败") << std::endl;
        }
        else if (cmd == "stop") {
            bool ok = arm->stop();
            std::cout << (ok ? "[机械臂] ✅ 已下发停止" : "[机械臂] ❌ 停止指令失败") << std::endl;
        }
        else if (cmd == "home") {
            bool ok = arm->goSafeHome(8000);
            std::cout << (ok ? "[机械臂] ✅ 已回安全零位" : "[机械臂] ❌ 回零失败") << std::endl;
        }
        else if (cmd == "addj") {
            std::string name;
            double j1, j2, j3, j4, j5, j6;
            if (!(iss >> name >> j1 >> j2 >> j3 >> j4 >> j5 >> j6)) {
                std::cout << "[机械臂] ❌ 参数错误，格式: addj name j1 j2 j3 j4 j5 j6" << std::endl;
                continue;
            }
            bool ok = arm->addJointPoint(name, j1, j2, j3, j4, j5, j6);
            std::cout << (ok ? "[机械臂] ✅ 点位已添加: " + name : "[机械臂] ❌ 添加失败(超限?)") << std::endl;
        }
        else if (cmd == "move") {
            std::string name;
            int wait_ms = 5000;
            if (!(iss >> name)) {
                std::cout << "[机械臂] ❌ 参数错误，格式: move name [wait_ms]" << std::endl;
                continue;
            }
            iss >> wait_ms;
            bool ok = arm->moveToPoint(name, wait_ms);
            std::cout << (ok ? "[机械臂] ✅ 运动完成: " + name : "[机械臂] ❌ 运动失败: " + name) << std::endl;
        }
        else if (cmd == "addc") {
            std::string name;
            double x, y, z, rx, ry, rz;
            if (!(iss >> name >> x >> y >> z >> rx >> ry >> rz)) {
                std::cout << "[机械臂] ❌ 参数错误，格式: addc name x y z rx ry rz" << std::endl;
                continue;
            }
            bool ok = arm->addCartPoint(name, x, y, z, rx, ry, rz);
            std::cout << (ok ? "[机械臂] ✅ 笛卡尔点位已添加: " + name : "[机械臂] ❌ 添加失败(范围异常?)") << std::endl;
        }
        else if (cmd == "movej") {
            std::string name;
            int wait_ms = 5000;
            if (!(iss >> name)) {
                std::cout << "[机械臂] ❌ 参数错误，格式: movej name [wait_ms]" << std::endl;
                continue;
            }
            iss >> wait_ms;
            bool ok = arm->moveCartJ(name, wait_ms);
            std::cout << (ok ? "[机械臂] ✅ MoveJ完成: " + name : "[机械臂] ❌ MoveJ失败: " + name) << std::endl;
        }
        else if (cmd == "movel") {
            std::string name;
            int wait_ms = 5000;
            if (!(iss >> name)) {
                std::cout << "[机械臂] ❌ 参数错误，格式: movel name [wait_ms]" << std::endl;
                continue;
            }
            iss >> wait_ms;
            bool ok = arm->moveCartL(name, wait_ms);
            std::cout << (ok ? "[机械臂] ✅ MoveL完成: " + name : "[机械臂] ❌ MoveL失败: " + name) << std::endl;
        }
        else if (cmd == "speed") {
            int ratio;
            if (!(iss >> ratio) || ratio < 1 || ratio > 100) {
                std::cout << "[机械臂] ❌ 参数错误，格式: speed 1~100" << std::endl;
                continue;
            }
            arm->setSpeedRatio(ratio);
            std::cout << "[机械臂] ✅ 速度倍率已设置: " << ratio << "%" << std::endl;
        }
        else if (cmd == "raw") {
            std::string raw;
            std::getline(iss, raw);
            if (raw.empty()) {
                std::cout << "[机械臂] ❌ 格式: raw {指令}" << std::endl;
                continue;
            }
            // 去掉前导空格
            raw.erase(0, raw.find_first_not_of(" \t"));
            bool ok = arm->sendRawCmd(raw);
            std::cout << (ok ? "[机械臂] ✅ 原始指令执行成功" : "[机械臂] ❌ 原始指令执行失败") << std::endl;
        }
        else if (cmd == "h" || cmd == "help") {
            showHelp();
        }
        else if (cmd == "q" || cmd == "quit") {
            std::cout << "正在断开机械臂并退出..." << std::endl;
            if (arm->isConnected()) {
                arm->goSafeHome(8000);
            }
            arm->disconnect();
            break;
        }
        else {
            std::cout << "❓ 未知命令: " << cmd << "，输入 h 查看帮助" << std::endl;
        }
    }
    std::cout << "机械臂测试程序已退出" << std::endl;
    return 0;
}
