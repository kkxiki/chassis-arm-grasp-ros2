#include "arm/arm_controller.h"
#include <iostream>
#include <thread>
#include <chrono>

// 构造函数
ArmController::ArmController(DispatchClient& dispatch_client)
    : dispatch_client_(dispatch_client)
{
}

// 启动指定ID的动作（非阻塞）
int ArmController::startAction(int action_id) {
    return dispatch_client_.startAction(action_id);
}

// 按类型启动动作（带参数，非阻塞）
int ArmController::startActionByType(int type, const std::vector<std::string>& param_list) {
    return dispatch_client_.startActionByType(type, param_list);
}

// 查询机械臂运行状态
int ArmController::getStatus(bool& is_finished, bool& is_error, std::string& error_reason) {
    return dispatch_client_.getRobotArmStatus(is_finished, is_error, error_reason);
}

// 阻塞执行机械臂动作
int ArmController::executeActionBlocking(int action_id, volatile bool* stop_flag, int check_interval_ms) {
    if (action_id <= 0) {
        return 0;
    }
    std::cout << "\n  🦾 开始执行机械臂动作，动作ID: " << action_id << std::endl;
    
    int ret = startAction(action_id);
    if (ret != 0) {
        std::cerr << "  ❌ 机械臂动作下发失败" << std::endl;
        return -1;
    }

    while (true) {
        if (stop_flag != nullptr && *stop_flag) {
            return -2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
        
        bool is_finished = false;
        bool is_error = false;
        std::string error_reason;
        int status_ret = getStatus(is_finished, is_error, error_reason);
        if (status_ret != 0) {
            continue;
        }
        if (is_error) {
            std::cerr << "  ❌ 机械臂动作异常: " << error_reason << std::endl;
            return -1;
        }
        if (is_finished) {
            std::cout << "  ✅ 机械臂动作执行完成" << std::endl;
            return 0;
        }
    }
}

// 查询动作是否完成
bool ArmController::isFinished() const {
    bool finished = false;
    bool is_error = false;
    std::string reason;
    
    int ret = dispatch_client_.getRobotArmStatus(finished, is_error, reason);
    if (ret != 0) {
        return true;
    }
    return finished || is_error;
}