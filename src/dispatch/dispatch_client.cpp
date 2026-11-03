#include "dispatch/dispatch_client.h"
#include "common/http_utils.h"
#include <iostream>

DispatchClient::DispatchClient(const std::string& dispatch_url, int robot_id)
    : base_url_(dispatch_url)
    , robot_id_(robot_id)
    , connect_timeout_s_(5)
    , total_timeout_s_(10)
{
}

std::string DispatchClient::makeUrl(const std::string& path) const {
    return base_url_ + path;
}

int DispatchClient::getRobotId() const {
    return robot_id_;
}

// 1. 获取地图列表
int DispatchClient::getMapInfoList(int count, std::vector<json>& out_map_list) {
    std::string path = "/dispatch/get_map_info_list?count=" + std::to_string(count);
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    if (resp.value("code", -1) != 0) return -1;
    out_map_list.clear();
    if (resp.contains("data") && resp["data"].contains("infoList")) {
        out_map_list = resp["data"]["infoList"].get<std::vector<json>>();
    }
    return 0;
}

// 2. 获取作业点列表
int DispatchClient::getTaskNodeList(int map_id, std::vector<json>& out_node_list) {
    std::string path = "/dispatch/get_task_node_list?mapId=" + std::to_string(map_id) + "&count=-1";
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    if (resp.value("code", -1) != 0) return -1;
    out_node_list.clear();
    if (resp.contains("data") && resp["data"].contains("taskNodeInfoList")) {
        out_node_list = resp["data"]["taskNodeInfoList"].get<std::vector<json>>();
    }
    return 0;
}

// 12. 获取站点动作列表
int DispatchClient::getTaskNodeActionList(int node_id, std::vector<json>& out_action_list) {
    std::string path = "/dispatch/get_task_node_action_list?nodeId=" + std::to_string(node_id);
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    if (resp.value("code", -1) != 0) return -1;
    out_action_list.clear();
    if (resp.contains("data") && resp["data"].contains("infos")) {
        out_action_list = resp["data"]["infos"].get<std::vector<json>>();
    }
    return 0;
}

// 3. 提交任务组
int DispatchClient::addTaskGroup(int map_id, const std::vector<int>& task_node_ids, const std::vector<int>& action_group_ids, std::string& out_task_group_id) {
    json body;
    body["mapId"] = map_id;
    body["robotId"] = robot_id_;
    
    json task_list = json::array();
    for (size_t i = 0; i < task_node_ids.size(); ++i) {
        json task;
        task["name"] = "task_" + std::to_string(i);
        task["taskNodeId"] = task_node_ids[i];
        if (i < action_group_ids.size() && action_group_ids[i] > 0) {
            task["taskActionGroupId"] = action_group_ids[i];
        }
        task_list.push_back(task);
    }
    body["taskList"] = task_list;

    json resp = HttpUtils::httpPost(makeUrl("/dispatch/add_task_group"), body, connect_timeout_s_, total_timeout_s_);
    if (resp.value("code", -1) != 0) {
        std::cerr << "[addTaskGroup] 失败: " << resp.value("msg", "unknown") << std::endl;
        return -1;
    }
    out_task_group_id = resp.value("data", json::object()).value("id", "");
    return 0;
}

// 4. 获取任务组列表
int DispatchClient::getTaskGroupList(int count, std::vector<json>& out_group_list) {
    std::string path = "/dispatch/get_task_group_list?count=" + std::to_string(count);
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    if (resp.value("code", -1) != 0) return -1;
    out_group_list.clear();
    if (resp.contains("data") && resp["data"].contains("taskGroupList")) {
        out_group_list = resp["data"]["taskGroupList"].get<std::vector<json>>();
    }
    return 0;
}

// 5. 删除任务组
int DispatchClient::deleteTaskGroup(const std::string& task_group_id) {
    std::string path = "/dispatch/delete_task_group?taskGroupId=" + task_group_id;
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    return resp.value("code", -1) == 0 ? 0 : -1;
}

// 15. 取消单个任务
int DispatchClient::cancelSingleTask(const std::string& task_group_id, const std::string& task_id) {
    std::string path = "/dispatch/cancel_task?taskGroupId=" + task_group_id + "&taskId=" + task_id;
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    return resp.value("code", -1) == 0 ? 0 : -1;
}

// 6. 停止机器人任务
int DispatchClient::stopRobotTask() {
    std::string path = "/dispatch/stop_robot_task?robotId=" + std::to_string(robot_id_);
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    return resp.value("code", -1) == 0 ? 0 : -1;
}

// 7. 暂停机器人任务
int DispatchClient::pauseRobotTask() {
    std::string path = "/dispatch/pause_robot_task?robotId=" + std::to_string(robot_id_);
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    return resp.value("code", -1) == 0 ? 0 : -1;
}

// 8. 恢复机器人任务
int DispatchClient::resumeRobotTask() {
    std::string path = "/dispatch/resume_robot_task?robotId=" + std::to_string(robot_id_);
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    return resp.value("code", -1) == 0 ? 0 : -1;
}

// 9. 重新规划路径
int DispatchClient::refreshRobotRoute() {
    std::string path = "/dispatch/refresh_robot_route?robotId=" + std::to_string(robot_id_);
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    return resp.value("code", -1) == 0 ? 0 : -1;
}

// 10. 获取所有机器人状态
int DispatchClient::getAllRobotStatus(std::vector<json>& out_robot_list) {
    json resp = HttpUtils::httpGet(makeUrl("/dispatch/get_all_robot_status"), connect_timeout_s_, total_timeout_s_);
    if (resp.value("code", -1) != 0) return -1;
    out_robot_list.clear();
    if (resp.contains("data") && resp["data"].contains("robotStatusList")) {
        out_robot_list = resp["data"]["robotStatusList"].get<std::vector<json>>();
    }
    return 0;
}

// 11. 获取机械臂状态
int DispatchClient::getTargetRobotArmStatus(json& out_arm_status) {
    std::string path = "/dispatch/get_target_robot_arm_status?robotId=" + std::to_string(robot_id_);
    json resp = HttpUtils::httpGet(makeUrl(path), connect_timeout_s_, total_timeout_s_);
    if (resp.value("code", -1) != 0) return -1;
    out_arm_status = resp.value("data", json::object());
    return 0;
}

// 14. 执行动作（按ID）
int DispatchClient::startAction(int action_id) {
    json body;
    body["robotId"] = robot_id_;
    body["actionId"] = action_id;
    json resp = HttpUtils::httpPost(makeUrl("/dispatch/start_action"), body, connect_timeout_s_, total_timeout_s_);
    return resp.value("code", -1) == 0 ? 0 : -1;
}

// 14. 执行动作（按类型+参数）
int DispatchClient::startActionByType(int type, const std::vector<std::string>& param_list) {
    json body;
    body["robotId"] = robot_id_;
    body["type"] = type;
    body["paramList"] = param_list;
    json resp = HttpUtils::httpPost(makeUrl("/dispatch/start_action"), body, connect_timeout_s_, total_timeout_s_);
    return resp.value("code", -1) == 0 ? 0 : -1;
}

// 兼容接口：安全获取机器人位姿
bool DispatchClient::getRobotPoseSafe(float& x, float& y, float& angle, bool& is_fault) {
    std::vector<json> robot_list;
    // 调用调度接口获取全量机器人状态
    if (getAllRobotStatus(robot_list) != 0) {
        is_fault = true;
        return false;
    }

    // 遍历查找目标机器人
    for (const auto& robot : robot_list) {
        if (robot.value("robotId", -1) == robot_id_) {
            // 读取位姿信息，字段与调度接口RobotStatus定义完全对齐
            x = robot.value("x", 0.0f);
            y = robot.value("y", 0.0f);
            angle = robot.value("angle", 0.0f);
            
            // ========== 核心修复 ==========
            // 原代码读取不存在的"fault"字段，替换为接口定义的"isForceStop"（硬件急停标志）
            // 急停触发时视为故障状态，上层任务逻辑可据此停止任务、触发告警
            is_fault = robot.value("isForceStop", false);
            return true;
        }
    }

    // 未在调度系统中找到目标机器人，视为故障状态
    is_fault = true;
    return false;
}

// 兼容接口：获取机械臂状态
int DispatchClient::getRobotArmStatus(bool& is_finished, bool& is_error, std::string& error_reason) {
    json status;
    // 调用调度接口获取机械臂状态
    if (getTargetRobotArmStatus(status) != 0) {
        return -1;
    }

    // ========== 核心修复：字段名与接口文档对齐 ==========
    // 原代码读取错误字段：finished / error / msg
    // 接口文档实际字段：isFinished / isError / errorReason
    is_finished = status.value("isFinished", false);
    is_error = status.value("isError", false);
    error_reason = status.value("errorReason", "");
    // ======================================================

    return 0;
}