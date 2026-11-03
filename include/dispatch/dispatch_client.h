#ifndef DISPATCH_CLIENT_H
#define DISPATCH_CLIENT_H

#include <string>
#include <vector>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class DispatchClient {
public:
    /**
     * @brief 构造调度客户端
     * @param dispatch_url 调度服务完整地址
     * @param robot_id 目标机器人ID
     */
    explicit DispatchClient(const std::string& dispatch_url, int robot_id);

    // ==================== 地图与站点接口 ====================
    int getMapInfoList(int count, std::vector<json>& out_map_list);
    int getTaskNodeList(int map_id, std::vector<json>& out_node_list);
    int getTaskNodeActionList(int node_id, std::vector<json>& out_action_list);

    // ==================== 任务管理接口 ====================
    int addTaskGroup(int map_id, const std::vector<int>& task_node_ids, const std::vector<int>& action_group_ids, std::string& out_task_group_id);
    int getTaskGroupList(int count, std::vector<json>& out_group_list);
    int deleteTaskGroup(const std::string& task_group_id);
    int cancelSingleTask(const std::string& task_group_id, const std::string& task_id);

    // ==================== 机器人控制接口 ====================
    int stopRobotTask();
    int pauseRobotTask();
    int resumeRobotTask();
    int refreshRobotRoute();

    // ==================== 状态查询接口 ====================
    int getAllRobotStatus(std::vector<json>& out_robot_list);
    int getTargetRobotArmStatus(json& out_arm_status);

        /**
     * @brief 安全获取目标机器人位姿（兼容老代码接口）
     * @param x 输出x坐标
     * @param y 输出y坐标
     * @param angle 输出朝向角
     * @param is_fault 输出是否故障
     * @return true=获取成功
     */
    bool getRobotPoseSafe(float& x, float& y, float& angle, bool& is_fault);

    /**
     * @brief 获取机械臂执行状态（兼容老代码接口）
     * @param is_finished 输出是否完成
     * @param is_error 输出是否出错
     * @param error_reason 输出错误原因
     * @return 0=成功，-1=失败
     */
    int getRobotArmStatus(bool& is_finished, bool& is_error, std::string& error_reason);

    // ==================== 动作执行接口 ====================
    int startAction(int action_id);
    int startActionByType(int type, const std::vector<std::string>& param_list);

    /**
     * @brief 获取当前绑定的机器人ID
     * @return 机器人ID
     */
    int getRobotId() const;

private:
    std::string base_url_;
    int robot_id_;
    int connect_timeout_s_;
    int total_timeout_s_;

    std::string makeUrl(const std::string& path) const;
};

#endif // DISPATCH_CLIENT_H