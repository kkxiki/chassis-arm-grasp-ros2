#include "task/task_manager.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstdio>
#include "nlohmann/json.hpp"
using json = nlohmann::json;
extern std::atomic<bool> g_arm_working;


TaskManager::TaskManager(DispatchClient& dispatch, ChassisController& chassis, ArmController& arm,
                         ArmDirectController* arm_direct)
    : dispatch_client_(dispatch)
    , chassis_(chassis)
    , arm_(arm)
    , arm_direct_(arm_direct) // 新增
    , is_finished_(false)
    , is_stopped_(false)
    , is_paused_(false)
    , current_idx_(0)
    , has_left_site_(false)
    , site_initialized_(false)
    , stable_count_(0)
    , last_angle_(0.0f)
    , pause_sent_(false)
    , wait_timeout_(0)
    , arrival_dist_threshold_(0.5f)
    , leave_factor_(1.8f)
    , stable_wait_ms_(1500)
{
}

// 新增：配置站点动作映射
void TaskManager::setSiteActionMap(const std::map<int, std::vector<std::string>>& site_action_map)
{
    site_action_map_ = site_action_map;
}

void TaskManager::runTask(int map_id, const std::vector<int>& site_ids, const std::vector<int>& action_ids) {
    is_finished_ = false;
    is_stopped_ = false;
    current_idx_ = 0;

     // 1. 站点列表为空直接返回，无效任务
    if (site_ids.empty()) {
        std::cerr << "[错误] 站点ID列表为空，无法执行任务" << std::endl;
        is_finished_ = true;
        return;
    }

    // 2. 先保存动作列表，长度不足时自动补齐默认值0（无动作）
    action_ids_ = action_ids;
    if (action_ids_.size() != site_ids.size()) {
        std::cerr << "[警告] 动作ID列表长度(" << action_ids_.size() 
                  << ")与站点ID列表长度(" << site_ids.size() 
                  << ")不一致，不足部分自动补0（无动作）" << std::endl;
        // 调整到和站点列表相同长度，新增元素默认填充0（不执行动作）
        action_ids_.resize(site_ids.size(), 0);
    }

    // 1. 获取站点坐标
    std::vector<json> all_nodes;
    int ret = dispatch_client_.getTaskNodeList(map_id, all_nodes);
    if (ret != 0) {
        std::cerr << "[错误] 获取站点列表失败" << std::endl;
        is_finished_ = true;
        return;
    }
    site_coords_.clear();
    for (int sid : site_ids) {
        bool found = false;
        for (const auto& node : all_nodes) {
            if (node.value("id", -1) == sid) {
                site_coords_.emplace_back(node.value("x", 0.0f), node.value("y", 0.0f));
                found = true;
                break;
            }
        }
        if (!found) {
            std::cerr << "[警告] 未找到站点ID: " << sid << std::endl;
            site_coords_.emplace_back(0.0f, 0.0f);
        }
    }

    // 2. 提交任务组
    std::string task_group_id;
    std::vector<int> empty_groups(site_ids.size(), 0);
    ret = dispatch_client_.addTaskGroup(map_id, site_ids, empty_groups, task_group_id);
    if (ret != 0) {
        std::cerr << "[错误] 提交任务失败" << std::endl;
        is_finished_ = true;
        return;
    }

    // 3. 主循环
    while (!is_stopped_ && static_cast<size_t>(current_idx_) < site_coords_.size()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // ========== 新增：全局暂停等待逻辑 ==========
        // 如果被外部暂停，就循环等待，不推进任务进度
        while (is_paused_ && !is_stopped_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            // 暂停状态下不执行后续的到站判定和动作逻辑
            continue;
        }
        // 如果暂停期间触发了停止，直接退出循环
        if (is_stopped_)
        {
            break;
        }
        
        std::vector<json> robot_list;
        if (dispatch_client_.getAllRobotStatus(robot_list) != 0) continue;

        float curr_x = 0, curr_y = 0, curr_angle = 0;
        bool found = false;
        for (const auto& r : robot_list) {
            if (r.value("robotId", -1) == dispatch_client_.getRobotId()) {
                curr_x = r.value("x", 0.0f);
                curr_y = r.value("y", 0.0f);
                curr_angle = r.value("angle", 0.0f);
                found = true;
                break;
            }
        }
        if (!found) continue;

        printf("\r  进度: 第%d/%zu个站点  状态: 运行中", current_idx_ + 1, site_coords_.size());
        fflush(stdout);

        float tx = site_coords_[current_idx_].first;
        float ty = site_coords_[current_idx_].second;
        int act = action_ids_[current_idx_];
        int current_site_id = site_ids[current_idx_];
        float dist = std::sqrt((curr_x - tx)*(curr_x - tx) + (curr_y - ty)*(curr_y - ty));

        // 初始化站点状态
        if (!site_initialized_) {
            has_left_site_ = (dist > arrival_dist_threshold_);
            site_initialized_ = true;
            stable_count_ = 0;
            last_angle_ = curr_angle;
            pause_sent_ = false;
            wait_timeout_ = 0;
        }

        // 离站判定
        if (!has_left_site_) {
            if (dist > arrival_dist_threshold_ * leave_factor_) {
                has_left_site_ = true;
                stable_count_ = 0;
                pause_sent_ = false;
                wait_timeout_ = 0;
            }
            continue;
        }

        // ========== 到站判定（核心修复：自定义动作优先级最高，不受原action_id限制） ==========
        if (dist < arrival_dist_threshold_) {

            // 先判断：当前站点是否绑定了自定义直连机械臂动作
            auto it = site_action_map_.find(current_site_id);
            bool has_custom_action = (it != site_action_map_.end() && arm_direct_ != nullptr && arm_direct_->isConnected());

            // ========== 情况1：有自定义机械臂动作 → 暂停-等稳-执行-恢复 ==========
            if (has_custom_action)
            {
                if (!pause_sent_)
                {
                    // 到站后先暂停调度任务，让机器人停稳
                    dispatch_client_.pauseRobotTask();
                    pause_sent_ = true;
                    stable_count_ = 0;
                    wait_timeout_ = 0;
                }

                // 判断角度是否稳定
                float delta = std::abs(curr_angle - last_angle_);
                if (delta > 180)
                    delta = 360 - delta;
                if (delta < 1.0f)
                {
                    stable_count_++;
                }
                else
                {
                    stable_count_ = 0;
                    dispatch_client_.pauseRobotTask();
                }
                last_angle_ = curr_angle;
                wait_timeout_++;

                // 满足稳定条件或等待超时，开始执行动作
                if (stable_count_ >= 2 || wait_timeout_ >= 10)
                {
                    const auto &point_list = it->second;
                    std::cout << "\n到达站点 " << current_site_id << "，准备执行自定义机械臂动作" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(stable_wait_ms_));

                    std::cout << "  站点" << current_site_id << "绑定自定义点位序列，共" << point_list.size() << "个点位" << std::endl;

                    // ========== 新增：上锁，机械臂作业开始，禁止底盘移动 ==========
                    g_arm_working.store(true);
                    std::cout << "[互斥保护] 机械臂作业开始，底盘已锁定" << std::endl;

                    // 阻塞执行：所有点位跑完 + 自动回零
                    arm_direct_->executePointList(point_list, 500);
                    std::cout << "  站点" << current_site_id << "自定义动作执行完毕" << std::endl;

                    // ========== 新增：解锁，作业完成，允许底盘行驶 ==========
                    g_arm_working.store(false);
                    std::cout << "[互斥保护] 机械臂作业结束，底盘已解锁" << std::endl;

                    // 无论动作成功失败，都恢复调度任务
                    dispatch_client_.resumeRobotTask();
                    has_left_site_ = false;
                    site_initialized_ = false;
                    stable_count_ = 0;
                    pause_sent_ = false;
                    wait_timeout_ = 0;
                    current_idx_++;
                }
            }
            // ========== 情况2：无自定义动作 + 原调度有动作ID → 走原调度系统动作 ==========
            else if (act > 0)
            {
                if (!pause_sent_)
                {
                    // 到站后先暂停调度任务，让机器人停稳
                    dispatch_client_.pauseRobotTask();
                    pause_sent_ = true;
                    stable_count_ = 0;
                    wait_timeout_ = 0;
                }

                // 判断角度是否稳定
                float delta = std::abs(curr_angle - last_angle_);
                if (delta > 180)
                    delta = 360 - delta;
                if (delta < 1.0f)
                {
                    stable_count_++;
                }
                else
                {
                    stable_count_ = 0;
                    dispatch_client_.pauseRobotTask();
                }
                last_angle_ = curr_angle;
                wait_timeout_++;

                // 满足稳定条件或等待超时，开始执行动作
                if (stable_count_ >= 2 || wait_timeout_ >= 10)
                {
                    int current_action_id = action_ids_[current_idx_];
                    std::cout << "\n到达站点 " << current_site_id << "，准备执行调度动作" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(stable_wait_ms_));

                    int action_ret = arm_.startAction(current_action_id);
                    if (action_ret != 0)
                    {
                        std::cerr << "[警告] 调度动作下发失败（ID: " << current_action_id << "），跳过动作，继续航行" << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                    else
                    {
                        const int ACTION_TIMEOUT_MAX = 150;
                        int action_wait_cnt = 0;
                        bool action_timeout = false;
                        while (!arm_.isFinished() && !is_stopped_)
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
                            action_wait_cnt++;
                            if (action_wait_cnt >= ACTION_TIMEOUT_MAX)
                            {
                                std::cerr << "[警告] 动作执行超时，强制结束动作等待，继续航行" << std::endl;
                                action_timeout = true;
                                break;
                            }
                        }
                        if (!action_timeout)
                        {
                            std::cout << "调度动作执行完成，恢复航行..." << std::endl;
                        }
                    }

                    // 无论动作成功失败，都恢复调度任务
                    dispatch_client_.resumeRobotTask();
                    has_left_site_ = false;
                    site_initialized_ = false;
                    stable_count_ = 0;
                    pause_sent_ = false;
                    wait_timeout_ = 0;
                    current_idx_++;
                }
            }
            // ========== 情况3：无自定义动作 + 无调度动作 → 直接过站 ==========
            else
            {
                stable_count_++;
                if (stable_count_ >= 1) {
                    has_left_site_ = false;
                    site_initialized_ = false;
                    stable_count_ = 0;
                    current_idx_++;
                }
            }
        } else {
            stable_count_ = 0;
            last_angle_ = curr_angle;
            wait_timeout_ = 0;
        }
    }

    std::cout << "\n全部任务执行完成" << std::endl;
    is_finished_ = true;
}

void TaskManager::stop() {
    is_stopped_ = true;
    dispatch_client_.stopRobotTask();
    is_finished_ = true;
}

bool TaskManager::isFinished() const {
    return is_finished_;
}

// ========== 新增：全局暂停任务 ==========
int TaskManager::pauseTask() {
    // 任务已结束，不允许暂停
    if (is_finished_) {
        return -1;
    }
    // 设置暂停标志
    bool expected = false;
    if (!is_paused_.compare_exchange_strong(expected, true)) {
        // 已经是暂停状态，直接返回成功
        return 0;
    }
    // 调用调度接口暂停机器人任务
    int ret = dispatch_client_.pauseRobotTask();
    if (ret != 0) {
        // 调度指令下发失败，回滚暂停标志
        is_paused_ = false;
        return -1;
    }
    std::cout << "[任务控制] 已下发暂停指令" << std::endl;
    return 0;
}

// ========== 新增：全局恢复任务 ==========
int TaskManager::resumeTask() {
    // 任务已结束，不允许恢复
    if (is_finished_) {
        return -1;
    }
    // 清除暂停标志
    bool expected = true;
    if (!is_paused_.compare_exchange_strong(expected, false)) {
        // 已经是运行状态，直接返回成功
        return 0;
    }
    // 调用调度接口恢复机器人任务
    int ret = dispatch_client_.resumeRobotTask();
    if (ret != 0) {
        // 调度指令下发失败，回滚暂停标志
        is_paused_ = true;
        return -1;
    }
    std::cout << "[任务控制] 已下发恢复指令" << std::endl;
    return 0;
}

// ========== 新增：查询暂停状态 ==========
bool TaskManager::isPaused() const {
    return is_paused_.load();
}
