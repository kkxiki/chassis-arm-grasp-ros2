#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <vector>
#include <utility>
#include <atomic>  // 原子类型，保证多线程安全
#include "dispatch/dispatch_client.h"
#include "chassis/chassis_controller.h"
#include "arm/arm_controller.h"
#include "arm/arm_direct_controller.h"


class TaskManager {
public:
    TaskManager(DispatchClient& dispatch, ChassisController& chassis, ArmController& arm,
                ArmDirectController* arm_direct = nullptr);

    ~TaskManager() = default;


    // 配置站点绑定的机械臂点位序列
    void setSiteActionMap(const std::map<int, std::vector<std::string>>& site_action_map);
    /**
     * @brief 启动任务执行（阻塞运行，建议放在子线程调用）
     * @param map_id 地图ID
     * @param site_ids 站点ID列表
     * @param action_ids 每个站点对应的动作ID列表
     */
    void runTask(int map_id, const std::vector<int>& site_ids, const std::vector<int>& action_ids);

    /**
     * @brief 停止当前任务（线程安全，终止整个任务）
     */
    void stop();

    /**
     * @brief 查询任务是否完成（线程安全）
     * @return true=任务已结束
     */
    bool isFinished() const;

    // ========== 新增：全局暂停/继续接口 ==========
    /**
     * @brief 暂停当前航行任务（线程安全，可随时调用）
     * @details 暂停后机器人停止移动，保持当前位置和任务进度
     * @return 0=指令下发成功，-1=状态不允许暂停
     */
    int pauseTask();

    /**
     * @brief 恢复被暂停的航行任务（线程安全）
     * @details 从暂停位置继续往下一个站点行驶
     * @return 0=指令下发成功，-1=状态不允许恢复
     */
    int resumeTask();

    /**
     * @brief 查询任务是否处于暂停状态（线程安全）
     * @return true=已暂停
     */
    bool isPaused() const;
    // ==============================================

private:

    
    std::map<int, std::vector<std::string>> site_action_map_; // 站点ID -> 点位名称列表


    DispatchClient& dispatch_client_;
    ChassisController& chassis_;
    ArmController& arm_;
    ArmDirectController* arm_direct_;                     // 直连机械臂指针

    // 任务状态原子变量，多线程读写安全
    std::atomic<bool> is_finished_;   // 任务是否结束
    std::atomic<bool> is_stopped_;    // 外部停止指令
    std::atomic<bool> is_paused_;     // 外部暂停指令 【新增】

    // 站点与动作数据
    std::vector<std::pair<float, float>> site_coords_;
    std::vector<int> action_ids_;
    int current_idx_;

    // 站点判定状态变量（仅任务线程内部使用，无需原子）
    bool has_left_site_;
    bool site_initialized_;
    int stable_count_;
    float last_angle_;
    bool pause_sent_;
    int wait_timeout_;

    // 固定阈值参数
    const float arrival_dist_threshold_;
    const float leave_factor_;
    const int stable_wait_ms_;
};

#endif // TASK_MANAGER_H