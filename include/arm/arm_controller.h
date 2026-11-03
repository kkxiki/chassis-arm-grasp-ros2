#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <string>
#include <vector>
#include "dispatch/dispatch_client.h"

/**
 * @brief 机械臂控制器类（调度系统模式）
 * 通过调度系统HTTP接口下发动作、查询状态
 */
class ArmController {
public:
    explicit ArmController(DispatchClient& dispatch_client);
    ~ArmController() = default;

    // 启动指定ID的动作（非阻塞）
    int startAction(int action_id);
    // 按类型启动动作（带参数，非阻塞）
    int startActionByType(int type, const std::vector<std::string>& param_list);
    // 查询动作是否执行完成
    bool isFinished() const;
    // 查询详细运行状态
    int getStatus(bool& is_finished, bool& is_error, std::string& error_reason);
    // 阻塞执行机械臂动作，直到完成/失败/外部终止
    int executeActionBlocking(int action_id, volatile bool* stop_flag = nullptr, int check_interval_ms = 1000);

private:
    DispatchClient& dispatch_client_;
};

#endif // ARM_CONTROLLER_H