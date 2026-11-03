#include "arm/arm_direct_controller.h"
#include <chrono>
#include <thread>
#include <cstdio>
#include <cmath>
#include <string>
#include <regex>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(deg)  ((deg) * M_PI / 180.0)

ArmDirectController::ArmDirectController()
    : connected_(false)
{
    client_ = std::make_shared<TcpArmClient>();
}

ArmDirectController::~ArmDirectController()
{
    disconnect();
}

bool ArmDirectController::connect(const std::string& ip, int port)
{
    arm_ip_ = ip;
    arm_port_ = port;
    
    connected_ = client_->connect(ip, port);
    defined_vars_.clear();      // 关节变量缓存清空
    defined_cart_vars_.clear(); // 笛卡尔变量缓存清空
    return connected_;
}



void ArmDirectController::disconnect()
{
    if (connected_) {
        client_->disconnect();
        connected_ = false;
        defined_vars_.clear();
    }
}

bool ArmDirectController::isConnected() const
{
    return connected_;
}

double ArmDirectController::deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

bool ArmDirectController::axisValid(double val, double min, double max, int axis_id)
{
    if (val < min || val > max)
    {
        printf("[限位告警] J%d 角度%.2f 超出范围[%.2f,%.2f]\n", axis_id, val, min, max);
        return false;
    }
    return true;
}

bool ArmDirectController::jointPosValid(double j1, double j2, double j3,
                                         double j4, double j5, double j6)
{
    bool ok = true;
    ok &= axisValid(j1, J1_MIN_DEG, J1_MAX_DEG, 1);
    ok &= axisValid(j2, J2_MIN_DEG, J2_MAX_DEG, 2);
    ok &= axisValid(j3, J3_MIN_DEG, J3_MAX_DEG, 3);
    ok &= axisValid(j4, J4_MIN_DEG, J4_MAX_DEG, 4);
    ok &= axisValid(j5, J5_MIN_DEG, J5_MAX_DEG, 5);
    ok &= axisValid(j6, J6_MIN_DEG, J6_MAX_DEG, 6);
    return ok;
}

bool ArmDirectController::addJointPoint(const std::string& name,
                                        double j1, double j2, double j3,
                                        double j4, double j5, double j6)
{
    if (!jointPosValid(j1,j2,j3,j4,j5,j6))
    {
        printf("[错误]点位%s 轴角度超限，禁止保存\n", name.c_str());
        return false;
    }
    std::vector<double> joints = {j1, j2, j3, j4, j5, j6};
    joint_points_[name] = joints;
    // 移除 defined_vars_.erase(name)，不再强制触发重定义
    printf("[点位]成功添加%s\n", name.c_str());
    return true;
}


void ArmDirectController::defineJointPos(const std::string& name,
                                         double j1, double j2, double j3,
                                         double j4, double j5, double j6)
{
    if (!connected_) return;
    // 已在控制器中定义过的变量，直接跳过，不再重复下发
    if (defined_vars_.count(name)) {
        return;
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
             "{Var --type=jointtarget --name=%s --value={%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,0,0,0,0}}",
             name.c_str(),
             deg2rad(j1), deg2rad(j2), deg2rad(j3),
             deg2rad(j4), deg2rad(j5), deg2rad(j6));
    
    auto res = sendCmdWithRetry(buf, 2000, 2);
    if (res.first && (isCmdSuccess(res.second) || isVarAlreadyExists(res.second))) {
        defined_vars_.insert(name);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}



bool ArmDirectController::isCmdSuccess(const std::string& reply)
{
    std::regex reg(R"("return_code"\s*:\s*(-?\d+))");
    std::smatch match;
    if (std::regex_search(reply, match, reg))
    {
        int code = std::stoi(match[1].str());
        return code >= 0;
    }
    return false;
}

bool ArmDirectController::isVarAlreadyExists(const std::string& reply)
{
    return reply.find("variable already exists") != std::string::npos;
}

bool ArmDirectController::clearFault(int retry)
{
    auto ret = sendCmdWithRetry("{Clear}", 3000, retry);
    if(!ret.first || !isCmdSuccess(ret.second))
        return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return true;
}

bool ArmDirectController::enable(int retry)
{
    if(!clearFault(retry))
        return false;
    auto res = sendCmdWithRetry("{Enable}", 5000, retry);
    if(!res.first || !isCmdSuccess(res.second))
        return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    return true;
}

bool ArmDirectController::stop()
{
    auto res = sendCmdWithRetry("{Stop}", 3000, 2);
    return res.first && isCmdSuccess(res.second);
}

bool ArmDirectController::moveToPoint(const std::string& name, int wait_ms, int retry)
{
    if (!connected_) return false;

    auto it = joint_points_.find(name);
    if (it == joint_points_.end())
    {
        printf("[错误]点位%s不存在\n", name.c_str());
        return false;
    }

    // ========== 新增：运动前二次校验关节限位 ==========
    const auto& ang = it->second;
    if (!jointPosValid(ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]))
    {
        printf("[错误]点位%s超出关节限位，拒绝执行\n", name.c_str());
        return false;
    }
    // ==================================================

    // 下发点位变量
    defineJointPos(name, ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]);

    std::string move_cmd = "{MoveAbsJ --jointtarget_var=" + name + "}";

    // 重试逻辑
    for (int i = 0; i <= retry; i++)
    {
        clearFault();
        auto send_res = sendCmdWithRetry(move_cmd, 10000, 1);
        
        if (send_res.first && isCmdSuccess(send_res.second))
        {
            printf("[运动]点位%s指令下发成功，等待%d毫秒\n", name.c_str(), wait_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            return true;
        }

        if (i < retry)
        {
            printf("[重试%d/%d]点位%s运动失败，清除故障后重试\n", i+1, retry, name.c_str());
            clearFault();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    printf("[最终失败]点位%s运动失败，已重试%d次\n", name.c_str(), retry);
    return false;
}



bool ArmDirectController::executePointList(const std::vector<std::string>& point_list, int stay_ms)
{
    if (!connected_) return false;
    printf("\n[机械臂]开始执行序列，共%zu个点位\n", point_list.size());
    for (size_t i = 0; i < point_list.size(); i++)
    {
        const auto& pt = point_list[i];
        printf("[%zu/%zu]执行点位%s\n", i+1, point_list.size(), pt.c_str());
        // 工作点位默认等待3000ms，可根据实测耗时调整
        if (!moveToPoint(pt, 3000))
        {
            printf("[异常]点位%s执行失败，自动回零后终止序列\n", pt.c_str());
            goSafeHome(5000); // 失败后优先回零，避免机械臂悬停在危险位置
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(stay_ms));
    }
    printf("[序列全部执行完毕，回安全零位]\n");
    goSafeHome(5000); // 回零默认等待5000ms
    return true;
}


bool ArmDirectController::goSafeHome(int wait_ms)
{
    // 与你Web端零位完全匹配：J1=0° J2=-90° J3=0° J4=-90° J5=0° J6=0°
    if (joint_points_.find("safe_home") == joint_points_.end()) {
        addJointPoint("safe_home", 0.0, -90.0, 0.0, -90.0, 0.0, 0.0);
    }
    return moveToPoint("safe_home", wait_ms);
}


std::pair<bool, std::string> ArmDirectController::sendCmd(const std::string& cmd, int timeout)
{
    // 发送前检测连接，断开则自动重连
    if (!connected_ || !client_->is_connected())
    {
        if (!arm_ip_.empty() && arm_port_ > 0)
        {
            printf("[重连]检测到连接断开，尝试重连 %s:%d\n", arm_ip_.c_str(), arm_port_);
            connected_ = client_->connect(arm_ip_, arm_port_);
            
            if (connected_)
            {
                defined_vars_.clear(); // 重连后重置变量缓存，重新同步点位
                clearFault();
                enable(); // 重连后自动重新使能机械臂
                printf("[重连]重连成功，机械臂已恢复就绪\n");
            }
            else
            {
                printf("[重连]重连失败\n");
                return {false, ""};
            }
        }
        else
        {
            return {false, ""};
        }
    }

    if (debug_log_enable_) {
        printf("[发送]%s\n", cmd.c_str());
    }
    auto res = client_->sendCommand(cmd, timeout);
    if (debug_log_enable_) {
        printf("[返回]%s\n", res.second.c_str());
    }
    return res;
}



std::pair<bool, std::string> ArmDirectController::sendCmdWithRetry(const std::string& cmd, int timeout, int retry)
{
    std::pair<bool, std::string> ret;
    for(int i=0; i<=retry; i++)
    {
        ret = sendCmd(cmd, timeout);
        if(ret.first && (isCmdSuccess(ret.second) || isVarAlreadyExists(ret.second)))
            return ret;
        printf("[重试%d/%d]指令失败，等待重发\n", i+1, retry);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    printf("[最终失败]重试%d次仍失败: %s\n", retry, cmd.c_str());
    return ret;
}


void ArmDirectController::setSpeedRatio(int ratio)
{
    // 限幅保护，防止超出范围
    speed_ratio_ = std::max(1, std::min(100, ratio));
    
    // 如果控制器支持单独下发速度指令，在这里补上即可，例如：
    // char buf[64];
    // snprintf(buf, sizeof(buf), "{SetSpeed %d}", speed_ratio_);
    // sendCmd(buf, 1000);
    
    printf("[速度]全局速度倍率设置为 %d%%\n", speed_ratio_);
}

bool ArmDirectController::sendRawCmd(const std::string& cmd, int timeout, int retry)
{
    auto res = sendCmdWithRetry(cmd, timeout, retry);
    return res.first && isCmdSuccess(res.second);
}


bool ArmDirectController::addCartPoint(const std::string& name,
                                       double x, double y, double z,
                                       double rx, double ry, double rz)
{
    // 简单范围校验（z 放宽到 -500mm，视觉抓取目标可能在工作台面以下）
    if (z < -500 || fabs(x) > 2000 || fabs(y) > 2000)
    {
        printf("[错误]笛卡尔点位%s超出工作范围\n", name.c_str());
        return false;
    }

    std::vector<double> pose = {x, y, z, rx, ry, rz};
    cart_points_[name] = pose;
    // 标记为未下发，下次运动时自动同步到控制器
    defined_cart_vars_.erase(name);
    printf("[笛卡尔点位]成功添加 %s: [%.1f, %.1f, %.1f, %.1f°, %.1f°, %.1f°]\n",
           name.c_str(), x, y, z, rx, ry, rz);
    return true;
}

void ArmDirectController::defineCartPos(const std::string& name,
                                        double x, double y, double z,
                                        double rx, double ry, double rz)
{
    if (!connected_) return;
    // 已定义过则跳过，消除重复警告
    if (defined_cart_vars_.count(name)) {
        return;
    }

    char buf[512];
    // 姿态角度转弧度，和关节角度保持一致；位置单位mm直接下发
    snprintf(buf, sizeof(buf),
             "{Var --type=robtarget --name=%s --value={%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,0,0,0,0}}",
             name.c_str(),
             x, y, z,
             rx * M_PI / 180.0,
             ry * M_PI / 180.0,
             rz * M_PI / 180.0);

    auto res = sendCmdWithRetry(buf, 2000, 2);
    if (res.first && (isCmdSuccess(res.second) || isVarAlreadyExists(res.second))) {
        defined_cart_vars_.insert(name);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}


bool ArmDirectController::moveCartJ(const std::string& name, int wait_ms, int retry)
{
    if (!connected_) return false;

    auto it = cart_points_.find(name);
    if (it == cart_points_.end())
    {
        printf("[错误]笛卡尔点位%s不存在\n", name.c_str());
        return false;
    }

    const auto& pose = it->second;
    // 下发点位变量到控制器（已定义则自动跳过）
    defineCartPos(name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    std::string move_cmd = "{MoveJ --robtarget_var=" + name + "}";

    // 失败重试逻辑，和关节运动保持一致
    for (int i = 0; i <= retry; i++)
    {
        clearFault();
        auto send_res = sendCmdWithRetry(move_cmd, 10000, 1);

        if (send_res.first && isCmdSuccess(send_res.second))
        {
            printf("[笛卡尔运动J]点位%s指令下发成功，等待%d毫秒\n", name.c_str(), wait_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            return true;
        }

        if (i < retry)
        {
            printf("[重试%d/%d]笛卡尔点位%s运动失败，清除故障后重试\n", i+1, retry, name.c_str());
            clearFault();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    printf("[最终失败]笛卡尔点位%s运动失败\n", name.c_str());
    return false;
}

bool ArmDirectController::moveCartL(const std::string& name, int wait_ms, int retry)
{
    if (!connected_) return false;

    auto it = cart_points_.find(name);
    if (it == cart_points_.end())
    {
        printf("[错误]笛卡尔点位%s不存在\n", name.c_str());
        return false;
    }

    const auto& pose = it->second;
    defineCartPos(name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    std::string move_cmd = "{MoveL --robtarget_var=" + name + "}";

    for (int i = 0; i <= retry; i++)
    {
        clearFault();
        auto send_res = sendCmdWithRetry(move_cmd, 10000, 1);

        if (send_res.first && isCmdSuccess(send_res.second))
        {
            printf("[笛卡尔运动L]点位%s指令下发成功，等待%d毫秒\n", name.c_str(), wait_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            return true;
        }

        if (i < retry)
        {
            printf("[重试%d/%d]笛卡尔直线运动%s失败，清除故障后重试\n", i+1, retry, name.c_str());
            clearFault();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    printf("[最终失败]笛卡尔直线运动%s失败\n", name.c_str());
    return false;
}
