#ifndef ARM_DIRECT_CONTROLLER_H
#define ARM_DIRECT_CONTROLLER_H

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <chrono>
#include "arm/tcp_arm_client.h"

// 机械臂关节限位
#define J1_MIN_DEG -170.0
#define J1_MAX_DEG  170.0
#define J2_MIN_DEG -170.0
#define J2_MAX_DEG   60.0
#define J3_MIN_DEG -160.0
#define J3_MAX_DEG  160.0
#define J4_MIN_DEG -180.0
#define J4_MAX_DEG  180.0
#define J5_MIN_DEG -120.0
#define J5_MAX_DEG  120.0
#define J6_MIN_DEG -360.0
#define J6_MAX_DEG  360.0

class ArmDirectController
{
public:
    ArmDirectController();
    ~ArmDirectController();

    // 连接管理
    bool connect(const std::string& ip, int port);
    void disconnect();
    bool isConnected() const;

    // 基础控制（带重试）
    bool clearFault(int retry = 3);
    bool enable(int retry = 3);
    bool stop();

    // 添加关节点位(角度)，自动限位校验
    bool addJointPoint(const std::string& name,
                       double j1, double j2, double j3,
                       double j4, double j5, double j6);

    // 下发变量点位(弧度)，自动去重
    void defineJointPos(const std::string& name,
                        double j1, double j2, double j3,
                        double j4, double j5, double j6);

    // 运动到指定位点，wait_ms为运动等待时长
    bool moveToPoint(const std::string& name, int wait_ms = 5000, int retry = 2);

    // 批量执行点位序列
    bool executePointList(const std::vector<std::string>& point_list, int stay_ms = 200);

    // 回安全零位（6轴全0°）
    bool goSafeHome(int wait_ms = 8000);

    // 设置全局速度倍率 1~100，单位%
    void setSpeedRatio(int ratio);

    // 发送原始控制指令（用于抓手/自定义指令，如 {SetHandState --state=0 --hand_id=0}）
    bool sendRawCmd(const std::string& cmd, int timeout = 5000, int retry = 2);

    // 添加笛卡尔点位（单位：mm、角度°）
    bool addCartPoint(const std::string& name,
                      double x, double y, double z,
                      double rx, double ry, double rz);

    // 笛卡尔关节插补运动（MoveJ：路径最快，适合点位间快速移动）
    bool moveCartJ(const std::string& name, int wait_ms, int retry = 1);

    // 笛卡尔直线插补运动（MoveL：TCP走严格直线，适合作业路径）
    bool moveCartL(const std::string& name, int wait_ms, int retry = 1);



private:

    // 笛卡尔点位缓存：name -> {x, y, z, rx, ry, rz}  单位：mm、度
    std::unordered_map<std::string, std::vector<double>> cart_points_;
    // 已下发到控制器的笛卡尔变量名缓存
    std::unordered_set<std::string> defined_cart_vars_;


    bool debug_log_enable_ = true; // true=全开日志，false=仅关键日志

    
    int speed_ratio_ = 100; // 默认全速

    // 保存连接参数，用于自动重连
    std::string arm_ip_;
    int arm_port_ = 0;

    // 底层发送带重试
    std::pair<bool, std::string> sendCmdWithRetry(const std::string& cmd, int timeout, int retry);
    // 基础发送
    std::pair<bool, std::string> sendCmd(const std::string& cmd, int timeout);
    // 解析返回值，判断指令是否执行成功
    bool isCmdSuccess(const std::string& reply);
    // 判断是否为“变量已存在”的警告（不算失败）
    bool isVarAlreadyExists(const std::string& reply);

    double deg2rad(double deg);
    bool axisValid(double val, double min, double max, int axis_id);
    bool jointPosValid(double j1, double j2, double j3,
                       double j4, double j5, double j6);

    
    // 内部：下发笛卡尔点位变量到控制器
    void defineCartPos(const std::string& name,
                       double x, double y, double z,
                       double rx, double ry, double rz);

    std::shared_ptr<TcpArmClient> client_;
    bool connected_;
    // 本地点位缓存
    std::unordered_map<std::string, std::vector<double>> joint_points_;
    // 已在控制器中定义的变量缓存
    std::unordered_set<std::string> defined_vars_;
};

#endif // ARM_DIRECT_CONTROLLER_H
