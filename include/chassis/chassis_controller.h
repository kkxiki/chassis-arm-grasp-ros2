#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <string>
#include <vector>
#include <mutex>
#include <functional>
#include "dispatch/dispatch_client.h"
#include <nlohmann/json.hpp>
#include <atomic>
using json = nlohmann::json;

// ============================================================
// 统一错误码定义
// 约定：0 表示执行成功，负数表示各类失败场景
// ============================================================
enum ChassisErrorCode {
    CHASSIS_OK = 0,               // 执行成功
    CHASSIS_ERR_SEND_FAILED = -1, // 指令下发失败
    CHASSIS_ERR_TIMEOUT = -2,     // 运动执行超时
    CHASSIS_ERR_POSE_FAILED = -3, // 获取位姿失败
    CHASSIS_ERR_FAULT = -4,       // 底盘硬件故障/急停触发
    CHASSIS_ERR_CANCELED = -5,    // 任务被主动取消
    CHASSIS_ERR_STATE_INVALID = -6// 当前状态不允许执行该指令
};

// ============================================================
// 航点结构体：描述单个目标点位的坐标、角度与停留时间
// ============================================================
struct Waypoint {
    float x;       // 目标X坐标，单位：米
    float y;       // 目标Y坐标，单位：米
    float angle;   // 目标航向角，单位：度
    int stay_ms;   // 到位后停留时间，单位：毫秒
    std::vector<std::string> action_points; // 绑定的机械臂点位名称列表
};

// 航点到达回调函数类型
using WaypointArrivedCallback = std::function<void(size_t index, const Waypoint& wp)>;

/**
 * @brief 底盘控制器类
 * 包含两套独立的移动控制体系：
 * 1. 调度任务模式：通过8008端口调度系统执行任务，适合复杂多站点巡检
 * 2. 直控巡航模式：通过8767端口直接控制底盘运动，适合点对点精细运动
 */
class ChassisController {
public:
    /**
     * @brief 构造函数
     * @param chassis_url 底盘控制服务基础地址（端口8767）
     * @param dispatch_client 调度系统客户端引用，用于获取位姿
     */
    explicit ChassisController(const std::string& chassis_url, DispatchClient& dispatch_client);
    
    /**
     * @brief 析构函数
     */
    ~ChassisController();

    // ==================== 状态查询接口 ====================
    /**
     * @brief 获取机器人当前位姿（通过调度系统）
     * @param x 输出X坐标
     * @param y 输出Y坐标
     * @param angle 输出角度
     * @return 错误码
     */
    int getCurrentPose(float& x, float& y, float& angle);

    /**
     * @brief 查询是否正在执行直控巡航任务（线程安全）
     * @return true=巡航中，false=空闲
     */
    bool isCruising() const;

    // ==================== 调度原生动作接口（8008端口） ====================
    /**
     * @brief 底盘原地旋转指定角度（调度模式）
     * @param angle 旋转角度，单位：度，正为顺时针
     * @param speed 旋转速度，单位：度/秒
     * @return 0=下发成功，-1=失败
     */
    int rotateAngle(float angle, float speed = 30.0f);

    /**
     * @brief 底盘相对平移指定距离（调度模式）
     * @param angle 平移方向角，单位：度（前方右侧为正）
     * @param distance_mm 平移距离，单位：毫米
     * @param speed 平移速度，单位：m/s
     * @return 0=下发成功，-1=失败
     */
    int translateDistance(float angle, int distance_mm, float speed = 0.1f);

    // ==================== 直控移动接口（8767端口，新增） ====================
    /**
     * @brief 直控模式：启动底盘移动到指定目标点
     * @param target_x 目标X坐标，单位：米
     * @param target_y 目标Y坐标，单位：米
     * @param target_angle 目标朝向角，单位：度
     * @param speed 移动速度，单位：m/s
     * @param way_type 通道类型：1=后向，2=前向，3=双向（默认）
     * @return 0=下发成功，-1=失败
     */
    int startMoveTo(float target_x, float target_y, float target_angle, 
                    float speed, int way_type = 3);

    /**
     * @brief 直控模式：停止底盘移动
     * @return 0=下发成功，-1=失败
     */
    int stopMove();

    /**
     * @brief 直控模式：查询移动控制是否已启动
     * @param out_started 输出：是否正在移动
     * @return 0=查询成功，-1=失败
     */
    int isMoveStarted(bool& out_started);

    /**
     * @brief 直控模式：启动底盘原地旋转
     * @param angle 旋转角度/目标角度，单位：度
     * @param is_nav_angle true=旋转到绝对角度，false=旋转相对角度
     * @param speed 角速度，单位：度/秒
     * @return 0=下发成功，-1=失败
     */
    int startRotate(float angle, bool is_nav_angle = false, float speed = 30.0f);

    /**
     * @brief 直控模式：停止底盘旋转
     * @return 0=下发成功，-1=失败
     */
    int stopRotate();

    /**
     * @brief 直控模式：暂停底盘旋转
     * @return 0=下发成功，-1=失败
     */
    int pauseRotate();

    /**
     * @brief 直控模式：继续底盘旋转
     * @return 0=下发成功，-1=失败
     */
    int resumeRotate();

    // ==================== 静态工具函数 ====================
    /**
     * @brief 从JSON文件加载航点列表
     * @param file_path 航点文件路径
     * @param out_waypoints 输出参数：航点列表
     * @return true=加载成功，false=加载失败
     */
    static bool loadWaypointsFromFile(const std::string& file_path, std::vector<Waypoint>& out_waypoints);

    // ==================== 自实现直控巡航模式 ====================
    /**
     * @brief 执行多航点巡航任务（阻塞式运行，直控模式）
     * @details 逐点下发移动指令，到位自动停止，支持暂停、取消、回调
     * @param waypoints 航点列表
     * @param check_angle 是否校验角度到位
     * @return 错误码
     */
    int runCruise(const std::vector<Waypoint>& waypoints, bool check_angle = true);

    /**
     * @brief 暂停当前直控巡航任务
     * @return 错误码
     */
    int pauseCruise();

    /**
     * @brief 恢复被暂停的直控巡航任务
     * @return 错误码
     */
    int resumeCruise();

    /**
     * @brief 取消当前直控巡航任务
     * @return 错误码
     */
    int cancelCruise();

    /**
     * @brief 移动到单个指定位姿（直控模式）
     * @param x 目标X坐标
     * @param y 目标Y坐标
     * @param angle 目标角度
     * @param check_angle 是否校验角度到位
     * @return 错误码
     */
    int goToPose(float x, float y, float angle, bool check_angle = true);

    /**
     * @brief 返回原点位置（直控模式）
     * @param home_x 原点X坐标
     * @param home_y 原点Y坐标
     * @param home_angle 原点角度
     * @param check_angle 是否校验角度到位
     * @return 错误码
     */
    int goHome(float home_x, float home_y, float home_angle, bool check_angle = true);

    // ==================== 参数配置接口 ====================
    /**
     * @brief 设置运动速度参数
     * @param move_speed 直线行驶速度，m/s
     * @param rot_speed 旋转速度，度/秒
     */
    void setSpeed(float move_speed, float rot_speed);

    /**
     * @brief 设置指令失败重试次数
     * @param count 重试次数
     */
    void setRetryCount(int count);

    /**
     * @brief 注册航点到达回调函数
     * @param callback 回调函数
     */
    void setWaypointArrivedCallback(WaypointArrivedCallback callback);
    
private:
    private:
    std::string base_url_;          // 底盘直控服务基础地址（8767端口）
    DispatchClient& dispatch_client_; // 调度系统客户端引用

    // 运动参数
    float move_speed_;  // 直线行驶速度，单位m/s
    float rot_speed_;   // 旋转速度，单位度/秒
    int retry_count_;   // 指令失败重试次数

    // ========== 修改：状态标志全部改为原子变量，彻底避免死锁 ==========
    mutable std::mutex cruise_mutex_; // 保留互斥锁，后续可用于保护复杂资源
    std::atomic<bool> cruising_;      // 是否正在巡航（原子，多线程安全无锁）
    std::atomic<bool> pause_flag_;    // 暂停标志位（原子）
    std::atomic<bool> cancel_flag_;   // 取消标志位（原子）
    WaypointArrivedCallback arrived_callback_; // 航点到达回调

    // 固定阈值参数
    const int connect_timeout_s_ = 5;    // HTTP连接超时，秒
    const int total_timeout_s_ = 30;     // HTTP请求总超时，秒
    const float position_threshold_ = 0.1f; // 位置到位阈值，米
    const float angle_threshold_ = 5.0f;   // 角度到位阈值，度

    // ==================== 底盘原生HTTP接口（8767端口） ====================
    /**
     * @brief 发送GET请求到底盘直控服务
     * @param path 接口路径
     * @return JSON响应
     */
    json chassisHttpGet(const std::string& path);

    /**
     * @brief 发送POST请求到底盘直控服务
     * @param path 接口路径
     * @param body 请求体JSON
     * @return JSON响应
     */
    json chassisHttpPost(const std::string& path, const json& body);

    /**
     * @brief 检查直控接口返回是否成功（成功码为2000）
     * @param resp 接口响应JSON
     * @return true=成功
     */
    bool checkChassisResponse(const json& resp) const;
};

#endif // CHASSIS_CONTROLLER_H