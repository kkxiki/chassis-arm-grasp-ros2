#ifndef TCP_ARM_CLIENT_H
#define TCP_ARM_CLIENT_H
#include <string>
#include <cstdint>
#include <utility>

class TcpArmClient
{
public:
    TcpArmClient();
    ~TcpArmClient();

    // 连接机械臂控制器
    bool connect(const std::string& ip, int port = 5868);
    // 断开连接
    void disconnect();
    // 发送指令，返回<是否成功, 应答内容>
    std::pair<bool, std::string> sendCommand(const std::string& cmd, int timeout_ms = 3000);
    bool is_connected() const { return connected_; }

private:
    // 消息头结构体（40字节，与厂商定义完全一致）
    struct MsgHeader {
        uint32_t msg_size_;
        uint32_t msg_id_;
        uint64_t msg_type_;
        int64_t  reserved1_;
        int64_t  reserved2_;
        int64_t  reserved3_;
    };

    int sock_fd_;
    bool connected_;
    uint64_t seq_id_;
};

#endif // TCP_ARM_CLIENT_H
