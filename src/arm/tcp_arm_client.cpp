#include "arm/tcp_arm_client.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <vector>
#include <iostream>
#include <thread>

TcpArmClient::TcpArmClient()
    : sock_fd_(-1)
    , connected_(false)
    , seq_id_(0)
{
}

TcpArmClient::~TcpArmClient()
{
    disconnect();
}

bool TcpArmClient::connect(const std::string& ip, int port)
{
    sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
        std::cerr << "[TcpArmClient] 创建socket失败" << std::endl;
        return false;
    }

    // 设置收发超时
    struct timeval tv;
    tv.tv_sec = 8;
    tv.tv_usec = 0;
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    // 禁用Nagle减少粘包
    int flag = 1;
    setsockopt(sock_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);

    int ret = ::connect(sock_fd_, (struct sockaddr*)&addr, sizeof(addr));
    if (ret < 0) {
        std::cerr << "[TcpArmClient] 连接失败: " << strerror(errno) << std::endl;
        close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    connected_ = true;
    std::cout << "[TcpArmClient] 连接成功: " << ip << ":" << port << std::endl;
    return true;
}

void TcpArmClient::disconnect()
{
    if (connected_) {
        close(sock_fd_);
        sock_fd_ = -1;
        connected_ = false;
    }
}

// 读取指定字节数，超时返回false
static bool recv_bytes(int fd, void* buf, size_t len)
{
    size_t total = 0;
    char* ptr = static_cast<char*>(buf);
    while (total < len) {
        ssize_t n = recv(fd, ptr + total, len - total, 0);
        if (n <= 0) {
            return false;
        }
        total += n;
    }
    return true;
}

std::pair<bool, std::string> TcpArmClient::sendCommand(const std::string& cmd, int timeout_ms)
{
    if (!connected_) {
        return {false, "未连接控制器"};
    }

    // 1. 组装发送帧（完全对齐arm_ws原生格式）
    MsgHeader header;
    memset(&header, 0, sizeof(header));
    header.msg_size_ = static_cast<uint32_t>(cmd.size());
    header.msg_id_ = 8888;
    header.msg_type_ = 1;
    header.reserved1_ = ++seq_id_;
    header.reserved2_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    header.reserved3_ = 202401;

    size_t total_len = 1 + sizeof(MsgHeader) + cmd.size();
    std::vector<char> send_buf(total_len);
    send_buf[0] = 0x22;
    memcpy(send_buf.data() + 1, &header, sizeof(MsgHeader));
    memcpy(send_buf.data() + 1 + sizeof(MsgHeader), cmd.data(), cmd.size());

    // 2. 发送指令
    ssize_t sent = send(sock_fd_, send_buf.data(), total_len, 0);
    if (sent != static_cast<ssize_t>(total_len)) {
        return {false, "发送数据不完整"};
    }

    // 3. 帧同步：查找0x22标识头，丢弃状态推送帧
    uint8_t byte = 0;
    int skip_count = 0;
    const int max_skip = 4096;
    while (skip_count < max_skip) {
        if (!recv_bytes(sock_fd_, &byte, 1)) {
            return {false, "接收应答超时"};
        }
        if (byte == 0x22) {
            break;
        }
        skip_count++;
    }

    if (skip_count >= max_skip) {
        return {false, "帧同步失败，未找到有效帧头"};
    }

    // 4. 读取应答头
    MsgHeader recv_header;
    if (!recv_bytes(sock_fd_, &recv_header, sizeof(MsgHeader))) {
        return {false, "接收应答头不完整"};
    }

    // 5. 读取应答体
    uint32_t body_len = recv_header.msg_size_;
    if (body_len > 1024 * 1024) {
        return {false, "应答数据过长，异常"};
    }

    std::vector<char> recv_body(body_len);
    if (body_len > 0) {
        if (!recv_bytes(sock_fd_, recv_body.data(), body_len)) {
            return {false, "接收应答体不完整"};
        }
    }

    std::string reply(recv_body.begin(), recv_body.end());
    return {true, reply};
}
