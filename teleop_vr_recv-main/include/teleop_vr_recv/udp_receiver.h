#pragma once

#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include <netinet/in.h>

namespace teleop_vr {

/**
 * @brief UDP接收器类,用于接收VR手柄数据
 */
class UdpReceiver {
public:
    using DataCallback = std::function<void(const std::vector<uint8_t>&, size_t)>;

    /**
     * @brief 构造函数
     * @param host 监听地址(默认0.0.0.0)
     * @param port 监听端口(默认8080)
     */
    explicit UdpReceiver(const std::string& host = "0.0.0.0", int port = 8080);

    /**
     * @brief 析构函数
     */
    ~UdpReceiver();

    // 禁止拷贝
    UdpReceiver(const UdpReceiver&) = delete;
    UdpReceiver& operator=(const UdpReceiver&) = delete;

    /**
     * @brief 启动UDP接收器
     * @return 成功返回true,失败返回false
     */
    bool start();

    /**
     * @brief 停止UDP接收器
     */
    void stop();

    /**
     * @brief 检查接收器是否正在运行
     * @return 正在运行返回true,否则返回false
     */
    bool isRunning() const { return running_; }

    /**
     * @brief 设置数据接收回调函数
     * @param callback 回调函数
     */
    void setDataCallback(DataCallback callback) { data_callback_ = callback; }

    /**
     * @brief 接收一次数据(非阻塞)
     * @param timeout_ms 超时时间(毫秒)
     * @return 成功接收返回true,超时或错误返回false
     */
    bool receiveOnce(int timeout_ms = 100);

private:
    std::string host_;              // 监听地址
    int port_;                      // 监听端口
    int socket_fd_;                 // socket文件描述符
    bool running_;                  // 运行状态
    struct sockaddr_in server_addr_; // 服务器地址结构
    DataCallback data_callback_;    // 数据接收回调
    std::vector<uint8_t> recv_buffer_; // 接收缓冲区
};

} // namespace teleop_vr
