#include "teleop_vr_recv/udp_receiver.h"
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <cerrno>
#include <iostream>

namespace teleop_vr {

UdpReceiver::UdpReceiver(const std::string& host, int port)
    : host_(host)
    , port_(port)
    , socket_fd_(-1)
    , running_(false)
    , recv_buffer_(1024)  // 预分配1KB缓冲区
{
    std::memset(&server_addr_, 0, sizeof(server_addr_));
}

UdpReceiver::~UdpReceiver() {
    stop();
}

bool UdpReceiver::start() {
    // 1. 创建UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
        return false;
    }

    // 2. 设置地址重用
    int reuse = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "Failed to set SO_REUSEADDR: " << strerror(errno) << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // 3. 设置接收超时(100ms)
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        std::cerr << "Failed to set SO_RCVTIMEO: " << strerror(errno) << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // 4. 配置服务器地址
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);

    if (host_ == "0.0.0.0") {
        server_addr_.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_pton(AF_INET, host_.c_str(), &server_addr_.sin_addr) <= 0) {
            std::cerr << "Invalid address: " << host_ << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }
    }

    // 5. 绑定地址
    if (bind(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        std::cerr << "Failed to bind to " << host_ << ":" << port_
                  << " - " << strerror(errno) << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    running_ = true;
    std::cout << "UDP receiver started on " << host_ << ":" << port_ << std::endl;
    return true;
}

void UdpReceiver::stop() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    running_ = false;
}

bool UdpReceiver::receiveOnce(int timeout_ms) {
    if (!running_ || socket_fd_ < 0) {
        return false;
    }

    // 更新超时设置(如果与当前不同)
    if (timeout_ms > 0) {
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    ssize_t recv_len = recvfrom(
        socket_fd_,
        recv_buffer_.data(),
        recv_buffer_.size(),
        0,
        (struct sockaddr*)&client_addr,
        &addr_len
    );

    if (recv_len > 0) {
        // 成功接收数据,调用回调
        if (data_callback_) {
            data_callback_(recv_buffer_, static_cast<size_t>(recv_len));
        }
        return true;
    } else if (recv_len == 0) {
        // 连接关闭(对于UDP不太可能发生)
        return false;
    } else {
        // recv_len < 0,检查错误类型
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // 超时,这是正常情况
            return false;
        }
        // 其他错误
        std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
        return false;
    }
}

} // namespace teleop_vr
