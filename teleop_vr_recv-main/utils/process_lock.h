#ifndef TELEOP_VR_RECV_PROCESS_LOCK_H
#define TELEOP_VR_RECV_PROCESS_LOCK_H

#include <string>
#include <iostream>
#include <unistd.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <cerrno>
#include <cstring>
#include <filesystem>

namespace teleop_vr {

/**
 * @brief 进程锁类 - 确保进程只能启动一次
 *
 * 使用文件锁机制（flock）来确保同一时间只有一个进程实例在运行。
 * 当进程异常退出时，操作系统会自动释放文件锁。
 */
class ProcessLock {
public:
    /**
     * @brief 构造函数
     * @param lock_file_path 锁文件路径（默认使用 /tmp/teleop_vr_recv.lock）
     */
    explicit ProcessLock(const std::string& lock_file_path = "/tmp/teleop_vr_recv.lock")
        : lock_file_path_(lock_file_path), lock_fd_(-1), is_locked_(false) {
    }

    /**
     * @brief 析构函数 - 自动释放锁
     */
    ~ProcessLock() {
        unlock();
    }

    // 禁止拷贝和赋值
    ProcessLock(const ProcessLock&) = delete;
    ProcessLock& operator=(const ProcessLock&) = delete;

    /**
     * @brief 尝试获取锁
     * @return true 如果成功获取锁，false 如果锁已被其他进程持有
     */
    bool tryLock() {
        if (is_locked_) {
            std::cerr << "⚠️  警告: 锁已经被当前进程持有" << std::endl;
            return true;
        }

        // 确保锁文件所在目录存在
        std::filesystem::path lock_path(lock_file_path_);
        std::filesystem::path lock_dir = lock_path.parent_path();

        if (!lock_dir.empty() && !std::filesystem::exists(lock_dir)) {
            try {
                std::filesystem::create_directories(lock_dir);
            } catch (const std::exception& e) {
                std::cerr << "❌ 错误: 无法创建锁文件目录 " << lock_dir
                          << ": " << e.what() << std::endl;
                return false;
            }
        }

        // 打开或创建锁文件
        lock_fd_ = open(lock_file_path_.c_str(), O_CREAT | O_RDWR, 0666);
        if (lock_fd_ == -1) {
            std::cerr << "❌ 错误: 无法打开锁文件 " << lock_file_path_
                      << ": " << std::strerror(errno) << std::endl;
            return false;
        }

        // 尝试获取排他锁（非阻塞）
        if (flock(lock_fd_, LOCK_EX | LOCK_NB) == -1) {
            if (errno == EWOULDBLOCK) {
                // 锁已被其他进程持有
                std::cerr << "\n" << std::string(70, '=') << std::endl;
                std::cerr << "❌ 启动失败: teleop_vr_recv 节点已经在运行中！" << std::endl;
                std::cerr << std::string(70, '=') << std::endl;
                std::cerr << "\n📋 详细信息:" << std::endl;
                std::cerr << "  • 锁文件: " << lock_file_path_ << std::endl;
                std::cerr << "  • 原因: 另一个 teleop_vr_recv 进程正在运行" << std::endl;
                std::cerr << "\n💡 解决方案:" << std::endl;
                std::cerr << "  1. 检查是否有其他 teleop_vr_recv 进程在运行:" << std::endl;
                std::cerr << "     ps aux | grep teleop_vr_recv" << std::endl;
                std::cerr << "  2. 如果需要重启，请先停止现有进程:" << std::endl;
                std::cerr << "     pkill -9 teleop_vr_recv" << std::endl;
                std::cerr << "  3. 或者使用 ROS2 命令停止节点:" << std::endl;
                std::cerr << "     ros2 lifecycle set /teleop_vr_recv_node shutdown" << std::endl;
                std::cerr << std::string(70, '=') << std::endl << std::endl;
            } else {
                std::cerr << "❌ 错误: 获取文件锁失败: " << std::strerror(errno) << std::endl;
            }
            close(lock_fd_);
            lock_fd_ = -1;
            return false;
        }

        // 写入当前进程的PID到锁文件
        if (ftruncate(lock_fd_, 0) == -1) {
            std::cerr << "⚠️  警告: 无法清空锁文件: " << std::strerror(errno) << std::endl;
        }

        pid_t pid = getpid();
        std::string pid_str = std::to_string(pid) + "\n";
        if (write(lock_fd_, pid_str.c_str(), pid_str.length()) == -1) {
            std::cerr << "⚠️  警告: 无法写入PID到锁文件: " << std::strerror(errno) << std::endl;
        }

        is_locked_ = true;

        std::cout << "\n✅ 成功获取进程锁" << std::endl;
        std::cout << "  • 锁文件: " << lock_file_path_ << std::endl;
        std::cout << "  • 进程PID: " << pid << std::endl << std::endl;

        return true;
    }

    /**
     * @brief 释放锁
     */
    void unlock() {
        if (!is_locked_) {
            return;
        }

        if (lock_fd_ != -1) {
            // 释放文件锁
            flock(lock_fd_, LOCK_UN);
            close(lock_fd_);
            lock_fd_ = -1;

            std::cout << "🔓 已释放进程锁: " << lock_file_path_ << std::endl;
        }

        is_locked_ = false;
    }

    /**
     * @brief 检查是否已获取锁
     * @return true 如果已获取锁
     */
    bool isLocked() const {
        return is_locked_;
    }

    /**
     * @brief 获取锁文件路径
     * @return 锁文件路径
     */
    std::string getLockFilePath() const {
        return lock_file_path_;
    }

private:
    std::string lock_file_path_;  // 锁文件路径
    int lock_fd_;                 // 锁文件描述符
    bool is_locked_;              // 是否已获取锁
};

} // namespace teleop_vr

#endif // TELEOP_VR_RECV_PROCESS_LOCK_H
