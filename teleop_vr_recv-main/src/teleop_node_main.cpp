#include "teleop_vr_recv/teleop_node.h"
#include "teleop_vr_recv/teleop_vr_config.h"
#include "log/log_config.h"
#include "process_lock.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>
#include <iostream>

// 全局变量,用于信号处理
volatile sig_atomic_t g_shutdown_requested = 0;

/**
 * @brief 信号处理函数
 * @param signum 信号编号
 */
void signalHandler(int signum) {
    (void)signum;  // 避免未使用警告
    g_shutdown_requested = 1;
}

/**
 * @brief 初始化配置文件
 * @return true 成功, false 失败
 */
bool initConfig() {
    std::cout << "Loading configuration..." << std::endl;

    try {
        // 使用ament_index_cpp获取package share目录
        std::string config_path =
            ament_index_cpp::get_package_share_directory("teleop_vr_recv") +
            "/config/teleop_vr_recv.toml";

        // 使用全局配置管理器初始化配置
        if (!teleop_vr::initConfig(config_path)) {
            std::cerr << "Failed to initialize configuration" << std::endl;
            return false;
        }

        if (!teleop_vr::GlobalConfigManager::getInstance().validate()) {
            std::cerr << "Configuration validation failed" << std::endl;
            return false;
        }

        std::cout << "Configuration loaded successfully from: " << config_path
                  << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load configuration: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 初始化日志系统
 */
void initLog() {
    // 从配置文件读取日志参数
    std::string log_dir = "./log";
    std::string log_level = "info";
    int max_log_size_mb = 100;
    bool enable_console = true;

    if (teleop_vr::isConfigInitialized()) {
        const auto& config = teleop_vr::getConfigConst();
        log_dir = config.logging.dir;
        log_level = config.logging.level;
        max_log_size_mb = config.logging.max_size_mb;
        enable_console = true;  // 总是启用控制台输出
    }

    // 转换日志级别
    common_utils::LogLevel level = common_utils::LogLevel::INFO;
    if (log_level == "warning") {
        level = common_utils::LogLevel::WARNING;
    } else if (log_level == "error") {
        level = common_utils::LogLevel::ERROR;
    } else if (log_level == "fatal") {
        level = common_utils::LogLevel::FATAL;
    }

    // 初始化日志系统
    auto& logManager = common_utils::LogConfigManager::getInstance();
    if (!logManager.initialize(log_dir, "teleop_vr_recv", level, enable_console, max_log_size_mb)) {
        std::cerr << "Failed to initialize log system" << std::endl;
    } else {
        std::cout << "Log system initialized successfully" << std::endl;
        std::cout << "  Log directory: " << log_dir << std::endl;
        std::cout << "  Log level: " << log_level << std::endl;
    }
}

/**
 * @brief 打印配置信息
 */
void printAllConfig() {
    if (!teleop_vr::isConfigInitialized()) {
        std::cerr << "Configuration not initialized!" << std::endl;
        return;
    }

    const auto& config = teleop_vr::getConfigConst();

    std::cout << "\n============================================================" << std::endl;
    std::cout << "Configuration Summary:" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "UDP Settings:" << std::endl;
    std::cout << "  Host: " << config.udp.host << std::endl;
    std::cout << "  Port: " << config.udp.port << std::endl;
    std::cout << "  Enable UDP Receive: " << (config.udp.enable_udp_receive ? "true" : "false") << std::endl;
    std::cout << "\nPublish Settings:" << std::endl;
    std::cout << "  Mode: " << config.publish.mode << std::endl;
    std::cout << "  Publish Debug Raw: " << (config.publish.publish_debug_raw ? "true" : "false") << std::endl;
    std::cout << "\nLog Settings:" << std::endl;
    std::cout << "  Level: " << config.logging.level << std::endl;
    std::cout << "  Directory: " << config.logging.dir << std::endl;
    std::cout << "  Max Size (MB): " << config.logging.max_size_mb << std::endl;
    std::cout << "\nGripper Settings:" << std::endl;
    std::cout << "  Default Force: " << config.gripper.default_force << "%" << std::endl;
    std::cout << "============================================================\n" << std::endl;
}

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 退出码
 */
int main(int argc, char** argv) {
    // 0. 创建进程锁，确保只有一个实例运行
    teleop_vr::ProcessLock process_lock("/tmp/teleop_vr_recv.lock");

    if (!process_lock.tryLock()) {
        // 锁获取失败，说明已有实例在运行
        std::cerr << "⛔ teleop_vr_recv node startup failed!" << std::endl;
        return 1;
    }

    // 1. 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // 2. 初始化配置文件
    if (!initConfig()) {
        std::cerr << "Failed to load configuration, using defaults..." << std::endl;
    }

    // 3. 初始化日志系统
    initLog();

    // 4. 打印配置信息
    printAllConfig();

    // 5. 初始化ROS2
    rclcpp::init(argc, argv);

    // 6. 打印启动信息
    LOG_INFO("============================================================");
    LOG_INFO("UDP VR Data Receiver + ROS2 Joint Command Publisher");
    LOG_INFO("============================================================");
    LOG_INFO("Press Ctrl+C to stop the program");
    LOG_INFO("============================================================");

    try {
        // 7. 创建节点
        auto node = std::make_shared<teleop_vr::TeleopVrRecvNode>();

        // 8. 主循环
        LOG_INFO("Entering main loop...");

        while (rclcpp::ok() && !g_shutdown_requested) {
            // 运行节点主循环(UDP接收 + ROS2回调)
            node->run();

            // 短暂休眠,避免CPU占用过高
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }

        LOG_INFO("Shutting down...");

    } catch (const std::exception& e) {
        LOG_ERROR("Exception: " << e.what());
        rclcpp::shutdown();
        common_utils::LogConfigManager::getInstance().shutdown();
        process_lock.unlock();
        return 1;
    }

    // 9. 清理资源
    LOG_INFO("Cleaning up resources...");
    rclcpp::shutdown();
    common_utils::LogConfigManager::getInstance().shutdown();
    process_lock.unlock();

    std::cout << "\nProgram terminated successfully" << std::endl;

    return 0;
}
