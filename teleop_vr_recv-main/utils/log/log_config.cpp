#include "log/log_config.h"
#include <iostream>
#include <filesystem>
#include <ctime>

namespace common_utils {

LogConfigManager& LogConfigManager::getInstance() {
  static LogConfigManager instance;
  return instance;
}

bool LogConfigManager::initialize(const std::string& logDir,
                                  const std::string& programName,
                                  LogLevel logLevel,
                                  bool enableConsole,
                                  int maxLogSizeMB) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (initialized_) {
    std::cout << "LogConfigManager already initialized" << std::endl;
    return true;
  }

  logDir_ = logDir;
  programName_ = programName;

  if (!createLogDirectory(logDir_)) {
    std::cerr << "Failed to create log directory: " << logDir_ << std::endl;
    return false;
  }

  try {
    // 初始化glog
    google::InitGoogleLogging(programName_.c_str());

    // 配置glog参数
    FLAGS_log_dir = logDir_;
    FLAGS_minloglevel = static_cast<int>(logLevel);
    FLAGS_alsologtostderr = enableConsole;
    FLAGS_colorlogtostderr = enableConsole;
    FLAGS_max_log_size = maxLogSizeMB;
    FLAGS_stop_logging_if_full_disk = true;
    FLAGS_logbufsecs = 0;   // 立即刷新日志
    FLAGS_v = 0;            // VLOG详细级别

    // 时间格式配置（注释掉不兼容的旧版本glog特性）
    // FLAGS_timestamp_in_logfile_name = true;  // 旧版本glog不支持
    FLAGS_log_prefix = true;
    // FLAGS_log_year_in_prefix = true;  // 旧版本glog不支持
    // FLAGS_log_utc_time = false;  // 旧版本glog不支持

    // 启用日志自动清理（7天后删除旧日志）
    // google::EnableLogCleaner(std::chrono::hours(24 * 7));  // 旧版本glog不支持

    initialized_ = true;
    LOG_INFO("LogConfigManager initialized successfully");
    LOG_INFO("Log directory: " << logDir_);
    LOG_INFO("Log level: " << static_cast<int>(logLevel));

    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to initialize glog: " << e.what() << std::endl;
    return false;
  }
}

void LogConfigManager::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    return;
  }

  // 刷新并关闭glog
  google::FlushLogFiles(google::GLOG_INFO);
  google::ShutdownGoogleLogging();

  initialized_ = false;
}

void LogConfigManager::setLogLevel(LogLevel level) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) {
    FLAGS_minloglevel = static_cast<int>(level);
    LOG_INFO("Log level changed to: " << static_cast<int>(level));
  }
}

bool LogConfigManager::setLogDirectory(const std::string& logDir) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!createLogDirectory(logDir)) {
    return false;
  }
  logDir_ = logDir;
  FLAGS_log_dir = logDir_;
  return true;
}

void LogConfigManager::enableConsoleOutput(bool enable) {
  std::lock_guard<std::mutex> lock(mutex_);
  FLAGS_alsologtostderr = enable;
  FLAGS_colorlogtostderr = enable;
}

void LogConfigManager::flushLogs() {
  google::FlushLogFiles(google::GLOG_INFO);
}

bool LogConfigManager::isInitialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

bool LogConfigManager::createLogDirectory(const std::string& logDir) {
  try {
    if (!std::filesystem::exists(logDir)) {
      std::filesystem::create_directories(logDir);
      std::cout << "Created log directory: " << logDir << std::endl;
    }
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to create log directory: " << e.what() << std::endl;
    return false;
  }
}

} // namespace common_utils
