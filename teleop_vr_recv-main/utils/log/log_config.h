#pragma once

#include <glog/logging.h>
#include <mutex>
#include <string>
#include <chrono>

namespace common_utils {

enum class LogLevel { INFO = 0, WARNING = 1, ERROR = 2, FATAL = 3 };

class LogConfigManager {
public:
  static LogConfigManager& getInstance();

  bool initialize(const std::string& logDir = "./logs",
                  const std::string& programName = "teleop_vr_recv",
                  LogLevel logLevel = LogLevel::INFO,
                  bool enableConsole = true,
                  int maxLogSizeMB = 100);

  void shutdown();
  void setLogLevel(LogLevel level);
  bool setLogDirectory(const std::string& logDir);
  void enableConsoleOutput(bool enable);
  void flushLogs();
  bool isInitialized() const;

private:
  LogConfigManager() = default;
  ~LogConfigManager() = default;

  // 禁止拷贝和赋值
  LogConfigManager(const LogConfigManager&) = delete;
  LogConfigManager& operator=(const LogConfigManager&) = delete;

  bool createLogDirectory(const std::string& logDir);

  std::string logDir_;
  std::string programName_;
  bool initialized_ = false;
  mutable std::mutex mutex_;
};

// 便利宏定义
#define LOG_INFO(msg) LOG(INFO) << "[INFO] " << msg
#define LOG_WARNING(msg) LOG(WARNING) << "[WARNING] " << msg
#define LOG_ERROR(msg) LOG(ERROR) << "[ERROR] " << msg
#define LOG_FATAL(msg) LOG(FATAL) << "[FATAL] " << msg

#ifdef NDEBUG
#define LOG_DEBUG(msg) do { } while (0)
#else
#define LOG_DEBUG(msg) DLOG(INFO) << "[DEBUG] " << msg
#endif

} // namespace common_utils
