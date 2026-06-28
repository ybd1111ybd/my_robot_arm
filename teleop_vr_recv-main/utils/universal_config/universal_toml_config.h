#pragma once

#include <toml.hpp>
#include <string>
#include <vector>
#include <stdexcept>
#include <mutex>
#include <memory>
#include <iostream>

namespace universal_config {

class UniversalTomlConfig {
public:
    virtual ~UniversalTomlConfig() = default;

    virtual bool loadFromFile(const std::string &config_file) = 0;
    virtual bool saveToFile(const std::string &config_file) const = 0;
    virtual bool validate() const { return true; }
    virtual void resetToDefaults() {}

protected:
    // 辅助函数：安全地从TOML中读取值
    template<typename T>
    T getTomlValue(const toml::value &data, const std::string &section,
                   const std::string &key, const T &defaultValue) const {
        try {
            if (data.contains(section) && data.at(section).contains(key)) {
                auto value = toml::get<T>(data.at(section).at(key));
                return value;
            }
        } catch (const std::exception &) {
            // 如果类型转换失败，使用默认值
        }
        return defaultValue;
    }

    // 加载字符串数组
    void loadStringArray(const toml::value &data, const std::string &section,
                        const std::string &key, std::vector<std::string> &names) const {
        try {
            if (data.contains(section) && data.at(section).contains(key)) {
                auto array = toml::get<std::vector<std::string>>(
                    data.at(section).at(key));
                names = array;
            }
        } catch (const std::exception &) {}
    }
};

// 全局配置管理器 - 单例模式
template<typename ConfigType>
class GlobalConfigManager {
    static_assert(std::is_base_of<UniversalTomlConfig, ConfigType>::value,
                 "ConfigType must inherit from UniversalTomlConfig");

private:
    std::unique_ptr<ConfigType> config_;
    std::string config_file_path_;
    bool initialized_;
    mutable std::mutex mutex_;

    GlobalConfigManager() : initialized_(false) {}

public:
    static GlobalConfigManager& getInstance() {
        static GlobalConfigManager instance;
        return instance;
    }

    // 禁用拷贝
    GlobalConfigManager(const GlobalConfigManager&) = delete;
    GlobalConfigManager& operator=(const GlobalConfigManager&) = delete;

    bool initialize(const std::string& config_file_path) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (initialized_) {
            std::cerr << "Warning: GlobalConfigManager already initialized" << std::endl;
            return true;
        }

        try {
            config_ = std::make_unique<ConfigType>();
            config_file_path_ = config_file_path;

            if (!config_->loadFromFile(config_file_path_)) {
                std::cerr << "Failed to load configuration from: "
                         << config_file_path_ << std::endl;
                config_.reset();
                return false;
            }

            initialized_ = true;
            return true;

        } catch (const std::exception& e) {
            std::cerr << "Exception during configuration initialization: "
                     << e.what() << std::endl;
            config_.reset();
            return false;
        }
    }

    bool isInitialized() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return initialized_ && config_ != nullptr;
    }

    ConfigType& getConfig() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_ || !config_) {
            throw std::runtime_error("Configuration not initialized");
        }
        return *config_;
    }

    const ConfigType& getConfig() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_ || !config_) {
            throw std::runtime_error("Configuration not initialized");
        }
        return *config_;
    }

    bool reload() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_ || !config_) {
            return false;
        }
        return config_->loadFromFile(config_file_path_);
    }

    bool save() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_ || !config_) {
            return false;
        }
        return config_->saveToFile(config_file_path_);
    }

    bool validate() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_ || !config_) {
            return false;
        }
        return config_->validate();
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        config_.reset();
        config_file_path_.clear();
        initialized_ = false;
    }
};

} // namespace universal_config
