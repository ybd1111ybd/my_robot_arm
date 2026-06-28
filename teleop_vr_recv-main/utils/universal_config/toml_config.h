#pragma once

#include "universal_toml_config.h"

// 定义项目特定的全局配置管理接口的宏
#define DEFINE_GLOBAL_CONFIG_MANAGER(NAMESPACE_NAME, CONFIG_TYPE) \
namespace NAMESPACE_NAME { \
    using GlobalConfigManager = universal_config::GlobalConfigManager<CONFIG_TYPE>; \
    static const CONFIG_TYPE* config_ = nullptr; \
    \
    inline const CONFIG_TYPE*& getGlobalConfigRef() { \
        static const CONFIG_TYPE* g_config = nullptr; \
        return g_config; \
    } \
    \
    inline bool initConfig(const std::string& config_file_path) { \
        bool result = GlobalConfigManager::getInstance().initialize(config_file_path); \
        if (result) { \
            config_ = &GlobalConfigManager::getInstance().getConfig(); \
            getGlobalConfigRef() = config_; \
        } \
        return result; \
    } \
    \
    inline bool isConfigInitialized() { \
        return GlobalConfigManager::getInstance().isInitialized(); \
    } \
    \
    inline const CONFIG_TYPE& getConfigConst() { \
        return GlobalConfigManager::getInstance().getConfig(); \
    } \
    \
    inline const CONFIG_TYPE* getConfigPtr() { \
        return config_; \
    } \
    \
    inline const CONFIG_TYPE& cfg() { \
        return *config_; \
    } \
} /* namespace NAMESPACE_NAME */
