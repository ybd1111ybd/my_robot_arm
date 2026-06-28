#pragma once

#include "teleop_vr_recv/config.h"
#include "universal_config/toml_config.h"

// 定义teleop_vr命名空间的全局配置管理器
DEFINE_GLOBAL_CONFIG_MANAGER(teleop_vr, teleop_vr::Config);
