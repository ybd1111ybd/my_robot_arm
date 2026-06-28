#pragma once

#include "universal_config/universal_toml_config.h"
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace teleop_vr {

struct Config : public universal_config::UniversalTomlConfig {
  // 日志配置
  struct {
    std::string level = "info";
    bool enable_debug_output = false;
    std::string dir = "./log";
    int max_size_mb = 100;
  } logging;

  // UDP配置
  struct {
    std::string host = "0.0.0.0";
    int port = 8080;
    bool enable_udp_receive = false;  // 默认关闭，需要服务启用
  } udp;

  // 发布模式配置
  struct {
    // "joint": 只发布关节/夹爪命令
    // "cartesian": 只发布VR位姿、手柄输入和末端target_pose
    // "both": 同时发布关节命令和末端target_pose
    std::string mode = "both";
    bool publish_debug_raw = true;
  } publish;

  // Topic配置
  struct {
    std::string left_arm_topic = "telecon/arm_left/joint_commands_input";
    std::string right_arm_topic = "telecon/arm_right/joint_commands_input";
    std::string left_gripper_topic = "left_gripper/gripper_commands";
    std::string right_gripper_topic = "right_gripper/gripper_commands";

    // VR设备位姿topic
    std::string left_controller_pose_topic = "vr/left_controller/pose";
    std::string right_controller_pose_topic = "vr/right_controller/pose";
    std::string headset_pose_topic = "vr/headset/pose";

    // 手柄输入topic
    std::string left_controller_input_topic = "vr/left_controller/input";
    std::string right_controller_input_topic = "vr/right_controller/input";

    // Debug topic
    std::string udp_raw_data_topic = "udp_raw_data";

    // 笛卡尔目标位姿topic（默认对齐 robot_kinematics）
    std::string left_arm_target_pose_topic = "/arm_left/target_pose";
    std::string right_arm_target_pose_topic = "/arm_right/target_pose";
  } topics;

  // 夹爪配置
  struct {
    double default_force = 80.0;  // 默认夹爪力度（百分比）
  } gripper;

  // FK订阅配置
  struct {
    std::string left_arm_fk_pose_topic = "/arm_left/fk_pose";
    std::string right_arm_fk_pose_topic = "/arm_right/fk_pose";
  } fk;

  // A/X键回到VR初始姿态的固定末端target配置
  struct {
    bool enabled = true;
    // [x, y, z, qx, qy, qz, qw]，坐标系为base_link
    std::array<double, 7> left_pose = {
        0.7324048656654268, 0.3731828053351678, 1.0581095836473335,
        -0.04307959999168663, 0.06159595547961809,
        -0.6862686308289023, 0.7234538358964814};
    std::array<double, 7> right_pose = {
        0.7321293328673628, -0.3734108305057274, 1.0580827168268605,
        0.04322027383914863, 0.06156938877171767,
        0.6858984346948285, 0.7237986982433239};
  } home_target;

  // 数据平滑滤波配置
  struct {
    bool enabled = true;          // 是否启用平滑滤波
    std::string smoother_type = "exp_moving_avg";  // 滤波器类型: "none", "moving_avg", "exp_moving_avg", "kalman"
    double position_alpha = 0.3;   // 位置数据平滑系数 (0.0-1.0, 越小越平滑)
    double rotation_alpha = 0.5;   // 旋转数据平滑系数 (0.0-1.0, 四元数需要更快的响应)
    double input_alpha = 0.2;      // 输入数据平滑系数 (0.0-1.0, 按钮/摇杆)
  } smoother;

  // VR坐标转换配置
  struct {
    // 全局比例因子（对双臂同时生效）
    double scale_factor = 0.8;

    // 全局工作空间边界限制配置（对双臂同时生效）
    struct {
      bool enable_workspace_limit = false;  // 是否启用边界限制
      double max_workspace_radius = 0.8;    // 最大工作半径（米）
      std::string boundary_type = "clamp";  // 边界类型: "clamp"(硬限制) 或 "saturate"(平滑限制)
    } workspace_limit;

    // 左臂配置
    struct {
      std::array<int, 3> axis_mapping = {-2, 3, -1};  // 坐标轴映射（VR基坐标→base_link）

      // 工作空间球心配置（与目标位姿同坐标系，推荐 base_link）
      std::array<double, 3> workspace_center = {0.0, 0.0, 0.0};
      bool workspace_center_use_topic = false;
      std::string workspace_center_topic = "/arm_left/workspace_center";
    } left_arm;

    // 右臂配置
    struct {
      std::array<int, 3> axis_mapping = {-2, 3, -1};  // 坐标轴映射（VR基坐标→base_link）

      // 工作空间球心配置（与目标位姿同坐标系，推荐 base_link）
      std::array<double, 3> workspace_center = {0.0, 0.0, 0.0};
      bool workspace_center_use_topic = false;
      std::string workspace_center_topic = "/arm_right/workspace_center";
    } right_arm;
  } vr_transform;

  // 实现纯虚函数
  virtual bool loadFromFile(const std::string& config_file) override {
    try {
      if (!std::filesystem::exists(config_file)) {
        std::cerr << "Configuration file does not exist: " << config_file << std::endl;
        return false;
      }

      auto data = toml::parse(config_file);

      // 加载日志配置
      logging.level = getTomlValue(data, "log", "log_level", logging.level);
      logging.enable_debug_output = getTomlValue(
          data, "log", "enable_debug_output", logging.enable_debug_output);
      logging.dir = getTomlValue(data, "log", "log_dir", logging.dir);
      logging.max_size_mb =
          getTomlValue(data, "log", "max_log_size_mb", logging.max_size_mb);

      // 加载UDP配置
      udp.host = getTomlValue(data, "udp", "host", udp.host);
      udp.port = getTomlValue(data, "udp", "port", udp.port);
      udp.enable_udp_receive = getTomlValue(
          data, "udp", "enable_udp_receive", udp.enable_udp_receive);

      // 加载发布模式配置
      publish.mode = getTomlValue(data, "publish", "mode", publish.mode);
      publish.publish_debug_raw = getTomlValue(
          data, "publish", "publish_debug_raw", publish.publish_debug_raw);

      // 加载Topic配置
      topics.left_arm_topic = getTomlValue(
          data, "topics", "left_arm_topic", topics.left_arm_topic);
      topics.right_arm_topic = getTomlValue(
          data, "topics", "right_arm_topic", topics.right_arm_topic);
      topics.left_gripper_topic = getTomlValue(
          data, "topics", "left_gripper_topic", topics.left_gripper_topic);
      topics.right_gripper_topic = getTomlValue(
          data, "topics", "right_gripper_topic", topics.right_gripper_topic);

      topics.left_controller_pose_topic = getTomlValue(
          data, "topics", "left_controller_pose_topic", topics.left_controller_pose_topic);
      topics.right_controller_pose_topic = getTomlValue(
          data, "topics", "right_controller_pose_topic", topics.right_controller_pose_topic);
      topics.headset_pose_topic = getTomlValue(
          data, "topics", "headset_pose_topic", topics.headset_pose_topic);

      topics.left_controller_input_topic = getTomlValue(
          data, "topics", "left_controller_input_topic", topics.left_controller_input_topic);
      topics.right_controller_input_topic = getTomlValue(
          data, "topics", "right_controller_input_topic", topics.right_controller_input_topic);

      topics.udp_raw_data_topic = getTomlValue(
          data, "topics", "udp_raw_data_topic", topics.udp_raw_data_topic);

      topics.left_arm_target_pose_topic = getTomlValue(
          data, "topics", "left_arm_target_pose_topic", topics.left_arm_target_pose_topic);
      topics.right_arm_target_pose_topic = getTomlValue(
          data, "topics", "right_arm_target_pose_topic", topics.right_arm_target_pose_topic);

      // 加载FK订阅配置
      fk.left_arm_fk_pose_topic = getTomlValue(
          data, "fk", "left_arm_fk_pose_topic", fk.left_arm_fk_pose_topic);
      fk.right_arm_fk_pose_topic = getTomlValue(
          data, "fk", "right_arm_fk_pose_topic", fk.right_arm_fk_pose_topic);

      // 加载固定VR初始末端target配置
      home_target.enabled = getTomlValue(
          data, "home_target", "enabled", home_target.enabled);
      if (data.contains("home_target")) {
        const auto& home = toml::find(data, "home_target");
        const auto parse_pose = [](const toml::value& value,
                                   std::array<double, 7>& pose) {
          if (!value.is_array()) {
            return;
          }

          const auto& array = value.as_array();
          if (array.size() < pose.size()) {
            return;
          }

          for (size_t i = 0; i < pose.size(); ++i) {
            if (array[i].is_floating()) {
              pose[i] = array[i].as_floating();
            } else if (array[i].is_integer()) {
              pose[i] = static_cast<double>(array[i].as_integer());
            }
          }
        };

        if (home.contains("left_pose")) {
          parse_pose(toml::find(home, "left_pose"), home_target.left_pose);
        }
        if (home.contains("right_pose")) {
          parse_pose(toml::find(home, "right_pose"), home_target.right_pose);
        }
      }

      // 加载夹爪配置
      gripper.default_force = getTomlValue(
          data, "gripper", "default_force", gripper.default_force);

      // 加载平滑滤波配置
      smoother.enabled = getTomlValue(data, "smoother", "enabled", smoother.enabled);
      smoother.smoother_type = getTomlValue(data, "smoother", "smoother_type", smoother.smoother_type);
      smoother.position_alpha = getTomlValue(data, "smoother", "position_alpha", smoother.position_alpha);
      smoother.rotation_alpha = getTomlValue(data, "smoother", "rotation_alpha", smoother.rotation_alpha);
      smoother.input_alpha = getTomlValue(data, "smoother", "input_alpha", smoother.input_alpha);

      // 加载VR坐标转换配置
      if (data.contains("vr_transform")) {
        const auto& vr = toml::find(data, "vr_transform");

        // 全局工作空间限制（双臂共用）
        if (vr.contains("scale_factor")) {
          vr_transform.scale_factor = toml::find_or(vr, "scale_factor",
            vr_transform.scale_factor);
        }

        if (vr.contains("enable_workspace_limit")) {
          vr_transform.workspace_limit.enable_workspace_limit = toml::find_or(vr, "enable_workspace_limit",
            vr_transform.workspace_limit.enable_workspace_limit);
        }
        if (vr.contains("max_workspace_radius")) {
          vr_transform.workspace_limit.max_workspace_radius = toml::find_or(vr, "max_workspace_radius",
            vr_transform.workspace_limit.max_workspace_radius);
        }
        if (vr.contains("boundary_type")) {
          vr_transform.workspace_limit.boundary_type = toml::find_or(vr, "boundary_type",
            vr_transform.workspace_limit.boundary_type);
        }

        const auto parse_arm = [](const toml::value& arm_value, auto& arm_cfg) {
          if (arm_value.contains("axis_mapping") && toml::find(arm_value, "axis_mapping").is_array()) {
            const auto& mapping = toml::find(arm_value, "axis_mapping").as_array();
            if (mapping.size() >= 3) {
              arm_cfg.axis_mapping[0] = mapping[0].as_integer();
              arm_cfg.axis_mapping[1] = mapping[1].as_integer();
              arm_cfg.axis_mapping[2] = mapping[2].as_integer();
            }
          }

          if (arm_value.contains("workspace_center") && toml::find(arm_value, "workspace_center").is_array()) {
            const auto& center = toml::find(arm_value, "workspace_center").as_array();
            if (center.size() >= 3) {
              arm_cfg.workspace_center[0] = center[0].as_floating();
              arm_cfg.workspace_center[1] = center[1].as_floating();
              arm_cfg.workspace_center[2] = center[2].as_floating();
            }
          }

          if (arm_value.contains("workspace_center_use_topic")) {
            arm_cfg.workspace_center_use_topic = toml::find_or(arm_value,
              "workspace_center_use_topic", arm_cfg.workspace_center_use_topic);
          }

          if (arm_value.contains("workspace_center_topic")) {
            arm_cfg.workspace_center_topic = toml::find_or(arm_value,
              "workspace_center_topic", arm_cfg.workspace_center_topic);
          }
        };

        if (vr.contains("left_arm")) {
          parse_arm(toml::find(vr, "left_arm"), vr_transform.left_arm);
        }

        if (vr.contains("right_arm")) {
          parse_arm(toml::find(vr, "right_arm"), vr_transform.right_arm);
        }
      }

      return true;
    } catch (const std::exception& e) {
      std::cerr << "Failed to load TOML configuration: " << e.what() << std::endl;
      return false;
    }
  }

  virtual bool saveToFile(const std::string& config_file) const override {
    // 暂不实现保存功能
    (void)config_file;  // 抑制未使用参数警告
    return true;
  }

  virtual bool validate() const override {
    // 验证端口范围
    if (udp.port < 1024 || udp.port > 65535) {
      std::cerr << "Invalid UDP port: " << udp.port << std::endl;
      return false;
    }

    if (publish.mode != "joint" && publish.mode != "cartesian" &&
        publish.mode != "both") {
      std::cerr << "Invalid publish.mode: " << publish.mode
                << " (supported: joint, cartesian, both)" << std::endl;
      return false;
    }

    // 验证日志级别
    if (logging.level != "info" && logging.level != "warning" &&
        logging.level != "error" && logging.level != "fatal") {
      std::cerr << "Invalid log level: " << logging.level << std::endl;
      return false;
    }

    // 验证平滑参数
    if (smoother.position_alpha < 0.0 || smoother.position_alpha > 1.0 ||
        smoother.rotation_alpha < 0.0 || smoother.rotation_alpha > 1.0 ||
        smoother.input_alpha < 0.0 || smoother.input_alpha > 1.0) {
      std::cerr << "Invalid smoother alpha (must be in [0,1])" << std::endl;
      return false;
    }

    if (smoother.smoother_type != "none" &&
        smoother.smoother_type != "moving_avg" &&
        smoother.smoother_type != "exp_moving_avg" &&
        smoother.smoother_type != "kalman") {
      std::cerr << "Invalid smoother_type: " << smoother.smoother_type << std::endl;
      return false;
    }

    if (vr_transform.workspace_limit.enable_workspace_limit &&
        vr_transform.workspace_limit.max_workspace_radius <= 0.0) {
      std::cerr << "Invalid max_workspace_radius in vr_transform" << std::endl;
      return false;
    }

    if (vr_transform.workspace_limit.boundary_type != "clamp" &&
        vr_transform.workspace_limit.boundary_type != "saturate") {
      std::cerr << "Invalid boundary_type in vr_transform" << std::endl;
      return false;
    }

    if (vr_transform.scale_factor <= 0.0) {
      std::cerr << "Invalid scale_factor in vr_transform" << std::endl;
      return false;
    }

    const auto validate_arm = [](const auto& arm, const char* name) {
      for (double v : arm.workspace_center) {
        if (!std::isfinite(v)) {
          std::cerr << "Invalid workspace_center (NaN/Inf) for " << name << std::endl;
          return false;
        }
      }

      if (arm.workspace_center_use_topic && arm.workspace_center_topic.empty()) {
        std::cerr << "workspace_center_use_topic=true but workspace_center_topic is empty for "
                  << name << std::endl;
        return false;
      }

      bool used_axis[3] = {false, false, false};
      for (int axis : arm.axis_mapping) {
        int abs_axis = std::abs(axis);
        if (abs_axis < 1 || abs_axis > 3) {
          std::cerr << "Invalid axis_mapping value for " << name << std::endl;
          return false;
        }
        if (used_axis[abs_axis - 1]) {
          std::cerr << "Duplicate axis_mapping value for " << name << std::endl;
          return false;
        }
        used_axis[abs_axis - 1] = true;
      }
      return true;
    };

    if (!validate_arm(vr_transform.left_arm, "left_arm") ||
        !validate_arm(vr_transform.right_arm, "right_arm")) {
      return false;
    }

    return true;
  }
};

} // namespace teleop_vr
