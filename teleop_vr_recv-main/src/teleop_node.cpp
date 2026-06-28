#include "teleop_vr_recv/teleop_node.h"

#include <Eigen/Geometry>
#include <cmath>

#include "log/log_config.h"
#include "teleop_vr_recv/teleop_vr_config.h"
#include "teleop_vr_recv/vr_button_handler.h"
#include "teleop_vr_recv/vr_smoothing_pipeline.h"

namespace teleop_vr {

TeleopVrRecvNode::TeleopVrRecvNode(const rclcpp::NodeOptions& options)
    : Node("teleop_vr_recv_node", options),
      port_(8080),
      enable_udp_receive_(false),
      left_fk_received_(false),
      right_fk_received_(false),
      has_last_left_target_pose_(false),
      has_last_right_target_pose_(false),
      last_left_grip_for_mode_(false),
      last_right_grip_for_mode_(false),
      last_left_primary_for_mode_(false),
      last_right_primary_for_mode_(false),
      left_workspace_center_use_topic_(false),
      right_workspace_center_use_topic_(false),
      left_workspace_center_received_(false),
      right_workspace_center_received_(false),
      left_workspace_center_waiting_logged_(false),
      right_workspace_center_waiting_logged_(false) {
  if (!isConfigInitialized()) {
    LOG_WARNING("配置未初始化，使用默认设置");
  }

  initializeParameters();
  initializePublishers();
  initializeSubscribers();
  setupUdpReceiver();

  enable_udp_service_ = this->create_service<std_srvs::srv::SetBool>(
      "enable_udp_receive",
      std::bind(&TeleopVrRecvNode::enableUdpReceiveCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  LOG_INFO("TeleopVrRecvNode initialized successfully");
  LOG_INFO("UDP receive is " << (enable_udp_receive_ ? "ENABLED" : "DISABLED")
                             << " (use service enable_udp_receive to control)");
}

TeleopVrRecvNode::~TeleopVrRecvNode() {
  if (udp_receiver_) {
    udp_receiver_->stop();
  }
  LOG_INFO("TeleopVrRecvNode destroyed");
}

void TeleopVrRecvNode::initializeParameters() {
  if (isConfigInitialized()) {
    const auto& config = getConfigConst();

    host_ = config.udp.host;
    port_ = config.udp.port;
    enable_udp_receive_ = config.udp.enable_udp_receive;

    LOG_INFO("Parameters loaded from configuration:");
    LOG_INFO("  host: " << host_);
    LOG_INFO("  port: " << port_);
    LOG_INFO(
        "  enable_udp_receive: " << (enable_udp_receive_ ? "true" : "false"));
    LOG_INFO("  publish.mode: " << config.publish.mode);
    LOG_INFO("  publish_debug_raw: "
             << (config.publish.publish_debug_raw ? "true" : "false"));

    LOG_INFO("=================================================");
    LOG_INFO("工作空间边界限制配置:");
    LOG_INFO("  全局:");
    LOG_INFO("    enable_workspace_limit: "
             << (config.vr_transform.workspace_limit.enable_workspace_limit
                     ? "true"
                     : "false"));
    LOG_INFO("    max_workspace_radius: "
             << config.vr_transform.workspace_limit.max_workspace_radius
             << " 米");
    LOG_INFO("    boundary_type: "
             << config.vr_transform.workspace_limit.boundary_type);
    LOG_INFO("  左臂球心:");
    LOG_INFO("    workspace_center: ["
             << config.vr_transform.left_arm.workspace_center[0] << ", "
             << config.vr_transform.left_arm.workspace_center[1] << ", "
             << config.vr_transform.left_arm.workspace_center[2] << "]");
    LOG_INFO("    workspace_center_use_topic: "
             << (config.vr_transform.left_arm.workspace_center_use_topic
                     ? "true"
                     : "false"));
    if (config.vr_transform.left_arm.workspace_center_use_topic) {
      LOG_INFO("    workspace_center_topic: "
               << config.vr_transform.left_arm.workspace_center_topic);
    }
    LOG_INFO("  右臂球心:");
    LOG_INFO("    workspace_center: ["
             << config.vr_transform.right_arm.workspace_center[0] << ", "
             << config.vr_transform.right_arm.workspace_center[1] << ", "
             << config.vr_transform.right_arm.workspace_center[2] << "]");
    LOG_INFO("    workspace_center_use_topic: "
             << (config.vr_transform.right_arm.workspace_center_use_topic
                     ? "true"
                     : "false"));
    if (config.vr_transform.right_arm.workspace_center_use_topic) {
      LOG_INFO("    workspace_center_topic: "
               << config.vr_transform.right_arm.workspace_center_topic);
    }
    LOG_INFO("=================================================");
  } else {
    host_ = "0.0.0.0";
    port_ = 8080;
    enable_udp_receive_ = false;

    LOG_WARNING("Using default parameters (configuration not loaded):");
    LOG_WARNING("  host: " << host_);
    LOG_WARNING("  port: " << port_);
    LOG_WARNING("  enable_udp_receive: false");
    LOG_WARNING("  publish.mode: both");
    LOG_WARNING(
        "工作空间边界限制: 使用默认值(enable=false, radius=1.0, type=clamp)");
  }

  initializeVrTransformers();
  initializeDataSmoother();
  button_handler_ = std::make_unique<VrButtonHandler>();
}

void TeleopVrRecvNode::initializeVrTransformers() {
  constexpr double kHalfPi = 1.5707963267948966;
  // 左臂：绕Z轴旋转90度
  const Eigen::Quaterniond left_tool_offset(
      Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()));
  // 右臂：绕Z轴旋转-90度（即270度）
  const Eigen::Quaterniond right_tool_offset(
      Eigen::AngleAxisd(-kHalfPi, Eigen::Vector3d::UnitZ()));

  left_workspace_center_received_ = false;
  right_workspace_center_received_ = false;
  left_workspace_center_waiting_logged_ = false;
  right_workspace_center_waiting_logged_ = false;

  if (isConfigInitialized()) {
    const auto& config = getConfigConst();

    left_arm_transformer_ = std::make_unique<VrCoordinateTransformer>(
        config.vr_transform.scale_factor);
    left_arm_transformer_->setAxisMapping(
        config.vr_transform.left_arm.axis_mapping);
    left_arm_transformer_->setToolOrientationOffset(left_tool_offset);
    left_arm_transformer_->setWorkspaceLimits(
        config.vr_transform.workspace_limit.enable_workspace_limit,
        config.vr_transform.workspace_limit.max_workspace_radius,
        config.vr_transform.workspace_limit.boundary_type);

    left_workspace_center_use_topic_ =
        config.vr_transform.left_arm.workspace_center_use_topic;
    if (!left_workspace_center_use_topic_) {
      left_arm_transformer_->setWorkspaceCenter(
          config.vr_transform.left_arm.workspace_center.data());
      left_workspace_center_received_ = true;
    }

    right_arm_transformer_ = std::make_unique<VrCoordinateTransformer>(
        config.vr_transform.scale_factor);
    right_arm_transformer_->setAxisMapping(
        config.vr_transform.right_arm.axis_mapping);
    right_arm_transformer_->setToolOrientationOffset(right_tool_offset);
    right_arm_transformer_->setWorkspaceLimits(
        config.vr_transform.workspace_limit.enable_workspace_limit,
        config.vr_transform.workspace_limit.max_workspace_radius,
        config.vr_transform.workspace_limit.boundary_type);

    right_workspace_center_use_topic_ =
        config.vr_transform.right_arm.workspace_center_use_topic;
    if (!right_workspace_center_use_topic_) {
      right_arm_transformer_->setWorkspaceCenter(
          config.vr_transform.right_arm.workspace_center.data());
      right_workspace_center_received_ = true;
    }

    LOG_INFO("VR坐标转换器已初始化(边界限制已应用)");
  } else {
    left_arm_transformer_ = std::make_unique<VrCoordinateTransformer>(0.8);
    right_arm_transformer_ = std::make_unique<VrCoordinateTransformer>(0.8);
    left_arm_transformer_->setToolOrientationOffset(left_tool_offset);
    right_arm_transformer_->setToolOrientationOffset(right_tool_offset);
    left_workspace_center_use_topic_ = false;
    right_workspace_center_use_topic_ = false;
    left_workspace_center_received_ = true;
    right_workspace_center_received_ = true;
    LOG_WARNING("VR坐标转换器使用默认配置");
  }
}

void TeleopVrRecvNode::initializePublishers() {
  LOG_INFO("=================================================");

  auto qos = rclcpp::QoS(10).reliable().keep_last(10);

  std::string left_arm_topic = "telecon/arm_left/joint_commands_input";
  std::string right_arm_topic = "telecon/arm_right/joint_commands_input";
  std::string left_gripper_topic = "left_gripper/gripper_commands";
  std::string right_gripper_topic = "right_gripper/gripper_commands";
  std::string left_controller_pose_topic = "vr/left_controller/pose";
  std::string right_controller_pose_topic = "vr/right_controller/pose";
  std::string headset_pose_topic = "vr/headset/pose";
  std::string left_controller_input_topic = "vr/left_controller/input";
  std::string right_controller_input_topic = "vr/right_controller/input";
  std::string udp_raw_data_topic = "udp_raw_data";
  std::string left_arm_target_pose_topic = "/arm_left/target_pose";
  std::string right_arm_target_pose_topic = "/arm_right/target_pose";
  std::string publish_mode = "both";
  bool publish_debug_raw = true;

  if (isConfigInitialized()) {
    const auto& config = getConfigConst();
    left_arm_topic = config.topics.left_arm_topic;
    right_arm_topic = config.topics.right_arm_topic;
    left_gripper_topic = config.topics.left_gripper_topic;
    right_gripper_topic = config.topics.right_gripper_topic;
    left_controller_pose_topic = config.topics.left_controller_pose_topic;
    right_controller_pose_topic = config.topics.right_controller_pose_topic;
    headset_pose_topic = config.topics.headset_pose_topic;
    left_controller_input_topic = config.topics.left_controller_input_topic;
    right_controller_input_topic = config.topics.right_controller_input_topic;
    udp_raw_data_topic = config.topics.udp_raw_data_topic;
    left_arm_target_pose_topic = config.topics.left_arm_target_pose_topic;
    right_arm_target_pose_topic = config.topics.right_arm_target_pose_topic;
    publish_mode = config.publish.mode;
    publish_debug_raw = config.publish.publish_debug_raw;
  }

  if (publish_mode == "joint" || publish_mode == "both") {
    left_arm_smooth_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>(left_arm_topic, qos);
    right_arm_smooth_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        right_arm_topic, qos);
  }

  left_gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      left_gripper_topic, qos);
  right_gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      right_gripper_topic, qos);

  left_controller_pose_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          left_controller_pose_topic, qos);
  right_controller_pose_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          right_controller_pose_topic, qos);
  headset_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      headset_pose_topic, qos);

  left_controller_input_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          left_controller_input_topic, qos);
  right_controller_input_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          right_controller_input_topic, qos);

  left_arm_target_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          left_arm_target_pose_topic, qos);
  right_arm_target_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          right_arm_target_pose_topic, qos);

  udp_raw_data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      udp_raw_data_topic, qos);

  LOG_INFO("Publish Mode: SMOOTH (JointState)");
  LOG_INFO("Configured publish.mode: " << publish_mode);
  LOG_INFO("Configured publish_debug_raw: "
           << (publish_debug_raw ? "true" : "false"));
  LOG_INFO("Topics (relative paths, support namespace):");
  if (publish_mode == "joint" || publish_mode == "both") {
    LOG_INFO("  Arm Commands:");
    LOG_INFO("    - " << left_arm_topic);
    LOG_INFO("    - " << right_arm_topic);
  } else {
    LOG_INFO("  Arm Commands: disabled by publish.mode=" << publish_mode);
  }
  LOG_INFO("  Gripper Commands:");
  LOG_INFO("    - " << left_gripper_topic);
  LOG_INFO("    - " << right_gripper_topic);
  LOG_INFO("  VR Device Poses:");
  LOG_INFO("    - " << left_controller_pose_topic);
  LOG_INFO("    - " << right_controller_pose_topic);
  LOG_INFO("    - " << headset_pose_topic);
  LOG_INFO("  Controller Inputs:");
  LOG_INFO("    - " << left_controller_input_topic);
  LOG_INFO("    - " << right_controller_input_topic);
  LOG_INFO("  Cartesian Target Poses (PoseStamped, with workspace limits):");
  LOG_INFO("    - " << left_arm_target_pose_topic);
  LOG_INFO("    - " << right_arm_target_pose_topic);
  LOG_INFO("  Debug:");
  LOG_INFO("    - " << udp_raw_data_topic);
  LOG_INFO("=================================================");
}

void TeleopVrRecvNode::initializeSubscribers() {
  std::string left_fk_topic = "/arm_left/fk_pose";
  std::string right_fk_topic = "/arm_right/fk_pose";
  std::string left_center_topic = "/arm_left/workspace_center";
  std::string right_center_topic = "/arm_right/workspace_center";

  if (isConfigInitialized()) {
    const auto& config = getConfigConst();
    left_fk_topic = config.fk.left_arm_fk_pose_topic;
    right_fk_topic = config.fk.right_arm_fk_pose_topic;
    left_workspace_center_use_topic_ =
        config.vr_transform.left_arm.workspace_center_use_topic;
    right_workspace_center_use_topic_ =
        config.vr_transform.right_arm.workspace_center_use_topic;
    left_center_topic = config.vr_transform.left_arm.workspace_center_topic;
    right_center_topic = config.vr_transform.right_arm.workspace_center_topic;
  }

  auto qos = rclcpp::QoS(10).reliable().keep_last(10);

  left_fk_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      left_fk_topic, qos,
      std::bind(&TeleopVrRecvNode::leftFkPoseCallback, this,
                std::placeholders::_1));
  right_fk_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          right_fk_topic, qos,
          std::bind(&TeleopVrRecvNode::rightFkPoseCallback, this,
                    std::placeholders::_1));

  LOG_INFO("FK subscribers initialized:");
  LOG_INFO("  - " << left_fk_topic);
  LOG_INFO("  - " << right_fk_topic);

  if (left_workspace_center_use_topic_) {
    left_workspace_center_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            left_center_topic, qos,
            std::bind(&TeleopVrRecvNode::leftWorkspaceCenterCallback, this,
                      std::placeholders::_1));
    LOG_INFO("Workspace center subscriber initialized (left): "
             << left_center_topic);
  }

  if (right_workspace_center_use_topic_) {
    right_workspace_center_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            right_center_topic, qos,
            std::bind(&TeleopVrRecvNode::rightWorkspaceCenterCallback, this,
                      std::placeholders::_1));
    LOG_INFO("Workspace center subscriber initialized (right): "
             << right_center_topic);
  }
}

void TeleopVrRecvNode::setupUdpReceiver() {
  udp_receiver_ = std::make_unique<UdpReceiver>(host_, port_);

  udp_receiver_->setDataCallback(
      [this](const std::vector<uint8_t>& data, size_t size) {
        this->onUdpDataReceived(data, size);
      });

  if (!udp_receiver_->start()) {
    LOG_ERROR("Failed to start UDP receiver");
    throw std::runtime_error("Failed to start UDP receiver");
  }

  LOG_INFO("Waiting for VR data...");
}

void TeleopVrRecvNode::onUdpDataReceived(const std::vector<uint8_t>& data,
                                         size_t size) {
  if (!enable_udp_receive_) {
    return;
  }

  auto packet_opt = VrDataParser::parse(data, size);
  if (!packet_opt.has_value()) {
    return;
  }

  const auto& packet = packet_opt.value();

  std::string publish_mode = "both";
  bool publish_debug_raw = true;
  if (isConfigInitialized()) {
    const auto& config = getConfigConst();
    publish_mode = config.publish.mode;
    publish_debug_raw = config.publish.publish_debug_raw;
  }

  if (publish_debug_raw) {
    auto udp_raw_msg = std_msgs::msg::Float64MultiArray();
    udp_raw_msg.data.resize(17);
    udp_raw_msg.data[0] = packet.torque;
    for (size_t i = 0; i < 16; ++i) {
      udp_raw_msg.data[i + 1] = packet.angles[i];
    }
    udp_raw_data_pub_->publish(udp_raw_msg);
  }

  publishGripperCommands(packet);

  if (publish_mode == "joint" || publish_mode == "both") {
    publishArmJointCommands(packet);
  }

  if (publish_mode != "cartesian" && publish_mode != "both") {
    return;
  }

  if (!packet.has_vr_device_poses || !packet.has_controller_inputs) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "publish.mode requires extended VR pose/controller input data; "
        "skipping Cartesian target publishing for this packet");
    return;
  }

  const auto smoothed_poses = smoothing_pipeline_->smoothPoses(packet);

  publishVrDevicePoses(packet, smoothed_poses.left_position,
                       smoothed_poses.left_rotation,
                       smoothed_poses.right_position,
                       smoothed_poses.right_rotation,
                       smoothed_poses.headset_position,
                       smoothed_poses.headset_rotation);
  publishControllerInputs(packet, smoothed_poses.left_position,
                          smoothed_poses.left_rotation,
                          smoothed_poses.right_position,
                          smoothed_poses.right_rotation);
  publishCartesianTargetPose(packet, smoothed_poses.left_position,
                             smoothed_poses.left_rotation,
                             smoothed_poses.right_position,
                             smoothed_poses.right_rotation);
}

void TeleopVrRecvNode::publishArmJointCommands(const VrDataPacket& packet) {
  if (!left_arm_smooth_pub_ || !right_arm_smooth_pub_) {
    return;
  }

  const auto left_angles_deg = packet.getLeftArmAngles();
  const auto right_angles_deg = packet.getRightArmAngles();

  auto left_angles_rad = degreesToRadians(left_angles_deg);
  auto right_angles_rad = degreesToRadians(right_angles_deg);

  auto left_msg = sensor_msgs::msg::JointState();
  left_msg.header.stamp = this->get_clock()->now();
  left_msg.name = {"left_joint1", "left_joint2", "left_joint3",
                   "left_joint4", "left_joint5", "left_joint6",
                   "left_joint7"};
  left_msg.position = left_angles_rad;
  left_arm_smooth_pub_->publish(left_msg);

  auto right_msg = sensor_msgs::msg::JointState();
  right_msg.header.stamp = this->get_clock()->now();
  right_msg.name = {"right_joint1", "right_joint2", "right_joint3",
                    "right_joint4", "right_joint5", "right_joint6",
                    "right_joint7"};
  right_msg.position = right_angles_rad;
  right_arm_smooth_pub_->publish(right_msg);
}

void TeleopVrRecvNode::publishGripperCommands(const VrDataPacket& packet) {
  double gripper_force = 80.0;
  if (isConfigInitialized()) {
    gripper_force = getConfigConst().gripper.default_force;
  }

  auto left_gripper_msg = std_msgs::msg::Float64MultiArray();
  left_gripper_msg.data = {packet.getLeftGripper(), gripper_force};
  left_gripper_pub_->publish(left_gripper_msg);

  auto right_gripper_msg = std_msgs::msg::Float64MultiArray();
  right_gripper_msg.data = {packet.getRightGripper(), gripper_force};
  right_gripper_pub_->publish(right_gripper_msg);
}

void TeleopVrRecvNode::publishVrDevicePoses(
    const VrDataPacket& packet, const std::array<float, 3>& left_smoothed_pos,
    const std::array<float, 4>& left_smoothed_rot,
    const std::array<float, 3>& right_smoothed_pos,
    const std::array<float, 4>& right_smoothed_rot,
    const std::array<float, 3>& headset_smoothed_pos,
    const std::array<float, 4>& headset_smoothed_rot) {
  (void)packet;

  auto left_pose_msg = std_msgs::msg::Float64MultiArray();
  left_pose_msg.data.resize(7);
  left_pose_msg.data[0] = left_smoothed_pos[0];
  left_pose_msg.data[1] = left_smoothed_pos[1];
  left_pose_msg.data[2] = left_smoothed_pos[2];
  left_pose_msg.data[3] = left_smoothed_rot[0];
  left_pose_msg.data[4] = left_smoothed_rot[1];
  left_pose_msg.data[5] = left_smoothed_rot[2];
  left_pose_msg.data[6] = left_smoothed_rot[3];
  left_controller_pose_pub_->publish(left_pose_msg);

  auto right_pose_msg = std_msgs::msg::Float64MultiArray();
  right_pose_msg.data.resize(7);
  right_pose_msg.data[0] = right_smoothed_pos[0];
  right_pose_msg.data[1] = right_smoothed_pos[1];
  right_pose_msg.data[2] = right_smoothed_pos[2];
  right_pose_msg.data[3] = right_smoothed_rot[0];
  right_pose_msg.data[4] = right_smoothed_rot[1];
  right_pose_msg.data[5] = right_smoothed_rot[2];
  right_pose_msg.data[6] = right_smoothed_rot[3];
  right_controller_pose_pub_->publish(right_pose_msg);

  auto headset_pose_msg = std_msgs::msg::Float64MultiArray();
  headset_pose_msg.data.resize(7);
  headset_pose_msg.data[0] = headset_smoothed_pos[0];
  headset_pose_msg.data[1] = headset_smoothed_pos[1];
  headset_pose_msg.data[2] = headset_smoothed_pos[2];
  headset_pose_msg.data[3] = headset_smoothed_rot[0];
  headset_pose_msg.data[4] = headset_smoothed_rot[1];
  headset_pose_msg.data[5] = headset_smoothed_rot[2];
  headset_pose_msg.data[6] = headset_smoothed_rot[3];
  headset_pose_pub_->publish(headset_pose_msg);
}

void TeleopVrRecvNode::publishControllerInputs(
    const VrDataPacket& packet, const std::array<float, 3>& left_smoothed_pos,
    const std::array<float, 4>& left_smoothed_rot,
    const std::array<float, 3>& right_smoothed_pos,
    const std::array<float, 4>& right_smoothed_rot) {
  handleTeleopModeButtons(packet);

  const auto smoothed_inputs = smoothing_pipeline_->smoothInputs(packet);

  auto left_input_msg = std_msgs::msg::Float64MultiArray();
  left_input_msg.data.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    left_input_msg.data[i] = smoothed_inputs.left_input[i];
  }
  left_controller_input_pub_->publish(left_input_msg);

  auto right_input_msg = std_msgs::msg::Float64MultiArray();
  right_input_msg.data.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    right_input_msg.data[i] = smoothed_inputs.right_input[i];
  }
  right_controller_input_pub_->publish(right_input_msg);

  if (!button_handler_) {
    return;
  }

  const ArmButtonContext left_context{packet.left_input,
                                      left_smoothed_pos,
                                      left_smoothed_rot,
                                      left_fk_received_,
                                      left_fk_pose_,
                                      *left_arm_transformer_};
  const ArmButtonContext right_context{packet.right_input,
                                       right_smoothed_pos,
                                       right_smoothed_rot,
                                       right_fk_received_,
                                       right_fk_pose_,
                                       *right_arm_transformer_};

  button_handler_->process(
      left_context, right_context,
      [](const std::string& message) { LOG_INFO(message); },
      [](const std::string& message) { LOG_WARNING(message); });
}

void TeleopVrRecvNode::handleTeleopModeButtons(const VrDataPacket& packet) {
  const bool primary_pressed =
      (packet.left_input.primary_button && !last_left_primary_for_mode_) ||
      (packet.right_input.primary_button && !last_right_primary_for_mode_);

  if (primary_pressed) {
    has_last_left_target_pose_ = false;
    has_last_right_target_pose_ = false;
    publishConfiguredHomeTargets();
  }

  last_left_grip_for_mode_ = packet.left_input.grip_button;
  last_right_grip_for_mode_ = packet.right_input.grip_button;
  last_left_primary_for_mode_ = packet.left_input.primary_button;
  last_right_primary_for_mode_ = packet.right_input.primary_button;
}

void TeleopVrRecvNode::publishConfiguredHomeTargets() {
  if (!isConfigInitialized() || !getConfigConst().home_target.enabled) {
    LOG_WARNING("A/X键按下但固定VR初始target未启用，未发布回位target_pose");
    return;
  }

  const auto& home_target = getConfigConst().home_target;

  auto make_pose = [this](const std::array<double, 7>& target) {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.pose.position.x = target[0];
    msg.pose.position.y = target[1];
    msg.pose.position.z = target[2];
    msg.pose.orientation.x = target[3];
    msg.pose.orientation.y = target[4];
    msg.pose.orientation.z = target[5];
    msg.pose.orientation.w = target[6];
    return msg;
  };

  if (left_arm_target_pose_pub_) {
    last_left_target_pose_ = make_pose(home_target.left_pose);
    has_last_left_target_pose_ = true;
    left_arm_target_pose_pub_->publish(last_left_target_pose_);
  } else {
    LOG_WARNING("左手X键按下但左臂target_pose发布器未初始化");
  }

  if (right_arm_target_pose_pub_) {
    last_right_target_pose_ = make_pose(home_target.right_pose);
    has_last_right_target_pose_ = true;
    right_arm_target_pose_pub_->publish(last_right_target_pose_);
  } else {
    LOG_WARNING("右手A键按下但右臂target_pose发布器未初始化");
  }

  LOG_INFO("Primary button pressed - published configured VR initial target_pose");
}

void TeleopVrRecvNode::publishCartesianTargetPose(
    const VrDataPacket& packet, const std::array<float, 3>& left_smoothed_pos,
    const std::array<float, 4>& left_smoothed_rot,
    const std::array<float, 3>& right_smoothed_pos,
    const std::array<float, 4>& right_smoothed_rot) {
  (void)packet;

  if (left_arm_transformer_->isReferenceLocked()) {
    if (left_workspace_center_use_topic_ && !left_workspace_center_received_) {
      if (!left_workspace_center_waiting_logged_) {
        LOG_WARNING("左臂未收到工作空间球心topic，暂停发布target_pose");
        left_workspace_center_waiting_logged_ = true;
      }
    } else {
      auto left_target = left_arm_transformer_->transform(
          left_smoothed_pos.data(), left_smoothed_rot.data());
      auto left_msg = geometry_msgs::msg::PoseStamped();
      left_msg.header.stamp = this->get_clock()->now();
      left_msg.header.frame_id = "base_link";
      left_msg.pose.position.x = left_target.position[0];
      left_msg.pose.position.y = left_target.position[1];
      left_msg.pose.position.z = left_target.position[2];
      left_msg.pose.orientation.x = left_target.orientation[0];
      left_msg.pose.orientation.y = left_target.orientation[1];
      left_msg.pose.orientation.z = left_target.orientation[2];
      left_msg.pose.orientation.w = left_target.orientation[3];
      last_left_target_pose_ = left_msg;
      has_last_left_target_pose_ = true;
      left_arm_target_pose_pub_->publish(left_msg);
    }
  } else if (!left_workspace_center_use_topic_ ||
             left_workspace_center_received_) {
    publishHoldTargetPose(left_arm_target_pose_pub_, last_left_target_pose_,
                          has_last_left_target_pose_);
  }

  if (right_arm_transformer_->isReferenceLocked()) {
    if (right_workspace_center_use_topic_ && !right_workspace_center_received_) {
      if (!right_workspace_center_waiting_logged_) {
        LOG_WARNING("右臂未收到工作空间球心topic，暂停发布target_pose");
        right_workspace_center_waiting_logged_ = true;
      }
    } else {
      auto right_target = right_arm_transformer_->transform(
          right_smoothed_pos.data(), right_smoothed_rot.data());
      auto right_msg = geometry_msgs::msg::PoseStamped();
      right_msg.header.stamp = this->get_clock()->now();
      right_msg.header.frame_id = "base_link";
      right_msg.pose.position.x = right_target.position[0];
      right_msg.pose.position.y = right_target.position[1];
      right_msg.pose.position.z = right_target.position[2];
      right_msg.pose.orientation.x = right_target.orientation[0];
      right_msg.pose.orientation.y = right_target.orientation[1];
      right_msg.pose.orientation.z = right_target.orientation[2];
      right_msg.pose.orientation.w = right_target.orientation[3];
      last_right_target_pose_ = right_msg;
      has_last_right_target_pose_ = true;
      right_arm_target_pose_pub_->publish(right_msg);
    }
  } else if (!right_workspace_center_use_topic_ ||
             right_workspace_center_received_) {
    publishHoldTargetPose(right_arm_target_pose_pub_, last_right_target_pose_,
                          has_last_right_target_pose_);
  }
}

geometry_msgs::msg::PoseStamped TeleopVrRecvNode::createTargetPoseMsg(
    const RobotEndEffectorPose& pose) {
  auto msg = geometry_msgs::msg::PoseStamped();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_link";
  msg.pose.position.x = pose.position[0];
  msg.pose.position.y = pose.position[1];
  msg.pose.position.z = pose.position[2];
  msg.pose.orientation.x = pose.orientation[0];
  msg.pose.orientation.y = pose.orientation[1];
  msg.pose.orientation.z = pose.orientation[2];
  msg.pose.orientation.w = pose.orientation[3];
  return msg;
}

void TeleopVrRecvNode::publishHoldTargetPose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
    geometry_msgs::msg::PoseStamped& last_target_pose,
    bool& has_last_target_pose) {
  if (!pub || !has_last_target_pose) {
    return;
  }

  last_target_pose.header.stamp = this->get_clock()->now();
  pub->publish(last_target_pose);
}

std::vector<double> TeleopVrRecvNode::degreesToRadians(
    const std::array<float, 7>& degrees) {
  std::vector<double> radians(7);
  for (size_t i = 0; i < 7; ++i) {
    radians[i] = degrees[i] * M_PI / 180.0;
  }
  return radians;
}

void TeleopVrRecvNode::initializeDataSmoother() {
  if (isConfigInitialized()) {
    const auto& config = getConfigConst();

    SmootherType smoother_type = SmootherType::EXP_MOVING_AVG;
    if (config.smoother.smoother_type == "none") {
      smoother_type = SmootherType::NONE;
    } else if (config.smoother.smoother_type == "moving_avg") {
      smoother_type = SmootherType::MOVING_AVG;
    } else if (config.smoother.smoother_type == "exp_moving_avg") {
      smoother_type = SmootherType::EXP_MOVING_AVG;
    } else if (config.smoother.smoother_type == "kalman") {
      smoother_type = SmootherType::KALMAN;
    }

    data_smoother_ = std::make_unique<VrDataSmoother>(
        config.smoother.enabled, smoother_type,
        static_cast<float>(config.smoother.position_alpha),
        static_cast<float>(config.smoother.rotation_alpha),
        static_cast<float>(config.smoother.input_alpha));
    smoothing_pipeline_ =
        std::make_unique<VrSmoothingPipeline>(*data_smoother_);

    LOG_INFO("=================================================");
    LOG_INFO("数据平滑滤波配置:");
    LOG_INFO("  启用状态: " << (config.smoother.enabled ? "是" : "否"));
    LOG_INFO("  滤波器类型: " << config.smoother.smoother_type);
    LOG_INFO("  位置平滑系数: " << config.smoother.position_alpha);
    LOG_INFO("  旋转平滑系数: " << config.smoother.rotation_alpha);
    LOG_INFO("  输入平滑系数: " << config.smoother.input_alpha);
    LOG_INFO("=================================================");
  } else {
    data_smoother_ = std::make_unique<VrDataSmoother>(
        true, SmootherType::EXP_MOVING_AVG, 0.3f, 0.5f, 0.2f);
    smoothing_pipeline_ =
        std::make_unique<VrSmoothingPipeline>(*data_smoother_);
    LOG_WARNING("数据平滑滤波器使用默认配置");
  }
}

void TeleopVrRecvNode::run() {
  udp_receiver_->receiveOnce(100);
  rclcpp::spin_some(this->get_node_base_interface());
}

void TeleopVrRecvNode::enableUdpReceiveCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  enable_udp_receive_ = request->data;
  response->success = true;

  if (enable_udp_receive_) {
    response->message = "UDP receive enabled";
    LOG_INFO("UDP receive ENABLED");
  } else {
    response->message = "UDP receive disabled";
    LOG_INFO("UDP receive DISABLED");
  }
}

void TeleopVrRecvNode::leftFkPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  left_fk_pose_.position[0] = msg->pose.position.x;
  left_fk_pose_.position[1] = msg->pose.position.y;
  left_fk_pose_.position[2] = msg->pose.position.z;
  left_fk_pose_.orientation[0] = msg->pose.orientation.x;
  left_fk_pose_.orientation[1] = msg->pose.orientation.y;
  left_fk_pose_.orientation[2] = msg->pose.orientation.z;
  left_fk_pose_.orientation[3] = msg->pose.orientation.w;
  left_fk_received_ = true;
}

void TeleopVrRecvNode::rightFkPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  right_fk_pose_.position[0] = msg->pose.position.x;
  right_fk_pose_.position[1] = msg->pose.position.y;
  right_fk_pose_.position[2] = msg->pose.position.z;
  right_fk_pose_.orientation[0] = msg->pose.orientation.x;
  right_fk_pose_.orientation[1] = msg->pose.orientation.y;
  right_fk_pose_.orientation[2] = msg->pose.orientation.z;
  right_fk_pose_.orientation[3] = msg->pose.orientation.w;
  right_fk_received_ = true;
}

void TeleopVrRecvNode::leftWorkspaceCenterCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!left_arm_transformer_) {
    return;
  }

  const double center[3] = {msg->pose.position.x, msg->pose.position.y,
                            msg->pose.position.z};
  left_arm_transformer_->setWorkspaceCenter(center);
  left_workspace_center_received_ = true;
  if (left_workspace_center_waiting_logged_) {
    LOG_INFO("左臂收到工作空间球心topic，恢复发布target_pose");
    left_workspace_center_waiting_logged_ = false;
  }
}

void TeleopVrRecvNode::rightWorkspaceCenterCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!right_arm_transformer_) {
    return;
  }

  const double center[3] = {msg->pose.position.x, msg->pose.position.y,
                            msg->pose.position.z};
  right_arm_transformer_->setWorkspaceCenter(center);
  right_workspace_center_received_ = true;
  if (right_workspace_center_waiting_logged_) {
    LOG_INFO("右臂收到工作空间球心topic，恢复发布target_pose");
    right_workspace_center_waiting_logged_ = false;
  }
}

}  // namespace teleop_vr
