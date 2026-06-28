#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "data_smoother.h"
#include "udp_receiver.h"
#include "vr_coordinate_transformer.h"
#include "vr_data_parser.h"

namespace teleop_vr {

class VrSmoothingPipeline;
class VrButtonHandler;

/**
 * @brief VR遥操作接收节点
 * 接收UDP数据并发布关节命令、手柄状态和目标位姿
 */
class TeleopVrRecvNode : public rclcpp::Node {
 public:
  explicit TeleopVrRecvNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~TeleopVrRecvNode();

  /**
   * @brief 主循环,处理UDP接收和ROS2回调
   */
  void run();

 private:
  void initializeParameters();
  void initializeVrTransformers();
  void initializePublishers();
  void initializeSubscribers();
  void setupUdpReceiver();

  void onUdpDataReceived(const std::vector<uint8_t>& data, size_t size);
  void publishArmJointCommands(const VrDataPacket& packet);
  void publishGripperCommands(const VrDataPacket& packet);
  void publishVrDevicePoses(const VrDataPacket& packet,
                            const std::array<float, 3>& left_smoothed_pos,
                            const std::array<float, 4>& left_smoothed_rot,
                            const std::array<float, 3>& right_smoothed_pos,
                            const std::array<float, 4>& right_smoothed_rot,
                            const std::array<float, 3>& headset_smoothed_pos,
                            const std::array<float, 4>& headset_smoothed_rot);
  void publishControllerInputs(const VrDataPacket& packet,
                               const std::array<float, 3>& left_smoothed_pos,
                               const std::array<float, 4>& left_smoothed_rot,
                               const std::array<float, 3>& right_smoothed_pos,
                               const std::array<float, 4>& right_smoothed_rot);
  void publishCartesianTargetPose(
      const VrDataPacket& packet, const std::array<float, 3>& left_smoothed_pos,
      const std::array<float, 4>& left_smoothed_rot,
      const std::array<float, 3>& right_smoothed_pos,
      const std::array<float, 4>& right_smoothed_rot);
  geometry_msgs::msg::PoseStamped createTargetPoseMsg(
      const RobotEndEffectorPose& pose);
  void publishHoldTargetPose(
      const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
      geometry_msgs::msg::PoseStamped& last_target_pose,
      bool& has_last_target_pose);

  std::vector<double> degreesToRadians(const std::array<float, 7>& degrees);
  void initializeDataSmoother();

  void enableUdpReceiveCallback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void leftFkPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void rightFkPoseCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void leftWorkspaceCenterCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void rightWorkspaceCenterCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void handleTeleopModeButtons(const VrDataPacket& packet);
  void publishConfiguredHomeTargets();

  std::unique_ptr<UdpReceiver> udp_receiver_;

  std::unique_ptr<VrCoordinateTransformer> left_arm_transformer_;
  std::unique_ptr<VrCoordinateTransformer> right_arm_transformer_;

  std::unique_ptr<VrDataSmoother> data_smoother_;
  std::unique_ptr<VrSmoothingPipeline> smoothing_pipeline_;
  std::unique_ptr<VrButtonHandler> button_handler_;

  // 发布者 - JointState到SmoothMotionEngine
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      left_arm_smooth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      right_arm_smooth_pub_;

  // 发布者 - 夹爪命令
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      left_gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      right_gripper_pub_;

  // 发布者 - VR设备位姿 [x, y, z, qx, qy, qz, qw]
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      left_controller_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      right_controller_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      headset_pose_pub_;

  // 发布者 - 手柄输入 [trigger, grip_btn, primary_btn, secondary_btn,
  // menu_btn, joystick_x, joystick_y]
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      left_controller_input_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      right_controller_input_pub_;

  // 发布者 - 笛卡尔目标位姿
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      left_arm_target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      right_arm_target_pose_pub_;

  // 发布者 - UDP原始数据
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      udp_raw_data_pub_;

  // 订阅者 - 实时FK位姿
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      left_fk_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      right_fk_pose_sub_;

  // 订阅者 - 工作空间球心
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      left_workspace_center_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      right_workspace_center_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_udp_service_;
  std::string host_;
  int port_;
  bool enable_udp_receive_;

  RobotEndEffectorPose left_fk_pose_;
  RobotEndEffectorPose right_fk_pose_;
  bool left_fk_received_;
  bool right_fk_received_;

  geometry_msgs::msg::PoseStamped last_left_target_pose_;
  geometry_msgs::msg::PoseStamped last_right_target_pose_;
  bool has_last_left_target_pose_;
  bool has_last_right_target_pose_;
  bool last_left_grip_for_mode_;
  bool last_right_grip_for_mode_;
  bool last_left_primary_for_mode_;
  bool last_right_primary_for_mode_;

  bool left_workspace_center_use_topic_;
  bool right_workspace_center_use_topic_;
  bool left_workspace_center_received_;
  bool right_workspace_center_received_;
  bool left_workspace_center_waiting_logged_;
  bool right_workspace_center_waiting_logged_;
};

}  // namespace teleop_vr
