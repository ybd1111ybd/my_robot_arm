#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string readFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  std::stringstream contents;
  contents << file.rdbuf();
  return contents.str();
}

}  // namespace

int main() {
  const auto source_path =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "src" /
      "teleop_node.cpp";
  const auto source = readFile(source_path);

  const auto joint_branch = source.find(
      "if (publish_mode == \"joint\" || publish_mode == \"both\")");
  const auto cartesian_branch = source.find(
      "if (publish_mode == \"cartesian\" || publish_mode == \"both\")");
  const auto gripper_call = source.find("publishGripperCommands(packet);");
  const auto arm_publisher_creation = source.find(
      "left_arm_smooth_pub_ =\n"
      "        this->create_publisher<sensor_msgs::msg::JointState>");

  assert(joint_branch != std::string::npos);
  assert(cartesian_branch != std::string::npos);
  assert(gripper_call != std::string::npos);
  assert(arm_publisher_creation != std::string::npos);
  assert(joint_branch < gripper_call);
  assert(gripper_call < cartesian_branch);

  const auto joint_block = source.substr(joint_branch,
                                         gripper_call - joint_branch);
  assert(joint_block.find("publishArmJointCommands(packet);") !=
         std::string::npos);

  const auto publisher_gate = source.find(
      "if (publish_mode == \"joint\" || publish_mode == \"both\")");
  assert(publisher_gate != std::string::npos);
  assert(publisher_gate < arm_publisher_creation);

  return 0;
}
