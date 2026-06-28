#include <array>
#include <cassert>
#include <string>

#include "teleop_vr_recv/vr_button_handler.h"

int main() {
  teleop_vr::VrCoordinateTransformer left_transformer;
  teleop_vr::VrCoordinateTransformer right_transformer;
  teleop_vr::ControllerInput left_input;
  teleop_vr::ControllerInput right_input;
  const std::array<float, 3> position{0.0f, 0.0f, 0.0f};
  const std::array<float, 4> orientation{0.0f, 0.0f, 0.0f, 1.0f};
  const teleop_vr::RobotEndEffectorPose fk_pose;

  const teleop_vr::ArmButtonContext left_context{
      left_input, position, orientation, true, fk_pose, left_transformer};
  const teleop_vr::ArmButtonContext right_context{
      right_input, position, orientation, true, fk_pose, right_transformer};

  teleop_vr::VrButtonHandler handler;
  int primary_press_count = 0;

  const auto info_logger = [](const std::string&) {};
  const auto warning_logger = [](const std::string&) {};
  const auto primary_callback = [&primary_press_count]() {
    ++primary_press_count;
  };

  handler.process(left_context, right_context, info_logger, warning_logger,
                  primary_callback);
  assert(primary_press_count == 0);

  left_input.primary_button = true;
  handler.process(left_context, right_context, info_logger, warning_logger,
                  primary_callback);
  assert(primary_press_count == 1);

  handler.process(left_context, right_context, info_logger, warning_logger,
                  primary_callback);
  assert(primary_press_count == 1);

  left_input.primary_button = false;
  right_input.primary_button = true;
  handler.process(left_context, right_context, info_logger, warning_logger,
                  primary_callback);
  assert(primary_press_count == 2);

  return 0;
}
