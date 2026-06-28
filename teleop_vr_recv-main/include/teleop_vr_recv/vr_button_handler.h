#pragma once

#include <array>
#include <functional>
#include <string>

#include "teleop_vr_recv/vr_coordinate_transformer.h"
#include "teleop_vr_recv/vr_data_parser.h"

namespace teleop_vr {

struct ArmButtonContext {
    const ControllerInput& controller_input;
    const std::array<float, 3>& smoothed_position;
    const std::array<float, 4>& smoothed_orientation;
    bool fk_received;
    const RobotEndEffectorPose& fk_pose;
    VrCoordinateTransformer& transformer;
};

class VrButtonHandler {
public:
    using LogCallback = std::function<void(const std::string&)>;
    using ButtonPressCallback = std::function<void()>;

    void process(
        const ArmButtonContext& left_context,
        const ArmButtonContext& right_context,
        const LogCallback& info_logger,
        const LogCallback& warning_logger,
        const ButtonPressCallback& primary_press_callback = nullptr);

    void reset();

private:
    struct LastButtonState {
        bool grip_button = false;
        bool primary_button = false;
    };

    void processArm(
        const ArmButtonContext& context,
        LastButtonState& last_state,
        bool is_left_arm,
        const LogCallback& info_logger,
        const LogCallback& warning_logger,
        const ButtonPressCallback& primary_press_callback);

    LastButtonState left_last_button_state_;
    LastButtonState right_last_button_state_;
};

} // namespace teleop_vr
