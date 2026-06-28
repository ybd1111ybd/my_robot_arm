#include "teleop_vr_recv/vr_smoothing_pipeline.h"

namespace teleop_vr {

SmoothedVrPoses VrSmoothingPipeline::smoothPoses(const VrDataPacket& packet)
{
    SmoothedVrPoses poses;

    auto [left_smoothed_pos, left_smoothed_rot] = smoother_.smoothLeftController(
        packet.left_controller.position,
        packet.left_controller.rotation);

    auto [right_smoothed_pos, right_smoothed_rot] = smoother_.smoothRightController(
        packet.right_controller.position,
        packet.right_controller.rotation);

    auto [headset_smoothed_pos, headset_smoothed_rot] = smoother_.smoothHeadset(
        packet.headset.position,
        packet.headset.rotation);

    poses.left_position = left_smoothed_pos;
    poses.left_rotation = left_smoothed_rot;
    poses.right_position = right_smoothed_pos;
    poses.right_rotation = right_smoothed_rot;
    poses.headset_position = headset_smoothed_pos;
    poses.headset_rotation = headset_smoothed_rot;

    return poses;
}

SmoothedControllerInputs VrSmoothingPipeline::smoothInputs(const VrDataPacket& packet)
{
    SmoothedControllerInputs inputs;

    inputs.left_input = toRawInputArray(packet.left_input);
    inputs.right_input = toRawInputArray(packet.right_input);

    return inputs;
}

std::array<float, 7> VrSmoothingPipeline::toRawInputArray(const ControllerInput& input)
{
    return {
        input.trigger,
        input.grip_button ? 1.0f : 0.0f,
        input.primary_button ? 1.0f : 0.0f,
        input.secondary_button ? 1.0f : 0.0f,
        input.menu_button ? 1.0f : 0.0f,
        input.joystick_x,
        input.joystick_y,
    };
}

} // namespace teleop_vr
