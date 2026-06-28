#include "teleop_vr_recv/vr_button_handler.h"

namespace teleop_vr {

void VrButtonHandler::process(
    const ArmButtonContext& left_context,
    const ArmButtonContext& right_context,
    const LogCallback& info_logger,
    const LogCallback& warning_logger,
    const ButtonPressCallback& primary_press_callback)
{
    processArm(left_context, left_last_button_state_, true, info_logger, warning_logger,
               primary_press_callback);
    processArm(right_context, right_last_button_state_, false, info_logger, warning_logger,
               primary_press_callback);
}

void VrButtonHandler::reset()
{
    left_last_button_state_ = LastButtonState{};
    right_last_button_state_ = LastButtonState{};
}

void VrButtonHandler::processArm(
    const ArmButtonContext& context,
    LastButtonState& last_state,
    bool is_left_arm,
    const LogCallback& info_logger,
    const LogCallback& warning_logger,
    const ButtonPressCallback& primary_press_callback)
{
    if (context.controller_input.grip_button && !last_state.grip_button) {
        if (!context.fk_received) {
            warning_logger(is_left_arm
                ? "左臂Grip按下但尚未收到实时FK，忽略本次锁定"
                : "右臂Grip按下但尚未收到实时FK，忽略本次锁定");
        } else {
            context.transformer.setRobotHomePose(context.fk_pose.position, context.fk_pose.orientation);
            context.transformer.lockVrReference(
                context.smoothed_position.data(),
                context.smoothed_orientation.data());
            info_logger(is_left_arm
                ? "左臂Grip按下 - 使用实时FK重锁参考，开始增量控制"
                : "右臂Grip按下 - 使用实时FK重锁参考，开始增量控制");
        }
    }

    if (!context.controller_input.grip_button && last_state.grip_button) {
        context.transformer.resetVrReference();
        info_logger(is_left_arm
            ? "左臂Grip释放 - 停止笛卡尔跟随，保持当前位置"
            : "右臂Grip释放 - 停止笛卡尔跟随，保持当前位置");
    }
    last_state.grip_button = context.controller_input.grip_button;

    if (context.controller_input.primary_button && !last_state.primary_button) {
        context.transformer.resetVrReference();
        if (primary_press_callback) {
            primary_press_callback();
        }
        info_logger(is_left_arm
            ? "左手X键按下 - 左臂参考已解锁，发布预设VR初始末端target"
            : "右手A键按下 - 右臂参考已解锁，发布预设VR初始末端target");
    }
    last_state.primary_button = context.controller_input.primary_button;
}

} // namespace teleop_vr
