"""Grip and primary button state handling."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from .coordinate_transformer import VrCoordinateTransformer
from .types import ControllerInput, RobotEndEffectorPose


LogCallback = Callable[[str], None]
PrimaryPressCallback = Callable[[], None]


@dataclass
class ArmButtonContext:
    controller_input: ControllerInput
    smoothed_position: list[float]
    smoothed_orientation: list[float]
    fk_received: bool
    fk_pose: RobotEndEffectorPose
    transformer: VrCoordinateTransformer


@dataclass
class _LastButtonState:
    grip_button: bool = False
    primary_button: bool = False


class VrButtonHandler:
    def __init__(self) -> None:
        self.left_last_button_state = _LastButtonState()
        self.right_last_button_state = _LastButtonState()

    def process(
        self,
        left_context: ArmButtonContext,
        right_context: ArmButtonContext,
        info_logger: LogCallback = print,
        warning_logger: LogCallback = print,
        primary_press_callback: Optional[PrimaryPressCallback] = None,
    ) -> None:
        self._process_arm(
            left_context,
            self.left_last_button_state,
            True,
            info_logger,
            warning_logger,
            primary_press_callback,
        )
        self._process_arm(
            right_context,
            self.right_last_button_state,
            False,
            info_logger,
            warning_logger,
            primary_press_callback,
        )

    def reset(self) -> None:
        self.left_last_button_state = _LastButtonState()
        self.right_last_button_state = _LastButtonState()

    def _process_arm(
        self,
        context: ArmButtonContext,
        last_state: _LastButtonState,
        is_left_arm: bool,
        info_logger: LogCallback,
        warning_logger: LogCallback,
        primary_press_callback: Optional[PrimaryPressCallback],
    ) -> None:
        arm_name = "left" if is_left_arm else "right"
        if context.controller_input.grip_button and not last_state.grip_button:
            if not context.fk_received:
                warning_logger(f"{arm_name}: Grip pressed but no FK/current EE pose is available")
            else:
                context.transformer.set_robot_home_pose(
                    context.fk_pose.position,
                    context.fk_pose.orientation,
                )
                context.transformer.lock_vr_reference(
                    context.smoothed_position,
                    context.smoothed_orientation,
                )
                info_logger(f"{arm_name}: Grip pressed, relocked VR reference")

        if not context.controller_input.grip_button and last_state.grip_button:
            context.transformer.reset_vr_reference()
            info_logger(f"{arm_name}: Grip released, holding last target")
        last_state.grip_button = context.controller_input.grip_button

        if context.controller_input.primary_button and not last_state.primary_button:
            context.transformer.reset_vr_reference()
            if primary_press_callback is not None:
                primary_press_callback()
            info_logger(f"{arm_name}: primary button pressed, reset reference/home target")
        last_state.primary_button = context.controller_input.primary_button
