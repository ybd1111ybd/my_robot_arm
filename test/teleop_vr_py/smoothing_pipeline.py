"""Convenience wrapper matching VrSmoothingPipeline in C++."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

from .data_smoother import VrDataSmoother
from .types import VrDataPacket


@dataclass
class SmoothedVrPoses:
    left_position: List[float]
    left_rotation: List[float]
    right_position: List[float]
    right_rotation: List[float]
    headset_position: List[float]
    headset_rotation: List[float]


@dataclass
class SmoothedControllerInputs:
    left_input: List[float]
    right_input: List[float]


class VrSmoothingPipeline:
    def __init__(self, smoother: VrDataSmoother) -> None:
        self.smoother = smoother

    def smooth_poses(self, packet: VrDataPacket) -> SmoothedVrPoses:
        left_position, left_rotation = self.smoother.smooth_left_controller(
            packet.left_controller.position,
            packet.left_controller.rotation,
        )
        right_position, right_rotation = self.smoother.smooth_right_controller(
            packet.right_controller.position,
            packet.right_controller.rotation,
        )
        headset_position, headset_rotation = self.smoother.smooth_headset(
            packet.headset.position,
            packet.headset.rotation,
        )
        return SmoothedVrPoses(
            left_position=left_position,
            left_rotation=left_rotation,
            right_position=right_position,
            right_rotation=right_rotation,
            headset_position=headset_position,
            headset_rotation=headset_rotation,
        )

    def smooth_inputs(self, packet: VrDataPacket) -> SmoothedControllerInputs:
        # C++ currently publishes raw input arrays to avoid filtering buttons.
        return SmoothedControllerInputs(
            left_input=packet.left_input.as_array(),
            right_input=packet.right_input.as_array(),
        )
