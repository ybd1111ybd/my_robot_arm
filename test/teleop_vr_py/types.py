"""Shared data structures for the pure Python VR receiver."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class VrDevicePose:
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rotation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])


@dataclass
class ControllerInput:
    trigger: float = 0.0
    grip_button: bool = False
    primary_button: bool = False
    secondary_button: bool = False
    menu_button: bool = False
    joystick_x: float = 0.0
    joystick_y: float = 0.0

    def as_array(self) -> List[float]:
        return [
            self.trigger,
            1.0 if self.grip_button else 0.0,
            1.0 if self.primary_button else 0.0,
            1.0 if self.secondary_button else 0.0,
            1.0 if self.menu_button else 0.0,
            self.joystick_x,
            self.joystick_y,
        ]


@dataclass
class VrDataPacket:
    torque: float = 0.0
    angles: List[float] = field(default_factory=lambda: [0.0] * 16)
    left_controller: VrDevicePose = field(default_factory=VrDevicePose)
    right_controller: VrDevicePose = field(default_factory=VrDevicePose)
    headset: VrDevicePose = field(default_factory=VrDevicePose)
    left_input: ControllerInput = field(default_factory=ControllerInput)
    right_input: ControllerInput = field(default_factory=ControllerInput)
    has_vr_device_poses: bool = False
    has_controller_inputs: bool = False
    length_units: int = 0
    packet_size: int = 0

    def get_left_arm_angles(self) -> List[float]:
        return self.angles[0:7]

    def get_right_arm_angles(self) -> List[float]:
        return self.angles[8:15]

    def get_left_gripper(self) -> float:
        return self.angles[7]

    def get_right_gripper(self) -> float:
        return self.angles[15]


@dataclass
class RobotEndEffectorPose:
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])

    @classmethod
    def from_list(cls, values: List[float]) -> "RobotEndEffectorPose":
        if len(values) != 7:
            raise ValueError("pose list must contain [x, y, z, qx, qy, qz, qw]")
        return cls(position=list(values[0:3]), orientation=list(values[3:7]))

    def as_list(self) -> List[float]:
        return list(self.position) + list(self.orientation)
