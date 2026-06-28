"""VR UDP packet parser compatible with teleop_vr_recv-main."""

from __future__ import annotations

import math
import struct
from typing import Optional

from .types import ControllerInput, VrDataPacket, VrDevicePose


class VrDataParser:
    FRAME_HEADER = b"\xAA\xBB"
    HEADER_SIZE = 2
    LENGTH_SIZE = 1
    TORQUE_SIZE = 2
    ANGLES_SIZE = 64
    VR_DEVICE_SIZE = 28
    VR_DEVICES_SIZE = 84
    CONTROLLER_INPUT_SIZE = 16
    MIN_PACKET_SIZE = HEADER_SIZE + LENGTH_SIZE + TORQUE_SIZE + ANGLES_SIZE
    FULL_PACKET_SIZE = MIN_PACKET_SIZE + VR_DEVICES_SIZE + 2 * CONTROLLER_INPUT_SIZE

    EXPECTED_DATA_LENGTH_OLD = 16
    EXPECTED_DATA_LENGTH_NEW = 62

    MAX_ABS_TORQUE = 500.0
    MAX_ABS_ANGLE_DEG = 720.0
    MAX_ABS_POSITION_M = 5.0
    MAX_ABS_JOYSTICK = 1.2

    @classmethod
    def parse(cls, data: bytes) -> Optional[VrDataPacket]:
        if len(data) < cls.MIN_PACKET_SIZE:
            return None
        if data[:2] != cls.FRAME_HEADER:
            return None

        length_units = data[2]
        declared_total_size = cls.HEADER_SIZE + cls.LENGTH_SIZE + length_units * 4
        if declared_total_size > len(data):
            return None

        packet = VrDataPacket(length_units=length_units, packet_size=len(data))
        torque_raw = struct.unpack_from("<h", data, 3)[0]
        packet.torque = torque_raw / 10.0

        offset = cls.HEADER_SIZE + cls.LENGTH_SIZE + cls.TORQUE_SIZE
        packet.angles = list(struct.unpack_from("<16f", data, offset))
        offset += cls.ANGLES_SIZE

        remaining = len(data) - offset
        if remaining == 0:
            return packet if cls.validate_packet_values(packet) else None

        needed = cls.VR_DEVICES_SIZE + 2 * cls.CONTROLLER_INPUT_SIZE
        if remaining < needed:
            return None

        packet.left_controller, offset = cls._parse_vr_device_pose(data, offset)
        packet.right_controller, offset = cls._parse_vr_device_pose(data, offset)
        packet.headset, offset = cls._parse_vr_device_pose(data, offset)
        packet.has_vr_device_poses = True

        packet.left_input, offset = cls._parse_controller_input(data, offset)
        packet.right_input, offset = cls._parse_controller_input(data, offset)
        packet.has_controller_inputs = True

        return packet if cls.validate_packet_values(packet) else None

    @classmethod
    def parse_or_raise(cls, data: bytes) -> VrDataPacket:
        packet = cls.parse(data)
        if packet is None:
            raise ValueError("invalid VR packet")
        return packet

    @staticmethod
    def _parse_float(data: bytes, offset: int) -> tuple[float, int]:
        return struct.unpack_from("<f", data, offset)[0], offset + 4

    @classmethod
    def _parse_vr_device_pose(cls, data: bytes, offset: int) -> tuple[VrDevicePose, int]:
        values = []
        for _ in range(7):
            value, offset = cls._parse_float(data, offset)
            values.append(value)
        return VrDevicePose(position=values[0:3], rotation=values[3:7]), offset

    @classmethod
    def _parse_controller_input(cls, data: bytes, offset: int) -> tuple[ControllerInput, int]:
        trigger, offset = cls._parse_float(data, offset)
        grip_button = bool(data[offset])
        primary_button = bool(data[offset + 1])
        secondary_button = bool(data[offset + 2])
        menu_button = bool(data[offset + 3])
        offset += 4
        joystick_x, offset = cls._parse_float(data, offset)
        joystick_y, offset = cls._parse_float(data, offset)
        return (
            ControllerInput(
                trigger=trigger,
                grip_button=grip_button,
                primary_button=primary_button,
                secondary_button=secondary_button,
                menu_button=menu_button,
                joystick_x=joystick_x,
                joystick_y=joystick_y,
            ),
            offset,
        )

    @staticmethod
    def _finite_in_range(value: float, min_value: float, max_value: float) -> bool:
        return math.isfinite(value) and min_value <= value <= max_value

    @classmethod
    def validate_packet_values(cls, packet: VrDataPacket) -> bool:
        if not cls._finite_in_range(packet.torque, -cls.MAX_ABS_TORQUE, cls.MAX_ABS_TORQUE):
            return False
        for angle in packet.angles:
            if not cls._finite_in_range(angle, -cls.MAX_ABS_ANGLE_DEG, cls.MAX_ABS_ANGLE_DEG):
                return False
        if packet.has_vr_device_poses:
            for pose in (packet.left_controller, packet.right_controller, packet.headset):
                if any(
                    not cls._finite_in_range(axis, -cls.MAX_ABS_POSITION_M, cls.MAX_ABS_POSITION_M)
                    for axis in pose.position
                ):
                    return False
                if any(not math.isfinite(q) for q in pose.rotation):
                    return False
                norm = math.sqrt(sum(q * q for q in pose.rotation))
                if norm <= 1e-6 or norm >= 10.0:
                    return False
        if packet.has_controller_inputs:
            for controller_input in (packet.left_input, packet.right_input):
                if not cls._finite_in_range(controller_input.trigger, -0.1, 1.1):
                    return False
                if not cls._finite_in_range(
                    controller_input.joystick_x, -cls.MAX_ABS_JOYSTICK, cls.MAX_ABS_JOYSTICK
                ):
                    return False
                if not cls._finite_in_range(
                    controller_input.joystick_y, -cls.MAX_ABS_JOYSTICK, cls.MAX_ABS_JOYSTICK
                ):
                    return False
        return True
