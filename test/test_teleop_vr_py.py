#!/usr/bin/env python3
"""Small self-checks for the pure Python VR teleop port."""

from __future__ import annotations

import math
import struct

from teleop_vr_py import RobotEndEffectorPose, VrCoordinateTransformer, VrDataParser


def make_packet(left_pos=(0.1, 0.2, 0.3), right_pos=(0.4, 0.5, 0.6)) -> bytes:
    data = bytearray()
    data += b"\xAA\xBB"
    data += bytes([62])
    data += struct.pack("<h", 0)
    data += struct.pack("<16f", *([0.0] * 16))
    data += struct.pack("<7f", left_pos[0], left_pos[1], left_pos[2], 0.0, 0.0, 0.0, 1.0)
    data += struct.pack("<7f", right_pos[0], right_pos[1], right_pos[2], 0.0, 0.0, 0.0, 1.0)
    data += struct.pack("<7f", 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    data += struct.pack("<fBBBBff", 0.0, 1, 0, 0, 0, 0.0, 0.0)
    data += struct.pack("<fBBBBff", 0.0, 0, 0, 0, 0, 0.0, 0.0)
    data += bytes(68)
    return bytes(data)


def assert_close(actual: float, expected: float, eps: float = 1e-6) -> None:
    if abs(actual - expected) > eps:
        raise AssertionError(f"{actual} != {expected}")


def main() -> int:
    packet = VrDataParser.parse(make_packet())
    assert packet is not None
    assert packet.length_units == 62
    assert packet.packet_size == 253
    assert packet.has_vr_device_poses
    assert packet.left_input.grip_button

    transformer = VrCoordinateTransformer(scale_factor=1.0)
    transformer.set_axis_mapping([-2, 3, 1])
    transformer.set_robot_home_pose([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    transformer.lock_vr_reference([0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0])
    target = transformer.transform([0.2, 0.4, 0.6], [0.0, 0.0, 0.0, 1.0])

    # VR delta [0.1, 0.2, 0.3] maps to robot [0.3, -0.1, 0.2].
    assert_close(target.position[0], 1.3)
    assert_close(target.position[1], 1.9)
    assert_close(target.position[2], 3.2)
    assert_close(math.sqrt(sum(q * q for q in target.orientation)), 1.0)

    pose = RobotEndEffectorPose.from_list([1, 2, 3, 0, 0, 0, 1])
    assert pose.as_list() == [1, 2, 3, 0, 0, 0, 1]
    print("test_teleop_vr_py: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
