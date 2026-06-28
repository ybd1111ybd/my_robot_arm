#!/usr/bin/env python3
"""Pure Python demo for VR UDP receive, parse, smoothing, and target mapping.

This does not use ROS 2. It mirrors the controller-related behavior from
teleop_vr_recv-main and prints robot end-effector target poses.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from teleop_vr_py import (  # noqa: E402
    RobotEndEffectorPose,
    SmootherType,
    UdpReceiver,
    VrButtonHandler,
    VrCoordinateTransformer,
    VrDataParser,
    VrDataSmoother,
    VrSmoothingPipeline,
)
from teleop_vr_py.button_handler import ArmButtonContext  # noqa: E402


LEFT_HOME = [
    0.7324048656654268,
    0.3731828053351678,
    1.0581095836473335,
    -0.04307959999168663,
    0.06159595547961809,
    -0.6862686308289023,
    0.7234538358964814,
]
RIGHT_HOME = [
    0.7321293328673628,
    -0.3734108305057274,
    1.0580827168268605,
    0.04322027383914863,
    0.06156938877171767,
    0.6858984346948285,
    0.7237986982433239,
]


def fmt_pose(pose: RobotEndEffectorPose) -> str:
    x, y, z = pose.position
    qx, qy, qz, qw = pose.orientation
    return (
        f"pos=({x:+.4f}, {y:+.4f}, {z:+.4f}) "
        f"quat=({qx:+.4f}, {qy:+.4f}, {qz:+.4f}, {qw:+.4f})"
    )


def create_transformers(args) -> tuple[VrCoordinateTransformer, VrCoordinateTransformer]:
    left = VrCoordinateTransformer(args.scale)
    right = VrCoordinateTransformer(args.scale)
    left.set_axis_mapping(args.axis_mapping)
    right.set_axis_mapping(args.axis_mapping)
    left.set_tool_orientation_offset(VrCoordinateTransformer.left_tool_offset())
    right.set_tool_orientation_offset(VrCoordinateTransformer.right_tool_offset())
    left.set_robot_home_pose(LEFT_HOME[0:3], LEFT_HOME[3:7])
    right.set_robot_home_pose(RIGHT_HOME[0:3], RIGHT_HOME[3:7])
    left.set_workspace_center(args.left_workspace_center)
    right.set_workspace_center(args.right_workspace_center)
    left.set_workspace_limits(args.workspace_limit, args.workspace_radius, args.boundary_type)
    right.set_workspace_limits(args.workspace_limit, args.workspace_radius, args.boundary_type)
    return left, right


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pure Python VR receive/target mapping demo")
    parser.add_argument("--host", default="10.1.42.3", help="local IP to bind")
    parser.add_argument("--port", type=int, default=8080, help="UDP port")
    parser.add_argument("--timeout-ms", type=int, default=100, help="receive timeout")
    parser.add_argument("--print-every", type=int, default=10, help="print every N valid packets")
    parser.add_argument("--scale", type=float, default=1.0, help="VR delta to robot delta scale")
    parser.add_argument(
        "--axis-mapping",
        type=int,
        nargs=3,
        default=[-2, 3, 1],
        help="VR axis mapping, e.g. -2 3 1 means VR x->base -y, y->+z, z->+x",
    )
    parser.add_argument("--workspace-limit", action="store_true", help="enable spherical workspace limit")
    parser.add_argument("--workspace-radius", type=float, default=1.0, help="workspace sphere radius")
    parser.add_argument("--boundary-type", choices=["clamp", "saturate"], default="clamp")
    parser.add_argument(
        "--left-workspace-center",
        type=float,
        nargs=3,
        default=[0.2, 0.25, 0.6],
    )
    parser.add_argument(
        "--right-workspace-center",
        type=float,
        nargs=3,
        default=[0.2, -0.25, 0.6],
    )
    parser.add_argument(
        "--smoother-type",
        choices=[item.value for item in SmootherType],
        default=SmootherType.MOVING_AVG.value,
    )
    parser.add_argument("--position-alpha", type=float, default=0.25)
    parser.add_argument("--rotation-alpha", type=float, default=0.75)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    smoother_type = SmootherType(args.smoother_type)
    smoother = VrDataSmoother(
        enabled=smoother_type != SmootherType.NONE,
        smoother_type=smoother_type,
        position_alpha=args.position_alpha,
        rotation_alpha=args.rotation_alpha,
        input_alpha=1.0,
    )
    pipeline = VrSmoothingPipeline(smoother)
    left_transformer, right_transformer = create_transformers(args)
    button_handler = VrButtonHandler()

    left_current = RobotEndEffectorPose.from_list(LEFT_HOME)
    right_current = RobotEndEffectorPose.from_list(RIGHT_HOME)
    valid_count = 0
    bad_count = 0
    last_report_time = time.monotonic()
    last_report_count = 0

    def reset_home_targets() -> None:
        nonlocal left_current, right_current
        left_transformer.reset_to_home()
        right_transformer.reset_to_home()
        left_current = RobotEndEffectorPose.from_list(LEFT_HOME)
        right_current = RobotEndEffectorPose.from_list(RIGHT_HOME)

    print(f"listening on udp://{args.host}:{args.port}")
    print("hold Grip to lock relative control; press X/A primary to reset home; Ctrl+C to stop")
    with UdpReceiver(args.host, args.port) as receiver:
        try:
            while True:
                datagram = receiver.receive_once(args.timeout_ms)
                if datagram is None:
                    continue

                packet = VrDataParser.parse(datagram.data)
                if packet is None:
                    bad_count += 1
                    print(f"[bad #{bad_count}] from {datagram.address[0]}:{datagram.address[1]}")
                    continue
                if not packet.has_vr_device_poses or not packet.has_controller_inputs:
                    bad_count += 1
                    print(f"[bad #{bad_count}] packet has no controller pose/input data")
                    continue

                valid_count += 1
                smoothed = pipeline.smooth_poses(packet)
                button_handler.process(
                    ArmButtonContext(
                        packet.left_input,
                        smoothed.left_position,
                        smoothed.left_rotation,
                        True,
                        left_current,
                        left_transformer,
                    ),
                    ArmButtonContext(
                        packet.right_input,
                        smoothed.right_position,
                        smoothed.right_rotation,
                        True,
                        right_current,
                        right_transformer,
                    ),
                    info_logger=lambda message: print(f"[info] {message}"),
                    warning_logger=lambda message: print(f"[warn] {message}"),
                    primary_press_callback=reset_home_targets,
                )

                left_target = left_transformer.transform(smoothed.left_position, smoothed.left_rotation)
                right_target = right_transformer.transform(smoothed.right_position, smoothed.right_rotation)
                left_current = left_target
                right_current = right_target

                now = time.monotonic()
                fps = 0.0
                elapsed = now - last_report_time
                if elapsed >= 1.0:
                    fps = (valid_count - last_report_count) / elapsed
                    last_report_time = now
                    last_report_count = valid_count

                if valid_count % max(1, args.print_every) != 0:
                    continue
                print(
                    f"\n[packet #{valid_count}] from {datagram.address[0]}:{datagram.address[1]} "
                    f"bytes={len(datagram.data)} length={packet.length_units} fps={fps:.1f}"
                )
                print(
                    "  buttons "
                    f"L(grip={int(packet.left_input.grip_button)}, primary={int(packet.left_input.primary_button)}) "
                    f"R(grip={int(packet.right_input.grip_button)}, primary={int(packet.right_input.primary_button)})"
                )
                print(f"  left_target   {fmt_pose(left_target)}")
                print(f"  right_target  {fmt_pose(right_target)}")
        except KeyboardInterrupt:
            print(f"\nstopped. valid_packets={valid_count}, bad_packets={bad_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
