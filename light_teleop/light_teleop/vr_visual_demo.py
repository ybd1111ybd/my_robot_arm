"""VR controller to Pink + Meshcat visual teleoperation demo.

This demo intentionally does not publish robot commands. It receives the same
UDP packets as teleop_vr_recv-main, maps controller relative motion to left and
right end-effector targets, runs Pink IK, and displays the result in Meshcat.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pinocchio as pin
from meshcat import geometry as g
from pink.tasks import FrameTask

from .pink_solver import (
    DEFAULT_LEFT_EE_FRAME,
    DEFAULT_RIGHT_EE_FRAME,
    HOME_NEUTRAL,
    HOME_PREGRASP,
    HOME_VR_PREGRASP,
    PinkTeleopSolver,
    choose_qp_solver,
    limit_target_step,
    make_home_configuration,
)
from .model_loader import default_package_dir, default_urdf_path, load_and_validate_model, resolve_path
from .visualizer import TargetPair, TeleopVisualizer, parse_visual_meshes


REPO_ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = REPO_ROOT / "test"
if str(TEST_DIR) not in sys.path:
    sys.path.insert(0, str(TEST_DIR))

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


LEFT_RAW_COLOR = 0x00D5FF
RIGHT_RAW_COLOR = 0xFF3DAE
LEFT_MAPPED_COLOR = 0x00C853
RIGHT_MAPPED_COLOR = 0x00C853
LEFT_SENT_COLOR = 0xFF1F1F
RIGHT_SENT_COLOR = 0xFF1F1F
LEFT_RAW_REF_COLOR = 0x0077B6
RIGHT_RAW_REF_COLOR = 0xFFB703
LEFT_ANCHOR_COLOR = 0x0077FF
RIGHT_ANCHOR_COLOR = 0xFFD000
MAPPED_TARGET_BOX_SIZE = [0.075, 0.075, 0.075]
SENT_TARGET_BOX_SIZE = [0.045, 0.045, 0.045]
DEFAULT_LOG_FILE = Path(__file__).resolve().parent / "logs" / "vr_visual_demo.log"
DEFAULT_CONFIG_FILE = REPO_ROOT / "light_teleop" / "config" / "vr_visual_demo.yml"


def se3_to_ee_pose(transform: pin.SE3) -> RobotEndEffectorPose:
    """Convert Pinocchio SE3 to [position, qx qy qz qw]."""

    quat = pin.Quaternion(transform.rotation)
    quat.normalize()
    return RobotEndEffectorPose(
        position=transform.translation.tolist(),
        orientation=[quat.x, quat.y, quat.z, quat.w],
    )


def ee_pose_to_se3(pose: RobotEndEffectorPose) -> pin.SE3:
    """Convert [position, qx qy qz qw] to Pinocchio SE3."""

    qx, qy, qz, qw = pose.orientation
    quat = pin.Quaternion(qw, qx, qy, qz)
    quat.normalize()
    return pin.SE3(quat.toRotationMatrix(), np.array(pose.position, dtype=float))


def xyzw_to_rotation(orientation: list[float]) -> np.ndarray:
    qx, qy, qz, qw = orientation
    quat = pin.Quaternion(qw, qx, qy, qz)
    quat.normalize()
    return quat.toRotationMatrix()


def lock_target_orientation(target: pin.SE3, transformer: VrCoordinateTransformer) -> pin.SE3:
    return pin.SE3(xyzw_to_rotation(transformer.robot_reference_orientation), target.translation.copy())


def swing_target_orientation(target: pin.SE3, transformer: VrCoordinateTransformer) -> pin.SE3:
    reference_rotation = xyzw_to_rotation(transformer.robot_reference_orientation)
    reference_axis = reference_rotation[:, 2]
    target_axis = target.rotation[:, 2]
    axis_dot = float(np.clip(reference_axis.dot(target_axis), -1.0, 1.0))

    if axis_dot > 1.0 - 1e-8:
        swing_rotation = np.eye(3)
    elif axis_dot < -1.0 + 1e-8:
        helper = np.array([1.0, 0.0, 0.0])
        if abs(reference_axis.dot(helper)) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])
        rotation_axis = np.cross(reference_axis, helper)
        rotation_axis /= np.linalg.norm(rotation_axis)
        swing_rotation = pin.AngleAxis(np.pi, rotation_axis).toRotationMatrix()
    else:
        rotation_axis = np.cross(reference_axis, target_axis)
        rotation_axis /= np.linalg.norm(rotation_axis)
        swing_rotation = pin.AngleAxis(float(np.arccos(axis_dot)), rotation_axis).toRotationMatrix()
    return pin.SE3(swing_rotation @ reference_rotation, target.translation.copy())


def wrist_decoupled_target_orientation(
    target: pin.SE3,
    transformer: VrCoordinateTransformer,
    left_right_gain: float,
    up_down_gain: float,
    max_angle: float,
) -> pin.SE3:
    reference_rotation = xyzw_to_rotation(transformer.robot_reference_orientation)
    relative_rotation = reference_rotation.T @ target.rotation
    rotation_vector = pin.log3(relative_rotation)

    up_down = float(np.clip(rotation_vector[0] * up_down_gain, -max_angle, max_angle))
    left_right = float(np.clip(rotation_vector[1] * left_right_gain, -max_angle, max_angle))
    decoupled_rotation = pin.exp3(np.array([up_down, left_right, 0.0]))
    return pin.SE3(reference_rotation @ decoupled_rotation, target.translation.copy())


def apply_target_orientation_mode(
    target: pin.SE3,
    transformer: VrCoordinateTransformer,
    mode: str,
    wrist_left_right_gain: float,
    wrist_up_down_gain: float,
    wrist_max_angle: float,
) -> pin.SE3:
    if mode == "locked":
        return lock_target_orientation(target, transformer)
    if mode == "wrist-decoupled":
        return wrist_decoupled_target_orientation(
            target,
            transformer,
            wrist_left_right_gain,
            wrist_up_down_gain,
            wrist_max_angle,
        )
    if mode == "swing":
        return swing_target_orientation(target, transformer)
    return target


def parse_config_value(text: str) -> Any:
    value = text.strip()
    if not value:
        return ""
    if value[0] in {"'", '"'} and value[-1:] == value[0]:
        return value[1:-1]
    if value.startswith("[") and value.endswith("]"):
        items = value[1:-1].strip()
        if not items:
            return []
        return [parse_config_value(item) for item in items.split(",")]
    lowered = value.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    if lowered in {"null", "none"}:
        return None
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def load_simple_yaml_config(path: Path) -> dict[str, Any]:
    config: dict[str, Any] = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if ":" not in line:
            raise ValueError(f"invalid config line {line_number} in {path}: {raw_line}")
        key, value = line.split(":", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"empty config key on line {line_number} in {path}")
        config[key.replace("-", "_")] = parse_config_value(value)
    return config


def config_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_FILE),
        help="YAML config file loaded before CLI overrides. Use an empty path to disable.",
    )
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    config_args, remaining = config_parser().parse_known_args(argv)
    parser = build_arg_parser()
    parser.set_defaults(config=config_args.config)
    config_path_text = str(config_args.config)
    if config_path_text:
        config_path = Path(config_path_text).expanduser()
        if config_path.exists():
            known_dests = {action.dest for action in parser._actions}
            config = load_simple_yaml_config(config_path)
            unknown = sorted(set(config) - known_dests)
            if unknown:
                raise ValueError(f"unknown config keys in {config_path}: {', '.join(unknown)}")
            parser.set_defaults(**config)
    return parser.parse_args(remaining)


def translation_transform(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def offset_se3_local(transform: pin.SE3, local_offset: np.ndarray) -> np.ndarray:
    """Return homogeneous transform offset in the transform's local frame."""

    offset = pin.SE3(np.eye(3), local_offset)
    return (transform * offset).homogeneous


def offset_target_position_local(transform: pin.SE3, local_offset: np.ndarray) -> pin.SE3:
    return pin.SE3(transform.rotation.copy(), transform.translation + transform.rotation @ local_offset)


def tcp_control_to_frame_target(tcp_target: pin.SE3, local_tcp_offset: np.ndarray) -> pin.SE3:
    return pin.SE3(tcp_target.rotation.copy(), tcp_target.translation - tcp_target.rotation @ local_tcp_offset)


def local_axis_offset(axis: str, distance: float) -> np.ndarray:
    if axis == "x":
        return np.array([distance, 0.0, 0.0])
    if axis == "y":
        return np.array([0.0, distance, 0.0])
    if axis == "z":
        return np.array([0.0, 0.0, distance])
    raise ValueError(f"unsupported local offset axis: {axis}")


def vr_position_to_debug_world(
    vr_position: list[float],
    origin: np.ndarray,
    scale: float,
) -> np.ndarray:
    """Place a raw VR-space point in a separate Meshcat debug area.

    VR coordinates are not robot base coordinates, so raw points are shown in a
    small side display. VR Y is drawn as Meshcat Z so hand height is visually up.
    """

    x, y, z = vr_position
    return origin + scale * np.array([x, z, y], dtype=float)


def setup_vr_debug_markers(visualizer: TeleopVisualizer) -> None:
    viewer = visualizer.viewer
    viewer["vr_debug/left_raw"].set_object(
        g.Sphere(0.025),
        g.MeshLambertMaterial(color=LEFT_RAW_COLOR, opacity=0.9),
    )
    viewer["vr_debug/right_raw"].set_object(
        g.Sphere(0.025),
        g.MeshLambertMaterial(color=RIGHT_RAW_COLOR, opacity=0.9),
    )
    viewer["vr_debug/left_raw_ref"].set_object(
        g.Sphere(0.016),
        g.MeshLambertMaterial(color=LEFT_RAW_REF_COLOR, opacity=0.75),
    )
    viewer["vr_debug/right_raw_ref"].set_object(
        g.Sphere(0.016),
        g.MeshLambertMaterial(color=RIGHT_RAW_REF_COLOR, opacity=0.75),
    )
    viewer["vr_debug/left_mapped_target"].set_object(
        g.Box(MAPPED_TARGET_BOX_SIZE),
        g.MeshLambertMaterial(color=LEFT_MAPPED_COLOR, opacity=0.38, transparent=True),
    )
    viewer["vr_debug/right_mapped_target"].set_object(
        g.Box(MAPPED_TARGET_BOX_SIZE),
        g.MeshLambertMaterial(color=RIGHT_MAPPED_COLOR, opacity=0.38, transparent=True),
    )
    viewer["vr_debug/left_sent_target"].set_object(
        g.Box(SENT_TARGET_BOX_SIZE),
        g.MeshLambertMaterial(color=LEFT_SENT_COLOR, opacity=0.92),
    )
    viewer["vr_debug/right_sent_target"].set_object(
        g.Box(SENT_TARGET_BOX_SIZE),
        g.MeshLambertMaterial(color=RIGHT_SENT_COLOR, opacity=0.92),
    )
    viewer["vr_debug/left_anchor"].set_object(
        g.Sphere(0.015),
        g.MeshLambertMaterial(color=LEFT_ANCHOR_COLOR, opacity=0.95),
    )
    viewer["vr_debug/right_anchor"].set_object(
        g.Sphere(0.015),
        g.MeshLambertMaterial(color=RIGHT_ANCHOR_COLOR, opacity=0.95),
    )


def update_vr_debug_markers(
    visualizer: TeleopVisualizer,
    packet,
    left_transformer: VrCoordinateTransformer,
    right_transformer: VrCoordinateTransformer,
    left_mapped_target: pin.SE3,
    right_mapped_target: pin.SE3,
    left_target: pin.SE3,
    right_target: pin.SE3,
    origin: np.ndarray,
    scale: float,
    mapped_forward_offset: float,
    mapped_offset_axis: str,
) -> None:
    viewer = visualizer.viewer
    viewer["vr_debug/left_raw"].set_transform(
        translation_transform(vr_position_to_debug_world(packet.left_controller.position, origin, scale))
    )
    viewer["vr_debug/right_raw"].set_transform(
        translation_transform(vr_position_to_debug_world(packet.right_controller.position, origin, scale))
    )
    viewer["vr_debug/left_mapped_target"].set_transform(
        offset_se3_local(left_mapped_target, local_axis_offset(mapped_offset_axis, mapped_forward_offset))
    )
    viewer["vr_debug/right_mapped_target"].set_transform(
        offset_se3_local(right_mapped_target, local_axis_offset(mapped_offset_axis, mapped_forward_offset))
    )
    viewer["vr_debug/left_sent_target"].set_transform(
        offset_se3_local(left_target, local_axis_offset(mapped_offset_axis, mapped_forward_offset))
    )
    viewer["vr_debug/right_sent_target"].set_transform(
        offset_se3_local(right_target, local_axis_offset(mapped_offset_axis, mapped_forward_offset))
    )

    viewer["vr_debug/left_raw_ref"].set_property("visible", left_transformer.reference_locked)
    viewer["vr_debug/right_raw_ref"].set_property("visible", right_transformer.reference_locked)
    viewer["vr_debug/left_anchor"].set_property("visible", left_transformer.reference_locked)
    viewer["vr_debug/right_anchor"].set_property("visible", right_transformer.reference_locked)
    if left_transformer.reference_locked:
        viewer["vr_debug/left_raw_ref"].set_transform(
            translation_transform(
                vr_position_to_debug_world(left_transformer.vr_reference_position, origin, scale)
            )
        )
        viewer["vr_debug/left_anchor"].set_transform(
            translation_transform(np.array(left_transformer.robot_reference_position, dtype=float))
        )
    if right_transformer.reference_locked:
        viewer["vr_debug/right_raw_ref"].set_transform(
            translation_transform(
                vr_position_to_debug_world(right_transformer.vr_reference_position, origin, scale)
            )
        )
        viewer["vr_debug/right_anchor"].set_transform(
            translation_transform(np.array(right_transformer.robot_reference_position, dtype=float))
        )


def make_transformers(
    left_home: pin.SE3,
    right_home: pin.SE3,
    scale: float,
    axis_mapping: list[int],
) -> tuple[VrCoordinateTransformer, VrCoordinateTransformer]:
    """Create left/right relative VR-to-EE transformers."""

    left = VrCoordinateTransformer(scale)
    right = VrCoordinateTransformer(scale)
    left.set_axis_mapping(axis_mapping)
    right.set_axis_mapping(axis_mapping)
    left.set_tool_orientation_offset(VrCoordinateTransformer.left_tool_offset())
    right.set_tool_orientation_offset(VrCoordinateTransformer.right_tool_offset())
    left.set_robot_home_pose(se3_to_ee_pose(left_home).position, se3_to_ee_pose(left_home).orientation)
    right.set_robot_home_pose(se3_to_ee_pose(right_home).position, se3_to_ee_pose(right_home).orientation)
    # Keep the workspace sphere disabled in this visual demo. Pink target speed
    # limiting is still available via --target-max-speed.
    left.set_workspace_limits(False, 1.0, "clamp")
    right.set_workspace_limits(False, 1.0, "clamp")
    return left, right


def posture_cost_vector(base_cost: float, profile: str) -> np.ndarray | float:
    """Return a uniform or per-joint posture cost for the reduced 14-DoF model."""

    if profile == "uniform":
        return base_cost
    if profile not in {
        "front-flexible",
        "wrist-balanced",
        "wrist-priority",
        "joint7-priority",
        "joint6-priority",
        "wrist-cross-priority",
        "wrist-free",
    }:
        raise ValueError(f"unsupported posture cost profile: {profile}")

    if profile == "front-flexible":
        one_arm_weights = np.array(
            [
                1.00,
                1.00,
                0.80,
                0.65,
                0.30,
                0.25,
                0.25,
            ],
            dtype=float,
        )
    elif profile == "wrist-balanced":
        one_arm_weights = np.array(
            [
                1.00,
                1.00,
                0.80,
                0.60,
                0.18,
                0.12,
                0.12,
            ],
            dtype=float,
        )
    elif profile == "wrist-priority":
        one_arm_weights = np.array(
            [
                1.80,
                1.80,
                1.35,
                1.00,
                0.12,
                0.01,
                0.01,
            ],
            dtype=float,
        )
    elif profile == "joint7-priority":
        one_arm_weights = np.array(
            [
                1.90,
                1.90,
                1.45,
                1.10,
                0.85,
                1.20,
                0.01,
            ],
            dtype=float,
        )
    elif profile == "wrist-cross-priority":
        one_arm_weights = np.array(
            [
                1.90,
                1.90,
                1.45,
                1.10,
                0.85,
                0.08,
                0.01,
            ],
            dtype=float,
        )
    elif profile == "joint6-priority":
        one_arm_weights = np.array(
            [
                1.90,
                1.90,
                1.45,
                1.10,
                0.85,
                0.005,
                0.08,
            ],
            dtype=float,
        )
    else:
        one_arm_weights = np.array(
            [
                1.00,
                1.00,
                0.75,
                0.55,
                0.08,
                0.03,
                0.03,
            ],
            dtype=float,
        )
    return base_cost * np.concatenate([one_arm_weights, one_arm_weights])


def joint_motion_cost_vector(base_cost: float, profile: str) -> np.ndarray | float:
    """Return per-joint cost for minimizing one-step joint motion."""

    if base_cost <= 0.0:
        return 0.0
    if profile == "uniform":
        return base_cost
    if profile != "shoulder-heavy":
        raise ValueError(f"unsupported joint motion cost profile: {profile}")

    one_arm_weights = np.array(
        [
            5.0,
            5.0,
            2.5,
            2.0,
            1.0,
            0.7,
            0.7,
        ],
        dtype=float,
    )
    return base_cost * np.concatenate([one_arm_weights, one_arm_weights])


def wrist_motion_summary(solver: PinkTeleopSolver) -> str:
    """Summarize joint5/6/7 motion relative to home for both arms."""

    q = solver.configuration.q
    q0 = solver.q0_reduced
    left = q[solver.left_q_indices[4:7]] - q0[solver.left_q_indices[4:7]]
    right = q[solver.right_q_indices[4:7]] - q0[solver.right_q_indices[4:7]]
    return (
        "left_wrist_delta="
        f"({left[0]:+.3f}, {left[1]:+.3f}, {left[2]:+.3f}) "
        "right_wrist_delta="
        f"({right[0]:+.3f}, {right[1]:+.3f}, {right[2]:+.3f})"
    )


@dataclass
class JointMotionStats:
    """Running statistics for per-frame arm joint movement."""

    samples: int = 0
    total_l2: float = 0.0
    max_l2: float = 0.0
    max_abs: float = 0.0
    max_shoulder_abs: float = 0.0

    def update(self, delta_q: np.ndarray) -> None:
        abs_delta = np.abs(delta_q)
        shoulder_abs = abs_delta[[0, 1, 7, 8]]
        l2 = float(np.linalg.norm(delta_q))
        self.samples += 1
        self.total_l2 += l2
        self.max_l2 = max(self.max_l2, l2)
        self.max_abs = max(self.max_abs, float(np.max(abs_delta)))
        self.max_shoulder_abs = max(self.max_shoulder_abs, float(np.max(shoulder_abs)))

    @property
    def mean_l2(self) -> float:
        if self.samples == 0:
            return 0.0
        return self.total_l2 / self.samples

    def summary(self, dt: float) -> str:
        return (
            f"joint_step_l2_mean={self.mean_l2:.5f} "
            f"joint_step_l2_max={self.max_l2:.5f} "
            f"joint_step_abs_max={self.max_abs:.5f} "
            f"shoulder_step_abs_max={self.max_shoulder_abs:.5f} "
            f"joint_vel_abs_max={self.max_abs / dt:.3f} "
            f"shoulder_vel_abs_max={self.max_shoulder_abs / dt:.3f}"
        )


@dataclass(frozen=True)
class LoopTiming:
    frequency: float
    dt: float
    steps: int


def make_loop_timing(frequency: float, duration: float) -> LoopTiming:
    if frequency <= 0.0:
        raise ValueError("--frequency must be positive")
    if duration <= 0.0:
        raise ValueError("--duration must be positive")
    return LoopTiming(
        frequency=frequency,
        dt=1.0 / frequency,
        steps=int(round(duration * frequency)),
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, parents=[config_parser()])
    parser.add_argument("--host", default="10.1.42.3", help="local UDP bind IP")
    parser.add_argument("--port", type=int, default=8080, help="VR UDP port")
    parser.add_argument("--udp-timeout-ms", type=int, default=1)
    parser.add_argument(
        "--max-packets-per-frame",
        type=int,
        default=64,
        help="Drain at most this many UDP packets per IK frame and use the newest one.",
    )
    parser.add_argument("--urdf", default=str(default_urdf_path()))
    parser.add_argument("--package-dir", default=str(default_package_dir()))
    parser.add_argument("--left-ee-frame", default=DEFAULT_LEFT_EE_FRAME)
    parser.add_argument("--right-ee-frame", default=DEFAULT_RIGHT_EE_FRAME)
    parser.add_argument("--solver", default=None)
    parser.add_argument(
        "--home",
        choices=[HOME_NEUTRAL, HOME_PREGRASP, HOME_VR_PREGRASP],
        default=HOME_VR_PREGRASP,
    )
    parser.add_argument("--posture-cost", type=float, default=0.01)
    parser.add_argument(
        "--joint-motion-cost",
        type=float,
        default=0.0,
        help="Per-step joint motion cost. Increase to discourage large joint changes.",
    )
    parser.add_argument(
        "--joint-motion-cost-profile",
        choices=["uniform", "shoulder-heavy"],
        default="shoulder-heavy",
        help="Joint motion weighting profile; shoulder-heavy penalizes shoulder joints most.",
    )
    parser.add_argument(
        "--posture-cost-profile",
        choices=[
            "uniform",
            "front-flexible",
            "wrist-balanced",
            "wrist-priority",
            "joint7-priority",
            "joint6-priority",
            "wrist-cross-priority",
            "wrist-free",
        ],
        default="uniform",
        help="Use joint6-priority for left/right wrist motion; joint7-priority for up/down wrist bend.",
    )
    parser.add_argument(
        "--position-cost",
        type=float,
        default=1.0,
        help="Pink FrameTask position cost. Lower values reduce whole-arm point chasing.",
    )
    parser.add_argument(
        "--orientation-cost",
        type=float,
        default=0.05,
        help="Pink FrameTask orientation cost. Increase this to make wrist/cross joints participate more.",
    )
    parser.add_argument(
        "--target-orientation-mode",
        choices=["wrist-decoupled", "swing", "locked", "vr"],
        default="wrist-decoupled",
        help="Use wrist-decoupled to map local Y to joint6 left/right and local X to joint7 up/down.",
    )
    parser.add_argument(
        "--wrist-left-right-gain",
        type=float,
        default=1.0,
        help="Signed gain for wrist-decoupled local-Y rotation, mainly driving joint6/link6 left-right motion.",
    )
    parser.add_argument(
        "--wrist-up-down-gain",
        type=float,
        default=1.0,
        help="Gain for wrist-decoupled local-X rotation, mainly driving joint7/link7 up-down motion.",
    )
    parser.add_argument(
        "--wrist-max-angle",
        type=float,
        default=0.65,
        help="Clamp each wrist-decoupled rotation channel to this angle in radians.",
    )
    parser.add_argument(
        "--tcp-control-offset",
        type=float,
        default=0.0,
        help="Move the controlled target point this far along --tcp-control-offset-axis in the local EE frame.",
    )
    parser.add_argument(
        "--tcp-control-offset-axis",
        choices=["x", "y", "z"],
        default="z",
        help="Local EE axis for --tcp-control-offset. Use the same axis as the visible tool-forward direction.",
    )
    parser.add_argument(
        "--stabilize-camera",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Keep left_arm_link10/right_arm_link10 camera orientation close to the initial pose.",
    )
    parser.add_argument(
        "--camera-orientation-cost",
        type=float,
        default=0.6,
        help="Orientation cost for camera/link10 stabilization tasks.",
    )
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--frequency", type=float, default=60.0)
    parser.add_argument(
        "--display-every",
        type=int,
        default=1,
        help="Update Meshcat every N IK frames. Use 2 or 3 if mesh display causes lag.",
    )
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--axis-mapping", type=int, nargs=3, default=[-2, 3, 1])
    parser.add_argument(
        "--target-max-speed",
        type=float,
        default=0.80,
        help="Target translation speed limit in m/s; 0 disables it. Low values feel laggy on large hand motions.",
    )
    parser.add_argument("--print-every", type=int, default=30)
    parser.add_argument("--open-browser", action="store_true")
    parser.add_argument("--no-meshes", action="store_true")
    parser.add_argument("--arm-meshes-only", action="store_true")
    parser.add_argument(
        "--show-vr-debug",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show raw VR hand points and mapped target points in Meshcat.",
    )
    parser.add_argument(
        "--vr-debug-origin",
        type=float,
        nargs=3,
        default=[-0.65, 0.0, 0.15],
        help="Meshcat world origin for raw VR-space debug points.",
    )
    parser.add_argument(
        "--vr-debug-scale",
        type=float,
        default=0.35,
        help="Scale used when drawing raw VR-space debug points.",
    )
    parser.add_argument(
        "--vr-debug-target-forward-offset",
        type=float,
        default=-0.05,
        help="Draw mapped target boxes this far along --vr-debug-target-offset-axis.",
    )
    parser.add_argument(
        "--vr-debug-target-offset-axis",
        choices=["x", "y", "z"],
        default="z",
        help="Local frame axis used to offset mapped target boxes. Meshcat blue axis is usually local z.",
    )
    parser.add_argument(
        "--log-file",
        default=str(DEFAULT_LOG_FILE),
        help="Run log path. The file is overwritten on each start.",
    )
    parser.add_argument(
        "--smoother-type",
        choices=[item.value for item in SmootherType],
        default=SmootherType.MOVING_AVG.value,
    )
    parser.add_argument("--position-alpha", type=float, default=0.25)
    parser.add_argument("--rotation-alpha", type=float, default=0.75)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_path = Path(args.log_file).expanduser()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_handle = log_path.open("w", encoding="utf-8")

    def log(message: str) -> None:
        print(message)
        log_handle.write(message + "\n")
        log_handle.flush()

    loop_timing = make_loop_timing(args.frequency, args.duration)
    if args.target_max_speed < 0.0:
        raise ValueError("--target-max-speed must be non-negative")
    if args.display_every < 1:
        raise ValueError("--display-every must be at least 1")
    if args.orientation_cost < 0.0:
        raise ValueError("--orientation-cost must be non-negative")
    if args.position_cost < 0.0:
        raise ValueError("--position-cost must be non-negative")
    if args.joint_motion_cost < 0.0:
        raise ValueError("--joint-motion-cost must be non-negative")
    if args.camera_orientation_cost < 0.0:
        raise ValueError("--camera-orientation-cost must be non-negative")
    if args.wrist_up_down_gain < 0.0:
        raise ValueError("--wrist-up-down-gain must be non-negative")
    if args.wrist_max_angle < 0.0:
        raise ValueError("--wrist-max-angle must be non-negative")

    urdf_path = resolve_path(args.urdf)
    package_dir = resolve_path(args.package_dir)
    load_result = load_and_validate_model(urdf_path, package_dir, load_geometry=False)
    home_q = make_home_configuration(load_result.robot.model, load_result.configuration.q, args.home)
    solver = PinkTeleopSolver(
        robot=load_result.robot,
        q0=home_q,
        left_ee_frame=args.left_ee_frame,
        right_ee_frame=args.right_ee_frame,
        solver=args.solver or choose_qp_solver(),
        posture_cost=args.posture_cost,
        home_name=args.home,
        joint_motion_cost=args.joint_motion_cost,
    )
    solver.posture_task.cost = posture_cost_vector(args.posture_cost, args.posture_cost_profile)
    solver.motion_task.cost = joint_motion_cost_vector(
        args.joint_motion_cost,
        args.joint_motion_cost_profile,
    )
    solver.left_task.set_position_cost(args.position_cost)
    solver.right_task.set_position_cost(args.position_cost)
    solver.left_task.set_orientation_cost(args.orientation_cost)
    solver.right_task.set_orientation_cost(args.orientation_cost)
    camera_tasks = []
    if args.stabilize_camera:
        for frame_name in ("left_arm_link10", "right_arm_link10"):
            if not solver.model.existFrame(frame_name):
                raise ValueError(f"camera stabilization frame not found in reduced model: {frame_name}")
        left_camera_task = FrameTask(
            "left_arm_link10",
            position_cost=0.0,
            orientation_cost=args.camera_orientation_cost,
            lm_damping=1e-4,
        )
        right_camera_task = FrameTask(
            "right_arm_link10",
            position_cost=0.0,
            orientation_cost=args.camera_orientation_cost,
            lm_damping=1e-4,
        )
        left_camera_task.set_target(solver.configuration.get_transform_frame_to_world("left_arm_link10"))
        right_camera_task.set_target(solver.configuration.get_transform_frame_to_world("right_arm_link10"))
        camera_tasks = [left_camera_task, right_camera_task]
        solver.tasks = [solver.left_task, solver.right_task, *camera_tasks, solver.posture_task]
        if solver.joint_motion_cost > 0.0:
            solver.tasks.append(solver.motion_task)

    visual_meshes, missing_visual_meshes = parse_visual_meshes(
        urdf_path,
        package_dir,
        show_meshes=not args.no_meshes,
    )
    if args.arm_meshes_only:
        visual_meshes = [
            mesh
            for mesh in visual_meshes
            if mesh.link_name.startswith("left_") or mesh.link_name.startswith("right_")
        ]
    visualizer = TeleopVisualizer(
        solver.full_model,
        solver.full_data,
        left_ee_frame=solver.left_ee_frame,
        right_ee_frame=solver.right_ee_frame,
        visual_meshes=visual_meshes,
        open_browser=args.open_browser,
    )
    if args.show_vr_debug:
        setup_vr_debug_markers(visualizer)

    left_home, right_home = solver.get_current_ee_poses()
    left_transformer, right_transformer = make_transformers(
        left_home,
        right_home,
        args.scale,
        args.axis_mapping,
    )
    tcp_control_offset = local_axis_offset(args.tcp_control_offset_axis, args.tcp_control_offset)
    left_home_tcp = offset_target_position_local(left_home, tcp_control_offset)
    right_home_tcp = offset_target_position_local(right_home, tcp_control_offset)
    left_home_tcp_pose = se3_to_ee_pose(left_home_tcp)
    right_home_tcp_pose = se3_to_ee_pose(right_home_tcp)
    left_transformer.set_robot_home_pose(left_home_tcp_pose.position, left_home_tcp_pose.orientation)
    right_transformer.set_robot_home_pose(right_home_tcp_pose.position, right_home_tcp_pose.orientation)
    left_target = left_home.copy()
    right_target = right_home.copy()
    left_mapped_target = left_target.copy()
    right_mapped_target = right_target.copy()
    left_mapped_tcp_target = offset_target_position_local(left_mapped_target, tcp_control_offset)
    right_mapped_tcp_target = offset_target_position_local(right_mapped_target, tcp_control_offset)
    left_tcp_target = left_mapped_tcp_target.copy()
    right_tcp_target = right_mapped_tcp_target.copy()
    last_left_target = left_target.copy()
    last_right_target = right_target.copy()
    solver.set_targets(left_target, right_target)

    smoother_type = SmootherType(args.smoother_type)
    smoother = VrDataSmoother(
        enabled=smoother_type != SmootherType.NONE,
        smoother_type=smoother_type,
        position_alpha=args.position_alpha,
        rotation_alpha=args.rotation_alpha,
        input_alpha=1.0,
    )
    smoothing_pipeline = VrSmoothingPipeline(smoother)
    button_handler = VrButtonHandler()

    packet_count = 0
    dropped_packet_count = 0
    bad_count = 0
    last_print_packet_count = 0
    last_debug_packet = None
    dt = loop_timing.dt
    steps = loop_timing.steps
    start = time.monotonic()
    vr_debug_origin = np.array(args.vr_debug_origin, dtype=float)
    joint_motion_stats = JointMotionStats()
    last_arm_q = solver.configuration.q.copy()

    def reset_home_targets() -> None:
        nonlocal left_mapped_target, right_mapped_target, left_target, right_target
        nonlocal left_mapped_tcp_target, right_mapped_tcp_target, left_tcp_target, right_tcp_target
        nonlocal last_left_target, last_right_target, last_arm_q
        solver.configuration.update(solver.q0_reduced)
        solver.posture_task.set_target(solver.q0_reduced)
        solver.motion_task.set_target(solver.configuration.q)
        last_arm_q = solver.configuration.q.copy()
        left_transformer.reset_to_home()
        right_transformer.reset_to_home()
        left_target = left_home.copy()
        right_target = right_home.copy()
        left_mapped_target = left_target.copy()
        right_mapped_target = right_target.copy()
        left_mapped_tcp_target = offset_target_position_local(left_mapped_target, tcp_control_offset)
        right_mapped_tcp_target = offset_target_position_local(right_mapped_target, tcp_control_offset)
        left_tcp_target = left_mapped_tcp_target.copy()
        right_tcp_target = right_mapped_tcp_target.copy()
        last_left_target = left_target.copy()
        last_right_target = right_target.copy()
        solver.set_targets(left_target, right_target)
        if camera_tasks:
            camera_tasks[0].set_target(solver.configuration.get_transform_frame_to_world("left_arm_link10"))
            camera_tasks[1].set_target(solver.configuration.get_transform_frame_to_world("right_arm_link10"))

    log(f"[vr_visual_demo] log_file: {log_path}")
    log(f"[vr_visual_demo] Meshcat URL: {visualizer.url()}")
    log(f"[vr_visual_demo] listening on udp://{args.host}:{args.port}")
    log(f"[vr_visual_demo] solver_frequency: {loop_timing.frequency:g} Hz dt={loop_timing.dt:.6f}s")
    log("[vr_visual_demo] hold Grip to move, release Grip to hold, press X/A to reset home")
    log(f"[vr_visual_demo] posture_cost_profile: {args.posture_cost_profile}")
    log(
        "[vr_visual_demo] joint_motion_cost: "
        f"{args.joint_motion_cost:g} profile={args.joint_motion_cost_profile}"
    )
    log(f"[vr_visual_demo] position_cost: {args.position_cost:g}")
    log(f"[vr_visual_demo] orientation_cost: {args.orientation_cost:g}")
    log(f"[vr_visual_demo] target_orientation_mode: {args.target_orientation_mode}")
    if args.target_orientation_mode == "wrist-decoupled":
        log(
            "[vr_visual_demo] wrist_decoupled: "
            f"left_right_gain={args.wrist_left_right_gain:g}, "
            f"up_down_gain={args.wrist_up_down_gain:g}, "
            f"max_angle={args.wrist_max_angle:g} rad"
        )
    if args.tcp_control_offset != 0.0:
        log(
            "[vr_visual_demo] tcp control point: "
            f"axis={args.tcp_control_offset_axis}, offset={args.tcp_control_offset:.3f} m"
        )
    if args.stabilize_camera:
        log(f"[vr_visual_demo] camera/link10 stabilization enabled: cost={args.camera_orientation_cost:g}")
    if 0.0 < args.target_max_speed < 0.5:
        log(
            "[vr_visual_demo] note: low --target-max-speed can feel laggy on large hand motions; "
            "try 0.8 or 0 after confirming targets are stable"
        )
    if args.no_meshes:
        log("[vr_visual_demo] visual meshes disabled")
    else:
        log(f"[vr_visual_demo] visual meshes loaded: {len(visual_meshes)}")
    if args.show_vr_debug:
        log(
            "[vr_visual_demo] VR debug markers: "
            "cyan/magenta raw spheres=VR controller points, "
            "green boxes=mapped-back targets before speed limit, "
            "red boxes=targets sent to Pink after speed limit, "
            "blue/yellow spheres=left/right Grip binding origins"
        )
        log(
            "[vr_visual_demo] mapped target box offset: "
            f"axis={args.vr_debug_target_offset_axis}, distance={args.vr_debug_target_forward_offset:.3f} m"
        )
    if missing_visual_meshes:
        log(f"[vr_visual_demo] warning: {len(missing_visual_meshes)} visual mesh assets are missing")

    result = solver._make_result()
    try:
        with UdpReceiver(args.host, args.port) as receiver:
            for step_index in range(steps):
                loop_start = time.monotonic()
                datagram, drained_count = receiver.receive_latest(
                    args.udp_timeout_ms,
                    args.max_packets_per_frame,
                )
                if drained_count > 1:
                    dropped_packet_count += drained_count - 1
                if datagram is not None:
                    packet = VrDataParser.parse(datagram.data)
                    if packet is None or not packet.has_vr_device_poses or not packet.has_controller_inputs:
                        bad_count += 1
                    else:
                        packet_count += 1
                        last_debug_packet = packet
                        smoothed = smoothing_pipeline.smooth_poses(packet)
                        current_left, current_right = solver.get_current_ee_poses()
                        current_left_tcp = offset_target_position_local(current_left, tcp_control_offset)
                        current_right_tcp = offset_target_position_local(current_right, tcp_control_offset)
                        button_handler.process(
                            ArmButtonContext(
                                packet.left_input,
                                smoothed.left_position,
                                smoothed.left_rotation,
                                True,
                                se3_to_ee_pose(current_left_tcp),
                                left_transformer,
                            ),
                            ArmButtonContext(
                                packet.right_input,
                                smoothed.right_position,
                                smoothed.right_rotation,
                                True,
                                se3_to_ee_pose(current_right_tcp),
                                right_transformer,
                            ),
                            info_logger=lambda message: log(f"[vr_visual_demo] {message}"),
                            warning_logger=lambda message: log(f"[vr_visual_demo] warning: {message}"),
                            primary_press_callback=reset_home_targets,
                        )
                        left_mapped_target = ee_pose_to_se3(
                            left_transformer.transform(smoothed.left_position, smoothed.left_rotation)
                        )
                        right_mapped_target = ee_pose_to_se3(
                            right_transformer.transform(smoothed.right_position, smoothed.right_rotation)
                        )
                        left_mapped_target = apply_target_orientation_mode(
                            left_mapped_target,
                            left_transformer,
                            args.target_orientation_mode,
                            args.wrist_left_right_gain,
                            args.wrist_up_down_gain,
                            args.wrist_max_angle,
                        )
                        right_mapped_target = apply_target_orientation_mode(
                            right_mapped_target,
                            right_transformer,
                            args.target_orientation_mode,
                            args.wrist_left_right_gain,
                            args.wrist_up_down_gain,
                            args.wrist_max_angle,
                        )
                        left_mapped_tcp_target = left_mapped_target.copy()
                        right_mapped_tcp_target = right_mapped_target.copy()
                        left_mapped_target = tcp_control_to_frame_target(left_mapped_tcp_target, tcp_control_offset)
                        right_mapped_target = tcp_control_to_frame_target(right_mapped_tcp_target, tcp_control_offset)
                        left_target = left_mapped_target.copy()
                        right_target = right_mapped_target.copy()

                if args.target_max_speed > 0.0:
                    max_step = args.target_max_speed * dt
                    left_target = limit_target_step(last_left_target, left_mapped_target, max_step)
                    right_target = limit_target_step(last_right_target, right_mapped_target, max_step)
                left_tcp_target = offset_target_position_local(left_target, tcp_control_offset)
                right_tcp_target = offset_target_position_local(right_target, tcp_control_offset)
                solver.set_targets(left_target, right_target)
                last_left_target = left_target.copy()
                last_right_target = right_target.copy()

                result = solver.step(dt)
                delta_q = solver.configuration.q - last_arm_q
                joint_motion_stats.update(delta_q)
                last_arm_q = solver.configuration.q.copy()
                if step_index % args.display_every == 0:
                    visualizer.display(
                        result.q,
                        TargetPair(solver.left_target, solver.right_target),
                        show_targets=True,
                    )
                    if args.show_vr_debug and last_debug_packet is not None:
                        update_vr_debug_markers(
                            visualizer,
                            last_debug_packet,
                            left_transformer,
                            right_transformer,
                            left_mapped_tcp_target,
                            right_mapped_tcp_target,
                            left_tcp_target,
                            right_tcp_target,
                            vr_debug_origin,
                            args.vr_debug_scale,
                            args.vr_debug_target_forward_offset,
                            args.vr_debug_target_offset_axis,
                        )

                if (
                    packet_count
                    and packet_count != last_print_packet_count
                    and packet_count % max(1, args.print_every) == 0
                ):
                    last_print_packet_count = packet_count
                    log(
                        "[vr_visual_demo] "
                        f"packets={packet_count} dropped_old={dropped_packet_count} bad={bad_count} "
                        f"left_err={result.left_position_error:.4f} "
                        f"right_err={result.right_position_error:.4f} "
                        f"{wrist_motion_summary(solver)} "
                        f"{joint_motion_stats.summary(dt)}"
                    )

                sleep_until = start + (step_index + 1) * dt
                time.sleep(max(0.0, sleep_until - time.monotonic()))
                if time.monotonic() - loop_start > 2.0 * dt:
                    pass
    except KeyboardInterrupt:
        log("[vr_visual_demo] interrupted")

    log(
        "[vr_visual_demo] stopped: "
        f"packets={packet_count}, dropped_old_packets={dropped_packet_count}, bad_packets={bad_count}, "
        f"final_left_error={result.left_position_error:.6f}, "
        f"final_right_error={result.right_position_error:.6f}, "
        f"{joint_motion_stats.summary(dt)}"
    )
    log_handle.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
