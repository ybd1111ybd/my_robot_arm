"""Meshcat visualization for the light_teleop kinematics model.

The first visualizer intentionally works without URDF mesh assets. It draws the
left and right arm kinematic chains, current end-effector frames, and target
frames. This keeps M3 usable while the gripper STL files are still missing.
"""

from __future__ import annotations

import argparse
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree

import meshcat
import meshcat_shapes
import numpy as np
import pinocchio as pin
from meshcat import geometry as g

from .model_loader import (
    LEFT_JOINT_NAMES,
    PACKAGE_NAME,
    RIGHT_JOINT_NAMES,
    default_package_dir,
    default_urdf_path,
    load_and_validate_model,
    resolve_path,
)
from .pink_solver import HOME_NEUTRAL, HOME_PREGRASP, HOME_VR_PREGRASP, make_home_configuration


LEFT_CHAIN_FRAMES = [
    "body_link3",
    "left_arm_link1",
    "left_arm_link2",
    "left_arm_link3",
    "left_arm_link4",
    "left_arm_link5",
    "left_arm_link6",
    "left_arm_link7",
    "left_arm_link8",
    "left_arm_link9",
    "left_arm_link10",
]

RIGHT_CHAIN_FRAMES = [
    "body_link3",
    "right_arm_link1",
    "right_arm_link2",
    "right_arm_link3",
    "right_arm_link4",
    "right_arm_link5",
    "right_arm_link6",
    "right_arm_link7",
    "right_arm_link8",
    "right_arm_link9",
    "right_arm_link10",
]


@dataclass(frozen=True)
class TargetPair:
    """Left and right target transforms in the world frame."""

    left: pin.SE3
    right: pin.SE3


@dataclass(frozen=True)
class VisualMesh:
    """One visual mesh attached to a robot link."""

    link_name: str
    mesh_path: Path
    origin: pin.SE3
    color: int
    meshcat_path: str


class TeleopVisualizer:
    """Display arm kinematic chains and frame targets in Meshcat."""

    def __init__(
        self,
        model: pin.Model,
        data: pin.Data,
        left_ee_frame: str = "left_arm_link9",
        right_ee_frame: str = "right_arm_link9",
        visual_meshes: list[VisualMesh] | None = None,
        open_browser: bool = False,
    ) -> None:
        self.model = model
        self.data = data
        self.left_ee_frame = left_ee_frame
        self.right_ee_frame = right_ee_frame
        self.visual_meshes = visual_meshes or []
        self.viewer = meshcat.Visualizer()
        if open_browser:
            self.viewer.open()
        self.viewer.delete()
        self._setup_scene()
        self._load_visual_meshes()

    def _setup_scene(self) -> None:
        """Create static scene helpers."""

        self.viewer["/Background"].set_property("top_color", [0.96, 0.97, 0.98])
        self.viewer["/Background"].set_property("bottom_color", [0.86, 0.88, 0.9])
        meshcat_shapes.frame(self.viewer["world"], axis_length=0.2, axis_thickness=0.006)

        self.viewer["grid"].set_object(
            g.LineSegments(
                g.PointsGeometry(make_grid_points(size=2.0, divisions=20)),
                g.LineBasicMaterial(color=0xB8C2CC, linewidth=1),
            )
        )

        self._set_chain_materials()
        meshcat_shapes.frame(self.viewer["left/current"], axis_length=0.12, axis_thickness=0.004)
        meshcat_shapes.frame(self.viewer["right/current"], axis_length=0.12, axis_thickness=0.004)
        meshcat_shapes.frame(self.viewer["left/target"], axis_length=0.16, axis_thickness=0.005)
        meshcat_shapes.frame(self.viewer["right/target"], axis_length=0.16, axis_thickness=0.005)
        self.viewer["left/target_point"].set_object(
            g.Sphere(0.018),
            g.MeshLambertMaterial(color=0x1E88E5, opacity=0.85),
        )
        self.viewer["right/target_point"].set_object(
            g.Sphere(0.018),
            g.MeshLambertMaterial(color=0xF57C00, opacity=0.85),
        )
        self.viewer["joint_axes/left"].set_property("visible", False)
        self.viewer["joint_axes/right"].set_property("visible", False)
        self.viewer["joint_axes/selected"].set_property("visible", False)

    def _set_chain_materials(self) -> None:
        """Initialize chain point and line objects."""

        self.viewer["left/chain"].set_object(
            g.Line(
                g.PointsGeometry(np.zeros((3, len(LEFT_CHAIN_FRAMES)))),
                g.LineBasicMaterial(color=0x2B6CB0, linewidth=4),
            )
        )
        self.viewer["right/chain"].set_object(
            g.Line(
                g.PointsGeometry(np.zeros((3, len(RIGHT_CHAIN_FRAMES)))),
                g.LineBasicMaterial(color=0xC05621, linewidth=4),
            )
        )
        self.viewer["left/joints"].set_object(
            g.Points(
                g.PointsGeometry(np.zeros((3, len(LEFT_CHAIN_FRAMES)))),
                g.PointsMaterial(color=0x1A365D, size=0.035),
            )
        )
        self.viewer["right/joints"].set_object(
            g.Points(
                g.PointsGeometry(np.zeros((3, len(RIGHT_CHAIN_FRAMES)))),
                g.PointsMaterial(color=0x7B341E, size=0.035),
            )
        )

    def display(
        self,
        q: np.ndarray,
        targets: TargetPair | None = None,
        show_targets: bool = True,
        show_joint_axes: bool = False,
    ) -> None:
        """Display a robot configuration and optional target frames."""

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        left_points = self._frame_positions(LEFT_CHAIN_FRAMES)
        right_points = self._frame_positions(RIGHT_CHAIN_FRAMES)
        self._display_points("left", left_points)
        self._display_points("right", right_points)

        left_pose = self._frame_transform(self.left_ee_frame)
        right_pose = self._frame_transform(self.right_ee_frame)
        self.viewer["left/current"].set_transform(left_pose.homogeneous)
        self.viewer["right/current"].set_transform(right_pose.homogeneous)

        if targets is None:
            targets = TargetPair(left_pose.copy(), right_pose.copy())
        if show_targets:
            self.viewer["left/target"].set_property("visible", True)
            self.viewer["right/target"].set_property("visible", True)
            self.viewer["left/target"].set_transform(targets.left.homogeneous)
            self.viewer["right/target"].set_transform(targets.right.homogeneous)
            self.viewer["left/target_point"].set_property("visible", True)
            self.viewer["right/target_point"].set_property("visible", True)
            self.viewer["left/target_point"].set_transform(targets.left.homogeneous)
            self.viewer["right/target_point"].set_transform(targets.right.homogeneous)
        else:
            self.viewer["left/target"].set_property("visible", False)
            self.viewer["right/target"].set_property("visible", False)
            self.viewer["left/target_point"].set_property("visible", False)
            self.viewer["right/target_point"].set_property("visible", False)

        self._display_visual_meshes()
        if show_joint_axes:
            self.display_joint_axes()
        else:
            self.viewer["joint_axes/left"].set_property("visible", False)
            self.viewer["joint_axes/right"].set_property("visible", False)
            self.viewer["joint_axes/selected"].set_property("visible", False)

    def display_joint_axes(self) -> None:
        """Display world-frame revolute joint axes for left and right arms."""

        self.viewer["joint_axes/left"].set_property("visible", True)
        self.viewer["joint_axes/right"].set_property("visible", True)
        self.viewer["joint_axes/left"].set_object(
            g.LineSegments(
                g.PointsGeometry(joint_axis_segments(self.model, self.data, LEFT_JOINT_NAMES, 0.12)),
                g.LineBasicMaterial(color=0x00A6FB, linewidth=5),
            )
        )
        self.viewer["joint_axes/right"].set_object(
            g.LineSegments(
                g.PointsGeometry(joint_axis_segments(self.model, self.data, RIGHT_JOINT_NAMES, 0.12)),
                g.LineBasicMaterial(color=0xF97316, linewidth=5),
            )
        )

    def display_selected_link_axis(self, left_frame: str, right_frame: str) -> None:
        """Highlight the parent joint axis for the selected left/right frames."""

        segments = []
        for frame_name in (left_frame, right_frame):
            segment = parent_joint_axis_segment(self.model, self.data, frame_name, length=0.18)
            if segment is not None:
                segments.extend(segment)
        if not segments:
            self.viewer["joint_axes/selected"].set_property("visible", False)
            return
        self.viewer["joint_axes/selected"].set_property("visible", True)
        self.viewer["joint_axes/selected"].set_object(
            g.LineSegments(
                g.PointsGeometry(np.array(segments).T),
                g.LineBasicMaterial(color=0xE11D48, linewidth=8),
            )
        )

    def _load_visual_meshes(self) -> None:
        """Load available STL visual meshes into Meshcat once."""

        for visual in self.visual_meshes:
            try:
                mesh = g.StlMeshGeometry.from_file(str(visual.mesh_path))
                material = g.MeshLambertMaterial(color=visual.color, opacity=0.88)
                self.viewer[visual.meshcat_path].set_object(mesh, material)
            except Exception as exc:  # noqa: BLE001 - keep visualization best-effort.
                print(f"[visualizer] warning: failed to load mesh {visual.mesh_path}: {exc}")

    def _display_visual_meshes(self) -> None:
        """Update mesh transforms from current frame placements."""

        for visual in self.visual_meshes:
            if not self.model.existFrame(visual.link_name):
                continue
            transform = self._frame_transform(visual.link_name) * visual.origin
            self.viewer[visual.meshcat_path].set_transform(transform.homogeneous)

    def _display_points(self, side: str, points: np.ndarray) -> None:
        """Update line and point geometry for one arm."""

        self.viewer[f"{side}/chain"].set_object(
            g.Line(
                g.PointsGeometry(points),
                g.LineBasicMaterial(color=0x2B6CB0 if side == "left" else 0xC05621, linewidth=4),
            )
        )
        self.viewer[f"{side}/joints"].set_object(
            g.Points(
                g.PointsGeometry(points),
                g.PointsMaterial(color=0x1A365D if side == "left" else 0x7B341E, size=0.035),
            )
        )

    def _frame_positions(self, frame_names: Iterable[str]) -> np.ndarray:
        """Return frame translations as a Meshcat PointsGeometry matrix."""

        points = []
        for frame_name in frame_names:
            if self.model.existFrame(frame_name):
                points.append(self._frame_transform(frame_name).translation)
        if not points:
            return np.zeros((3, 0))
        return np.array(points).T

    def _frame_transform(self, frame_name: str) -> pin.SE3:
        """Return a frame transform in world coordinates."""

        frame_id = self.model.getFrameId(frame_name)
        return self.data.oMf[frame_id]

    def url(self) -> str:
        """Return Meshcat's browser URL."""

        return self.viewer.url()


def make_demo_targets(model: pin.Model, data: pin.Data, q: np.ndarray, phase: float) -> TargetPair:
    """Create small moving target frames around the current end-effectors."""

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    left = data.oMf[model.getFrameId("left_arm_link9")].copy()
    right = data.oMf[model.getFrameId("right_arm_link9")].copy()
    offset = np.array([0.04 * np.sin(phase), 0.0, 0.04 * np.cos(phase)])
    left.translation = left.translation + offset
    right.translation = right.translation + np.array([offset[0], 0.0, -offset[2]])
    return TargetPair(left=left, right=right)


def parse_visual_meshes(
    urdf_path: Path,
    package_dir: Path,
    show_meshes: bool,
) -> tuple[list[VisualMesh], list[str]]:
    """Parse URDF visual STL meshes and return available assets."""

    if not show_meshes:
        return [], []

    root = ElementTree.parse(urdf_path).getroot()
    visual_meshes: list[VisualMesh] = []
    missing: list[str] = []
    visual_index = 0

    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.attrib.get("filename", "")
            relative = package_mesh_relative_path(filename)
            if relative is None:
                continue

            mesh_path = package_dir / relative
            if not mesh_path.exists():
                missing.append(relative)
                continue

            origin = parse_origin(visual.find("origin"))
            visual_meshes.append(
                VisualMesh(
                    link_name=link_name,
                    mesh_path=mesh_path,
                    origin=origin,
                    color=color_for_link(link_name),
                    meshcat_path=f"robot/{visual_index:03d}_{sanitize_path(link_name)}",
                )
            )
            visual_index += 1

    return visual_meshes, sorted(set(missing))


def package_mesh_relative_path(filename: str) -> str | None:
    """Convert package:// mesh URL to a path relative to package_dir."""

    prefix = f"package://{PACKAGE_NAME}/"
    if filename.startswith(prefix):
        return filename[len(prefix) :]
    return None


def parse_origin(origin: ElementTree.Element | None) -> pin.SE3:
    """Parse a URDF origin element as a Pinocchio SE3."""

    if origin is None:
        return pin.SE3.Identity()

    xyz_text = origin.attrib.get("xyz", "0 0 0")
    rpy_text = origin.attrib.get("rpy", "0 0 0")
    xyz = np.array([float(value) for value in xyz_text.split()])
    rpy = np.array([float(value) for value in rpy_text.split()])
    return pin.SE3(pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2]), xyz)


def sanitize_path(name: str) -> str:
    """Return a Meshcat path-safe version of a frame or link name."""

    return re.sub(r"[^A-Za-z0-9_]+", "_", name)


def color_for_link(link_name: str) -> int:
    """Choose subdued colors by robot area."""

    if link_name.startswith("left_"):
        return 0x4A90E2
    if link_name.startswith("right_"):
        return 0xD9822B
    if "head" in link_name:
        return 0x7B8794
    return 0x9AA5B1


def make_grid_points(size: float, divisions: int) -> np.ndarray:
    """Create line-segment points for a simple XY ground grid."""

    half = size / 2.0
    values = np.linspace(-half, half, divisions + 1)
    points: list[np.ndarray] = []
    for value in values:
        points.append(np.array([-half, value, 0.0]))
        points.append(np.array([half, value, 0.0]))
        points.append(np.array([value, -half, 0.0]))
        points.append(np.array([value, half, 0.0]))
    return np.array(points).T


def joint_axis_segments(
    model: pin.Model,
    data: pin.Data,
    joint_names: Iterable[str],
    length: float,
) -> np.ndarray:
    """Return line segments showing joint rotation axes in world frame."""

    points: list[np.ndarray] = []
    for joint_name in joint_names:
        segment = joint_axis_segment(model, data, joint_name, length)
        if segment is not None:
            points.extend(segment)
    if not points:
        return np.zeros((3, 0))
    return np.array(points).T


def joint_axis_segment(
    model: pin.Model,
    data: pin.Data,
    joint_name: str,
    length: float,
) -> list[np.ndarray] | None:
    """Return one centered world-frame segment for a revolute joint axis."""

    if not model.existJointName(joint_name):
        return None
    joint_id = model.getJointId(joint_name)
    if joint_id >= len(data.oMi):
        return None
    axis_local = joint_axis_local(model, joint_id)
    if axis_local is None:
        return None
    joint_pose = data.oMi[joint_id]
    axis_world = joint_pose.rotation @ axis_local
    norm = float(np.linalg.norm(axis_world))
    if norm < 1e-12:
        return None
    axis_world = axis_world / norm
    center = joint_pose.translation
    half = 0.5 * length * axis_world
    return [center - half, center + half]


def parent_joint_axis_segment(
    model: pin.Model,
    data: pin.Data,
    frame_name: str,
    length: float,
) -> list[np.ndarray] | None:
    """Return the axis segment of the joint that owns a frame."""

    if not model.existFrame(frame_name):
        return None
    frame = model.frames[model.getFrameId(frame_name)]
    joint_id = frame.parentJoint
    if joint_id <= 0:
        return None
    return joint_axis_segment(model, data, model.names[joint_id], length)


def joint_axis_local(model: pin.Model, joint_id: int) -> np.ndarray | None:
    """Infer the local axis of a 1-DoF revolute joint from its shortname."""

    shortname = model.joints[joint_id].shortname()
    if "RX" in shortname:
        return np.array([1.0, 0.0, 0.0])
    if "RY" in shortname:
        return np.array([0.0, 1.0, 0.0])
    if "RZ" in shortname:
        return np.array([0.0, 0.0, 1.0])
    if model.nvs[joint_id] == 1:
        # Fallback for generic revolute joints in this URDF: Pinocchio stores
        # them as axis-specific RX/RY/RZ in practice, so reaching here means the
        # joint is not a simple revolute axis we can draw confidently.
        return None
    return None


def apply_demo_motion(model: pin.Model, q: np.ndarray, phase: float) -> np.ndarray:
    """Move only the 14 arm joints with small sinusoidal offsets."""

    q_demo = q.copy()
    for index, joint_name in enumerate(LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES):
        joint_id = model.getJointId(joint_name)
        q_index = model.idx_qs[joint_id]
        amplitude = 0.12 if index % 2 == 0 else 0.08
        q_demo[q_index] += amplitude * np.sin(phase + index * 0.35)
    return q_demo


def selected_parent_joint_names(model: pin.Model, left_frame: str, right_frame: str) -> list[str]:
    """Return parent joint names for selected frames, skipping duplicates."""

    names: list[str] = []
    for frame_name in (left_frame, right_frame):
        if not model.existFrame(frame_name):
            continue
        frame = model.frames[model.getFrameId(frame_name)]
        joint_id = frame.parentJoint
        if joint_id > 0:
            joint_name = model.names[joint_id]
            if joint_name not in names:
                names.append(joint_name)
    return names


def apply_joint_sweep(
    model: pin.Model,
    q: np.ndarray,
    joint_names: Iterable[str],
    phase: float,
    amplitude: float,
) -> np.ndarray:
    """Move selected 1-DoF joints sinusoidally around q."""

    q_demo = q.copy()
    offset = amplitude * np.sin(phase)
    for joint_name in joint_names:
        if not model.existJointName(joint_name):
            continue
        joint_id = model.getJointId(joint_name)
        if model.nqs[joint_id] != 1:
            continue
        q_index = model.idx_qs[joint_id]
        lower = model.lowerPositionLimit[q_index]
        upper = model.upperPositionLimit[q_index]
        q_demo[q_index] = float(np.clip(q[q_index] + offset, lower, upper))
    return q_demo


def build_arg_parser() -> argparse.ArgumentParser:
    """Create command-line parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", default=str(default_urdf_path()))
    parser.add_argument("--package-dir", default=str(default_package_dir()))
    parser.add_argument("--left-ee-frame", default="left_arm_link9")
    parser.add_argument("--right-ee-frame", default="right_arm_link9")
    parser.add_argument(
        "--home",
        choices=[HOME_NEUTRAL, HOME_PREGRASP, HOME_VR_PREGRASP],
        default=HOME_NEUTRAL,
        help="Robot posture used for static and demo visualization.",
    )
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--frequency", type=float, default=30.0)
    parser.add_argument("--open-browser", action="store_true")
    parser.add_argument(
        "--no-meshes",
        action="store_true",
        help="Only draw kinematic skeleton and frames.",
    )
    parser.add_argument(
        "--arm-meshes-only",
        action="store_true",
        help="Load only left/right arm visual meshes.",
    )
    parser.add_argument(
        "--static",
        action="store_true",
        help="Display one frame and keep Meshcat alive until Ctrl+C.",
    )
    parser.add_argument(
        "--hide-targets",
        action="store_true",
        help="Hide target frames and show only the selected current EE frames.",
    )
    parser.add_argument(
        "--show-joint-axes",
        action="store_true",
        help="Draw revolute joint axes for left/right arm joints.",
    )
    parser.add_argument(
        "--sweep-selected-joint",
        action="store_true",
        help="Animate the parent joint of --left-ee-frame and --right-ee-frame.",
    )
    parser.add_argument(
        "--sweep-joint",
        action="append",
        default=[],
        help="Joint name to animate. Can be repeated. Overrides --sweep-selected-joint targets when provided.",
    )
    parser.add_argument(
        "--sweep-amplitude",
        type=float,
        default=0.35,
        help="Joint sweep amplitude in radians.",
    )
    parser.add_argument(
        "--sweep-frequency",
        type=float,
        default=0.25,
        help="Joint sweep frequency in Hz.",
    )
    parser.add_argument(
        "--highlight-selected-axis",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Highlight the parent joint axis of --left-ee-frame and --right-ee-frame.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the Meshcat kinematics visualizer demo."""

    args = build_arg_parser().parse_args(argv)
    urdf_path = resolve_path(args.urdf)
    package_dir = resolve_path(args.package_dir)

    result = load_and_validate_model(urdf_path, package_dir, load_geometry=False)
    visual_meshes, missing_visual_meshes = parse_visual_meshes(
        urdf_path,
        package_dir,
        show_meshes=not args.no_meshes,
    )
    if args.arm_meshes_only:
        visual_meshes = [
            mesh
            for mesh in visual_meshes
            if mesh.link_name.startswith("left_arm_") or mesh.link_name.startswith("right_arm_")
        ]
    visualizer = TeleopVisualizer(
        result.robot.model,
        result.robot.data,
        left_ee_frame=args.left_ee_frame,
        right_ee_frame=args.right_ee_frame,
        visual_meshes=visual_meshes,
        open_browser=args.open_browser,
    )

    print(f"[visualizer] Meshcat URL: {visualizer.url()}")
    if args.no_meshes:
        print("[visualizer] mode: kinematics skeleton, mesh assets disabled")
    else:
        print(f"[visualizer] mode: visual meshes + kinematics skeleton ({len(visual_meshes)} meshes loaded)")
    if missing_visual_meshes:
        print(f"[visualizer] warning: {len(missing_visual_meshes)} visual mesh assets are missing")
        for path in missing_visual_meshes:
            print(f"  {path}")

    q0 = make_home_configuration(result.robot.model, result.configuration.q, args.home)
    dt = 1.0 / args.frequency
    start = time.monotonic()
    sweep_joint_names = list(args.sweep_joint)
    if args.sweep_selected_joint and not sweep_joint_names:
        sweep_joint_names = selected_parent_joint_names(
            result.robot.model,
            args.left_ee_frame,
            args.right_ee_frame,
        )

    try:
        if args.static:
            q_static = q0
            if sweep_joint_names:
                q_static = apply_joint_sweep(
                    result.robot.model,
                    q0,
                    sweep_joint_names,
                    phase=np.pi / 2.0,
                    amplitude=args.sweep_amplitude,
                )
            visualizer.display(q_static, show_targets=not args.hide_targets, show_joint_axes=args.show_joint_axes)
            if args.highlight_selected_axis:
                visualizer.display_selected_link_axis(args.left_ee_frame, args.right_ee_frame)
            print("[visualizer] static scene ready; current and target frames overlap unless --hide-targets is used")
            print(f"[visualizer] home: {args.home}")
            if sweep_joint_names:
                print(f"[visualizer] static sweep offset applied to: {', '.join(sweep_joint_names)}")
            if args.show_joint_axes:
                print("[visualizer] joint axes: left=blue, right=orange")
            if args.highlight_selected_axis:
                print("[visualizer] selected frame parent joint axis: red")
            print("[visualizer] press Ctrl+C to exit")
            while True:
                time.sleep(1.0)

        while time.monotonic() - start < args.duration:
            phase = time.monotonic() - start
            if sweep_joint_names:
                q_demo = apply_joint_sweep(
                    result.robot.model,
                    q0,
                    sweep_joint_names,
                    phase=2.0 * np.pi * args.sweep_frequency * phase,
                    amplitude=args.sweep_amplitude,
                )
            else:
                q_demo = apply_demo_motion(result.robot.model, q0, phase)
            targets = make_demo_targets(result.robot.model, result.robot.data, q_demo, phase)
            visualizer.display(
                q_demo,
                targets,
                show_targets=not args.hide_targets,
                show_joint_axes=args.show_joint_axes,
            )
            if args.highlight_selected_axis:
                visualizer.display_selected_link_axis(args.left_ee_frame, args.right_ee_frame)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("[visualizer] interrupted")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
