"""Load and validate the JZ robot model for light_teleop.

This module is intentionally focused on the first milestone: kinematics-only
loading, joint/frame validation, initial configuration checks, and mesh asset
reporting. Geometry loading is available as an explicit check because the
current URDF can reference mesh files that are not present in the repository.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pinocchio as pin
import pink


PACKAGE_NAME = "jz_robot_description"

LEFT_JOINT_NAMES = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
]

RIGHT_JOINT_NAMES = [
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_joint7",
]

ARM_JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES

CANDIDATE_EE_FRAMES = [
    "left_arm_link9",
    "right_arm_link9",
    "left_arm_link10",
    "right_arm_link10",
]


@dataclass(frozen=True)
class MeshReport:
    """Summary of package:// mesh references in a URDF."""

    references: list[str]
    existing: list[str]
    missing: list[str]


@dataclass(frozen=True)
class JointInfo:
    """Pinocchio index information for one joint."""

    name: str
    joint_id: int
    idx_q: int
    nq: int
    idx_v: int
    nv: int


@dataclass(frozen=True)
class FrameInfo:
    """Pinocchio index and pose information for one frame."""

    name: str
    frame_id: int
    xyz: np.ndarray
    quat_xyzw: np.ndarray


@dataclass(frozen=True)
class ModelLoadResult:
    """Loaded robot model plus validation metadata."""

    robot: pin.RobotWrapper
    configuration: pink.Configuration
    arm_joints: list[JointInfo]
    non_arm_actuated_joints: list[str]
    candidate_frames: list[FrameInfo]
    mesh_report: MeshReport


def default_project_root() -> Path:
    """Return the repository root inferred from this file location."""

    return Path(__file__).resolve().parents[2]


def default_urdf_path(project_root: Path | None = None) -> Path:
    """Return the default JZ URDF path in the current repository layout."""

    root = project_root or default_project_root()
    return root / "jz_descripetion-main/robot_urdf/urdf/robot urdf.10.8.SLDASM.urdf"


def default_package_dir(project_root: Path | None = None) -> Path:
    """Return the default ROS package directory for jz_robot_description."""

    root = project_root or default_project_root()
    return root / "jz_descripetion-main/robot_urdf"


def resolve_path(path: str | Path, project_root: Path | None = None) -> Path:
    """Resolve a path relative to the project root when it is not absolute."""

    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return candidate
    return (project_root or default_project_root()) / candidate


def parse_package_xml_name(package_dir: Path) -> str | None:
    """Read the package name from package.xml when available."""

    package_xml = package_dir / "package.xml"
    if not package_xml.exists():
        return None
    match = re.search(r"<name>\s*([^<]+?)\s*</name>", package_xml.read_text())
    return match.group(1) if match else None


def scan_mesh_references(urdf_path: Path, package_dir: Path) -> MeshReport:
    """Find package:// mesh references and check whether files exist."""

    text = urdf_path.read_text()
    pattern = rf"package://{re.escape(PACKAGE_NAME)}/([^\"'\s<>]+)"
    references = sorted(set(re.findall(pattern, text)))

    existing: list[str] = []
    missing: list[str] = []
    for reference in references:
        if (package_dir / reference).exists():
            existing.append(reference)
        else:
            missing.append(reference)

    return MeshReport(references=references, existing=existing, missing=missing)


def load_robot_model(
    urdf_path: Path,
    package_dir: Path,
    load_geometry: bool = False,
) -> pin.RobotWrapper:
    """Load the robot with Pinocchio.

    Kinematics-only mode uses :meth:`RobotWrapper.BuildFromURDF` without package
    directories so missing mesh files do not block IK development. Geometry mode
    passes package directories explicitly and should only be used once mesh
    assets are complete.
    """

    if load_geometry:
        return pin.RobotWrapper.BuildFromURDF(
            filename=str(urdf_path),
            package_dirs=[str(package_dir.parent)],
            root_joint=None,
        )
    model = pin.buildModelFromUrdf(str(urdf_path))
    return pin.RobotWrapper(model)


def make_initial_configuration(model: pin.Model) -> np.ndarray:
    """Create an initial configuration and keep it inside model limits."""

    q0 = pin.neutral(model)
    q_min = model.lowerPositionLimit
    q_max = model.upperPositionLimit

    for index in range(model.nq):
        lower = q_min[index]
        upper = q_max[index]
        if math.isfinite(lower) and math.isfinite(upper) and lower > upper:
            raise ValueError(f"invalid configuration limit at q[{index}]: {lower} > {upper}")
        if math.isfinite(lower) and q0[index] < lower:
            q0[index] = lower
        if math.isfinite(upper) and q0[index] > upper:
            q0[index] = upper

    return q0


def get_joint_info(model: pin.Model, joint_names: Iterable[str]) -> list[JointInfo]:
    """Return joint indices for required joints, raising on missing names."""

    infos: list[JointInfo] = []
    missing: list[str] = []
    for name in joint_names:
        if not model.existJointName(name):
            missing.append(name)
            continue
        joint_id = model.getJointId(name)
        infos.append(
            JointInfo(
                name=name,
                joint_id=joint_id,
                idx_q=model.idx_qs[joint_id],
                nq=model.nqs[joint_id],
                idx_v=model.idx_vs[joint_id],
                nv=model.nvs[joint_id],
            )
        )

    if missing:
        raise ValueError("missing required arm joints: " + ", ".join(missing))

    return infos


def list_non_arm_actuated_joints(model: pin.Model) -> list[str]:
    """List movable joints that are not part of the 14 controlled arm joints."""

    arm_set = set(ARM_JOINT_NAMES)
    non_arm: list[str] = []
    for joint_id, name in enumerate(model.names):
        if joint_id == 0:
            continue
        if model.nvs[joint_id] > 0 and name not in arm_set:
            non_arm.append(name)
    return non_arm


def get_candidate_frames(
    configuration: pink.Configuration,
    frame_names: Iterable[str],
) -> list[FrameInfo]:
    """Return frame ids and current poses for candidate end-effector frames."""

    model = configuration.model
    frames: list[FrameInfo] = []
    missing: list[str] = []
    for name in frame_names:
        if not model.existFrame(name):
            missing.append(name)
            continue
        frame_id = model.getFrameId(name)
        transform = configuration.get_transform_frame_to_world(name)
        quat = pin.Quaternion(transform.rotation)
        frames.append(
            FrameInfo(
                name=name,
                frame_id=frame_id,
                xyz=transform.translation.copy(),
                quat_xyzw=np.array([quat.x, quat.y, quat.z, quat.w]),
            )
        )

    required = {"left_arm_link9", "right_arm_link9"}
    missing_required = sorted(required.intersection(missing))
    if missing_required:
        raise ValueError("missing required candidate EE frames: " + ", ".join(missing_required))

    return frames


def validate_q0(configuration: pink.Configuration) -> None:
    """Validate the initial Pink configuration."""

    q = configuration.q
    if q.shape != (configuration.model.nq,):
        raise ValueError(f"q0 shape mismatch: got {q.shape}, expected {(configuration.model.nq,)}")
    if not np.all(np.isfinite(q)):
        raise ValueError("q0 contains NaN or Inf")
    configuration.check_limits()


def load_and_validate_model(
    urdf_path: Path,
    package_dir: Path,
    load_geometry: bool = False,
) -> ModelLoadResult:
    """Load the robot and run model_loader milestone checks."""

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")
    if not package_dir.exists():
        raise FileNotFoundError(f"package directory not found: {package_dir}")

    mesh_report = scan_mesh_references(urdf_path, package_dir)
    if load_geometry and mesh_report.missing:
        missing = "\n  ".join(mesh_report.missing)
        raise FileNotFoundError(
            "full-geometry mode requires all referenced meshes; missing:\n  " + missing
        )

    robot = load_robot_model(urdf_path, package_dir, load_geometry=load_geometry)
    q0 = make_initial_configuration(robot.model)
    configuration = pink.Configuration(robot.model, robot.data, q0)
    validate_q0(configuration)

    arm_joints = get_joint_info(robot.model, ARM_JOINT_NAMES)
    non_arm_actuated_joints = list_non_arm_actuated_joints(robot.model)
    candidate_frames = get_candidate_frames(configuration, CANDIDATE_EE_FRAMES)

    return ModelLoadResult(
        robot=robot,
        configuration=configuration,
        arm_joints=arm_joints,
        non_arm_actuated_joints=non_arm_actuated_joints,
        candidate_frames=candidate_frames,
        mesh_report=mesh_report,
    )


def format_vector(values: np.ndarray) -> str:
    """Format a vector compactly for command-line diagnostics."""

    return "[" + ", ".join(f"{value:.6g}" for value in values.tolist()) + "]"


def print_report(
    result: ModelLoadResult,
    urdf_path: Path,
    package_dir: Path,
    load_geometry: bool,
) -> None:
    """Print a human-readable acceptance report."""

    package_xml_name = parse_package_xml_name(package_dir) or "unknown"
    model = result.robot.model

    print(f"[model_loader] URDF: {urdf_path}")
    print(f"[model_loader] package name: {package_xml_name}")
    print(f"[model_loader] expected package name: {PACKAGE_NAME}")
    print(f"[model_loader] package dir: {package_dir}")
    print(f"[model_loader] mode: {'full-geometry' if load_geometry else 'kinematics-only'}")
    print()

    print("[model_loader] Pinocchio model loaded")
    print(f"[model_loader] nq = {model.nq}")
    print(f"[model_loader] nv = {model.nv}")
    print(f"[model_loader] joints = {len(model.names) - 1}")
    print(f"[model_loader] frames = {len(model.frames)}")
    print()

    print(f"[model_loader] arm joints found: {len(result.arm_joints)} / {len(ARM_JOINT_NAMES)}")
    for info in result.arm_joints:
        print(
            "  "
            f"{info.name}: joint_id={info.joint_id}, "
            f"idx_q={info.idx_q}, nq={info.nq}, "
            f"idx_v={info.idx_v}, nv={info.nv}"
        )
    print()

    print("[model_loader] candidate EE frames:")
    for info in result.candidate_frames:
        print(
            "  "
            f"{info.name}: frame_id={info.frame_id}, "
            f"xyz={format_vector(info.xyz)}, "
            f"quat_xyzw={format_vector(info.quat_xyzw)}"
        )
    print()

    print(f"[model_loader] non-arm actuated joints: {len(result.non_arm_actuated_joints)}")
    for name in result.non_arm_actuated_joints:
        print(f"  {name}")
    print()

    print("[model_loader] q0 check: OK")
    print("[model_loader] pink.Configuration check: OK")
    print()

    report = result.mesh_report
    print(f"[model_loader] mesh references: {len(report.references)}")
    print(f"[model_loader] existing meshes: {len(report.existing)}")
    print(f"[model_loader] missing meshes: {len(report.missing)}")
    for path in report.missing:
        print(f"  {path}")
    print()

    if report.missing and not load_geometry:
        print("[model_loader] geometry note: missing meshes reported; kinematics-only validation is still OK")
    elif report.missing:
        print("[model_loader] geometry note: full-geometry mode requested with missing meshes")
    else:
        print("[model_loader] geometry note: all referenced meshes exist")


def build_arg_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        type=str,
        default=str(default_urdf_path()),
        help="Path to the JZ URDF. Relative paths are resolved from the project root.",
    )
    parser.add_argument(
        "--package-dir",
        type=str,
        default=str(default_package_dir()),
        help="Path to the jz_robot_description package directory.",
    )
    parser.add_argument(
        "--load-geometry",
        action="store_true",
        help="Ask Pinocchio to load visual/collision geometry. Mesh files must be complete.",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Compatibility flag: run the default kinematics-only checks.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Command-line entry point."""

    parser = build_arg_parser()
    args = parser.parse_args(argv)
    project_root = default_project_root()
    urdf_path = resolve_path(args.urdf, project_root)
    package_dir = resolve_path(args.package_dir, project_root)

    try:
        result = load_and_validate_model(
            urdf_path=urdf_path,
            package_dir=package_dir,
            load_geometry=args.load_geometry,
        )
        print_report(result, urdf_path, package_dir, load_geometry=args.load_geometry)
    except Exception as exc:  # noqa: BLE001 - CLI must show a clear acceptance failure.
        print(f"[model_loader] ERROR: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
