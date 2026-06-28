"""VR controller pose to robot end-effector target transform."""

from __future__ import annotations

import math
from typing import Iterable, List

from .math_utils import (
    matrix_to_quat,
    quat_from_axis_angle,
    quat_inverse,
    quat_multiply,
    quat_normalize,
    quat_to_matrix,
    vec_add,
    vec_mul,
    vec_sub,
)
from .types import RobotEndEffectorPose
from .workspace_limiter import WorkspaceLimiter


class VrCoordinateTransformer:
    def __init__(self, scale_factor: float = 1.0) -> None:
        self.scale_factor = scale_factor
        self.axis_mapping = [1, 2, 3]
        self.tool_orientation_offset = [0.0, 0.0, 0.0, 1.0]
        self.workspace_limiter = WorkspaceLimiter()
        self.robot_home_position = [0.0, 0.0, 0.0]
        self.robot_home_orientation = [0.0, 0.0, 0.0, 1.0]
        self.robot_reference_position = [0.0, 0.0, 0.0]
        self.robot_reference_orientation = [0.0, 0.0, 0.0, 1.0]
        self.reference_locked = False
        self.vr_reference_position = [0.0, 0.0, 0.0]
        self.vr_reference_orientation = [0.0, 0.0, 0.0, 1.0]
        self.last_target_position = [0.0, 0.0, 0.0]
        self.last_target_orientation = [0.0, 0.0, 0.0, 1.0]

    @staticmethod
    def left_tool_offset() -> List[float]:
        return quat_from_axis_angle([0.0, 0.0, 1.0], math.pi / 2.0)

    @staticmethod
    def right_tool_offset() -> List[float]:
        return quat_from_axis_angle([0.0, 0.0, 1.0], -math.pi / 2.0)

    def set_robot_home_pose(self, position: Iterable[float], orientation: Iterable[float]) -> None:
        self.robot_home_position = list(position)
        self.robot_home_orientation = quat_normalize(orientation)
        self.robot_reference_position = list(self.robot_home_position)
        self.robot_reference_orientation = list(self.robot_home_orientation)
        self.last_target_position = list(self.robot_home_position)
        self.last_target_orientation = list(self.robot_home_orientation)

    def set_workspace_center(self, center: Iterable[float]) -> None:
        self.workspace_limiter.set_center(center)

    def set_workspace_limits(self, enable: bool, max_radius: float, boundary_type: str = "clamp") -> None:
        self.workspace_limiter.set_limits(enable, max_radius, boundary_type)

    def set_scale_factor(self, scale_factor: float) -> None:
        if math.isfinite(scale_factor) and scale_factor > 0.0:
            self.scale_factor = scale_factor

    def set_axis_mapping(self, mapping: Iterable[int]) -> None:
        values = list(mapping)
        if self.is_valid_axis_mapping(values):
            self.axis_mapping = values

    def set_tool_orientation_offset(self, offset: Iterable[float]) -> None:
        self.tool_orientation_offset = quat_normalize(offset)

    def lock_vr_reference(self, vr_position: Iterable[float], vr_orientation: Iterable[float]) -> None:
        self.vr_reference_position = list(vr_position)
        self.vr_reference_orientation = quat_normalize(vr_orientation)
        self.robot_reference_position = list(self.last_target_position)
        self.robot_reference_orientation = list(self.last_target_orientation)
        self.reference_locked = True

    def reset_vr_reference(self) -> None:
        self.reference_locked = False
        self.vr_reference_position = [0.0, 0.0, 0.0]
        self.vr_reference_orientation = [0.0, 0.0, 0.0, 1.0]

    def reset_to_home(self) -> None:
        self.reference_locked = False
        self.vr_reference_position = [0.0, 0.0, 0.0]
        self.vr_reference_orientation = [0.0, 0.0, 0.0, 1.0]
        self.robot_reference_position = list(self.robot_home_position)
        self.robot_reference_orientation = list(self.robot_home_orientation)
        self.last_target_position = list(self.robot_home_position)
        self.last_target_orientation = list(self.robot_home_orientation)

    def transform(self, vr_position: Iterable[float], vr_orientation: Iterable[float]) -> RobotEndEffectorPose:
        if not self.reference_locked:
            return RobotEndEffectorPose(
                position=list(self.last_target_position),
                orientation=list(self.last_target_orientation),
            )

        vr_delta = vec_sub(vr_position, self.vr_reference_position)
        mapped_delta = self.apply_axis_mapping(vr_delta)
        target_position = vec_add(
            self.robot_reference_position,
            vec_mul(mapped_delta, self.scale_factor),
        )
        target_position = self.workspace_limiter.apply(target_position)

        vr_current_orientation = quat_normalize(vr_orientation)
        vr_rotation_delta = quat_multiply(vr_current_orientation, quat_inverse(self.vr_reference_orientation))
        mapped_rotation_delta = self.apply_axis_mapping_to_quaternion(vr_rotation_delta)
        tool_delta = quat_multiply(
            quat_multiply(self.tool_orientation_offset, mapped_rotation_delta),
            quat_inverse(self.tool_orientation_offset),
        )
        target_orientation = quat_normalize(
            quat_multiply(self.robot_reference_orientation, tool_delta)
        )

        self.last_target_position = list(target_position)
        self.last_target_orientation = list(target_orientation)
        return RobotEndEffectorPose(position=target_position, orientation=target_orientation)

    @staticmethod
    def is_valid_axis_mapping(mapping: Iterable[int]) -> bool:
        values = list(mapping)
        if len(values) != 3:
            return False
        used = set()
        for axis in values:
            abs_axis = abs(axis)
            if abs_axis < 1 or abs_axis > 3 or abs_axis in used:
                return False
            used.add(abs_axis)
        return True

    def apply_axis_mapping(self, vector: Iterable[float]) -> List[float]:
        values = list(vector)
        result = [0.0, 0.0, 0.0]
        for source_index, target_axis in enumerate(self.axis_mapping):
            target_index = abs(target_axis) - 1
            sign = 1.0 if target_axis > 0 else -1.0
            result[target_index] = values[source_index] * sign
        return result

    def apply_axis_mapping_to_quaternion(self, quat: Iterable[float]) -> List[float]:
        matrix = quat_to_matrix(quat)
        mapped = [[0.0, 0.0, 0.0] for _ in range(3)]
        for i, target_axis in enumerate(self.axis_mapping):
            target_i = abs(target_axis) - 1
            sign_i = 1.0 if target_axis > 0 else -1.0
            for j, target_j_axis in enumerate(self.axis_mapping):
                target_j = abs(target_j_axis) - 1
                sign_j = 1.0 if target_j_axis > 0 else -1.0
                mapped[target_i][target_j] = matrix[i][j] * sign_i * sign_j
        return matrix_to_quat(mapped)
