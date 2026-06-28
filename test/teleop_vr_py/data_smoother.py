"""Smoothing filters for VR poses and controller inputs."""

from __future__ import annotations

import enum
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, List

from .math_utils import clamp, quat_dot, quat_normalize, quat_slerp


class SmootherType(enum.Enum):
    NONE = "none"
    MOVING_AVG = "moving_avg"
    EXP_MOVING_AVG = "exp_moving_avg"
    KALMAN = "kalman"


class ExponentialMovingAverage:
    def __init__(self, alpha: float = 0.3) -> None:
        self.alpha = clamp(alpha, 0.0, 1.0)
        self.initialized = False
        self.value = 0.0

    def filter(self, raw_value: float) -> float:
        if not self.initialized:
            self.value = raw_value
            self.initialized = True
        else:
            self.value = self.alpha * raw_value + (1.0 - self.alpha) * self.value
        return self.value

    def reset(self) -> None:
        self.initialized = False
        self.value = 0.0


class MovingAverage:
    def __init__(self, window_size: int = 5) -> None:
        self.window_size = max(1, int(window_size))
        self.buffer: Deque[float] = deque()
        self.total = 0.0

    def filter(self, raw_value: float) -> float:
        self.buffer.append(raw_value)
        self.total += raw_value
        if len(self.buffer) > self.window_size:
            self.total -= self.buffer.popleft()
        return self.total / len(self.buffer)

    def reset(self) -> None:
        self.buffer.clear()
        self.total = 0.0


class SimpleKalman1D:
    def __init__(self, process_noise: float = 1e-4, measurement_noise: float = 5e-3) -> None:
        self.q = max(process_noise, 1e-8)
        self.r = max(measurement_noise, 1e-8)
        self.initialized = False
        self.x = 0.0
        self.p = 1.0

    def filter(self, measurement: float) -> float:
        if not self.initialized:
            self.x = measurement
            self.initialized = True
            return self.x
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x = self.x + k * (measurement - self.x)
        self.p = (1.0 - k) * self.p
        return self.x

    def reset(self) -> None:
        self.initialized = False
        self.x = 0.0
        self.p = 1.0

    def set_from_alpha(self, alpha: float) -> None:
        alpha = clamp(alpha, 0.0, 1.0)
        self.q = max(1e-6, alpha * 1e-2)
        self.r = max(1e-6, (1.0 - alpha + 1e-3) * 1e-1)


class ScalarSmoother:
    def __init__(self, smoother_type: SmootherType, alpha: float) -> None:
        self.smoother_type = smoother_type
        self.alpha = clamp(alpha, 0.0, 1.0)
        self._build()

    @staticmethod
    def alpha_to_window_size(alpha: float) -> int:
        bounded = clamp(alpha, 0.01, 1.0)
        return int(clamp(round(1.0 / bounded + 1.0), 2, 30))

    def _build(self) -> None:
        self.ema = None
        self.moving_avg = None
        self.kalman = None
        if self.smoother_type == SmootherType.MOVING_AVG:
            self.moving_avg = MovingAverage(self.alpha_to_window_size(self.alpha))
        elif self.smoother_type == SmootherType.EXP_MOVING_AVG:
            self.ema = ExponentialMovingAverage(self.alpha)
        elif self.smoother_type == SmootherType.KALMAN:
            self.kalman = SimpleKalman1D()
            self.kalman.set_from_alpha(self.alpha)

    def filter(self, value: float) -> float:
        if self.smoother_type == SmootherType.NONE:
            return value
        if self.moving_avg is not None:
            return self.moving_avg.filter(value)
        if self.ema is not None:
            return self.ema.filter(value)
        if self.kalman is not None:
            return self.kalman.filter(value)
        return value

    def reset(self) -> None:
        for smoother in (self.ema, self.moving_avg, self.kalman):
            if smoother is not None:
                smoother.reset()


@dataclass
class QuaternionState:
    initialized: bool = False
    value: List[float] = None

    def __post_init__(self) -> None:
        if self.value is None:
            self.value = [0.0, 0.0, 0.0, 1.0]


class VrDataSmoother:
    def __init__(
        self,
        enabled: bool = True,
        smoother_type: SmootherType = SmootherType.EXP_MOVING_AVG,
        position_alpha: float = 0.3,
        rotation_alpha: float = 0.5,
        input_alpha: float = 0.2,
    ) -> None:
        self.enabled = enabled
        self.smoother_type = smoother_type
        self.position_alpha = clamp(position_alpha, 0.0, 1.0)
        self.rotation_alpha = clamp(rotation_alpha, 0.0, 1.0)
        self.input_alpha = clamp(input_alpha, 0.0, 1.0)
        self.left_position = self._scalar_group(3, self.position_alpha)
        self.right_position = self._scalar_group(3, self.position_alpha)
        self.headset_position = self._scalar_group(3, self.position_alpha)
        self.left_input = self._scalar_group(7, self.input_alpha)
        self.right_input = self._scalar_group(7, self.input_alpha)
        self.left_rotation = QuaternionState()
        self.right_rotation = QuaternionState()
        self.headset_rotation = QuaternionState()

    def _scalar_group(self, count: int, alpha: float) -> List[ScalarSmoother]:
        return [ScalarSmoother(self.smoother_type, alpha) for _ in range(count)]

    def smooth_left_controller(self, position: Iterable[float], rotation: Iterable[float]):
        return self._smooth_pose(self.left_position, self.left_rotation, position, rotation)

    def smooth_right_controller(self, position: Iterable[float], rotation: Iterable[float]):
        return self._smooth_pose(self.right_position, self.right_rotation, position, rotation)

    def smooth_headset(self, position: Iterable[float], rotation: Iterable[float]):
        return self._smooth_pose(self.headset_position, self.headset_rotation, position, rotation)

    def smooth_left_input(self, values: Iterable[float]) -> List[float]:
        return self._smooth_input(self.left_input, values)

    def smooth_right_input(self, values: Iterable[float]) -> List[float]:
        return self._smooth_input(self.right_input, values)

    def _smooth_input(self, smoothers: List[ScalarSmoother], values: Iterable[float]) -> List[float]:
        raw = list(values)
        if not self.enabled or self.smoother_type == SmootherType.NONE:
            return raw
        return [smoother.filter(value) for smoother, value in zip(smoothers, raw)]

    def _smooth_pose(
        self,
        position_smoothers: List[ScalarSmoother],
        rotation_state: QuaternionState,
        position: Iterable[float],
        rotation: Iterable[float],
    ) -> tuple[List[float], List[float]]:
        pos = list(position)
        if self.enabled and self.smoother_type != SmootherType.NONE:
            pos = [smoother.filter(value) for smoother, value in zip(position_smoothers, pos)]
        return pos, self._smooth_quaternion(rotation_state, rotation)

    def _smooth_quaternion(self, state: QuaternionState, rotation: Iterable[float]) -> List[float]:
        current = list(rotation)
        if any(not math.isfinite(v) for v in current):
            return state.value if state.initialized else [0.0, 0.0, 0.0, 1.0]
        current = quat_normalize(current)
        if not state.initialized or self.smoother_type == SmootherType.NONE:
            state.value = current
            state.initialized = True
            return current
        if quat_dot(state.value, current) < 0.0:
            current = [-v for v in current]
        state.value = quat_slerp(state.value, current, self.rotation_alpha)
        return state.value

    def reset(self) -> None:
        for group in (
            self.left_position,
            self.right_position,
            self.headset_position,
            self.left_input,
            self.right_input,
        ):
            for smoother in group:
                smoother.reset()
        self.left_rotation = QuaternionState()
        self.right_rotation = QuaternionState()
        self.headset_rotation = QuaternionState()
