"""Workspace boundary limiter ported from the C++ implementation."""

from __future__ import annotations

import math
from typing import Iterable, List

from .math_utils import vec_add, vec_mul, vec_norm, vec_sub


class WorkspaceLimiter:
    def __init__(self) -> None:
        self.enabled = False
        self.max_radius = 1.0
        self.boundary_type = "clamp"
        self.center = [0.0, 0.0, 0.0]

    def set_limits(self, enable: bool, max_radius: float, boundary_type: str = "clamp") -> None:
        self.enabled = enable
        self.max_radius = max_radius if math.isfinite(max_radius) and max_radius > 0.0 else 1.0
        self.boundary_type = boundary_type if boundary_type in ("clamp", "saturate") else "clamp"

    def set_center(self, center: Iterable[float]) -> None:
        values = list(center)
        if len(values) != 3 or any(not math.isfinite(v) for v in values):
            return
        self.center = values

    def apply(self, position: Iterable[float]) -> List[float]:
        values = list(position)
        if not self.enabled:
            return values
        if self.boundary_type == "saturate":
            return self._saturate(values)
        return self._clamp(values)

    def _clamp(self, position: List[float]) -> List[float]:
        delta = vec_sub(position, self.center)
        distance = vec_norm(delta)
        if distance > self.max_radius and distance > 1e-12:
            delta = vec_mul(delta, self.max_radius / distance)
        return vec_add(self.center, delta)

    def _saturate(self, position: List[float]) -> List[float]:
        delta = vec_sub(position, self.center)
        distance = vec_norm(delta)
        if distance < 1e-6:
            return position
        saturated_distance = self.max_radius * math.tanh(max(0.0, distance / self.max_radius))
        delta = vec_mul(delta, saturated_distance / distance)
        return vec_add(self.center, delta)
