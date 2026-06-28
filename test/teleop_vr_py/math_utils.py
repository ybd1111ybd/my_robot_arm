"""Small vector and quaternion helpers using only the Python standard library."""

from __future__ import annotations

import math
from typing import Iterable, List


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def vec_add(a: Iterable[float], b: Iterable[float]) -> List[float]:
    return [x + y for x, y in zip(a, b)]


def vec_sub(a: Iterable[float], b: Iterable[float]) -> List[float]:
    return [x - y for x, y in zip(a, b)]


def vec_mul(a: Iterable[float], scale: float) -> List[float]:
    return [x * scale for x in a]


def vec_norm(a: Iterable[float]) -> float:
    return math.sqrt(sum(x * x for x in a))


def vec_is_finite(a: Iterable[float]) -> bool:
    return all(math.isfinite(x) for x in a)


def quat_normalize(q: Iterable[float]) -> List[float]:
    values = list(q)
    norm = vec_norm(values)
    if norm < 1e-12 or not math.isfinite(norm):
        return [0.0, 0.0, 0.0, 1.0]
    return [v / norm for v in values]


def quat_conjugate(q: Iterable[float]) -> List[float]:
    x, y, z, w = q
    return [-x, -y, -z, w]


def quat_multiply(a: Iterable[float], b: Iterable[float]) -> List[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quat_inverse(q: Iterable[float]) -> List[float]:
    values = list(q)
    norm_sq = sum(v * v for v in values)
    if norm_sq < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [v / norm_sq for v in quat_conjugate(values)]


def quat_dot(a: Iterable[float], b: Iterable[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def quat_slerp(a: Iterable[float], b: Iterable[float], t: float) -> List[float]:
    qa = quat_normalize(a)
    qb = quat_normalize(b)
    t = clamp(t, 0.0, 1.0)
    dot = quat_dot(qa, qb)
    if dot < 0.0:
        qb = [-v for v in qb]
        dot = -dot
    if dot > 0.9995:
        return quat_normalize([(1.0 - t) * x + t * y for x, y in zip(qa, qb)])
    theta_0 = math.acos(clamp(dot, -1.0, 1.0))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return quat_normalize([s0 * x + s1 * y for x, y in zip(qa, qb)])


def quat_to_matrix(q: Iterable[float]) -> List[List[float]]:
    x, y, z, w = quat_normalize(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def matrix_to_quat(m: List[List[float]]) -> List[float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return quat_normalize([x, y, z, w])


def quat_from_axis_angle(axis: Iterable[float], angle: float) -> List[float]:
    ax = list(axis)
    norm = vec_norm(ax)
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    half = angle * 0.5
    scale = math.sin(half) / norm
    return quat_normalize([ax[0] * scale, ax[1] * scale, ax[2] * scale, math.cos(half)])
