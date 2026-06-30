"""Pink differential IK solver for the light_teleop dual-arm demo.

This module implements the third milestone only: offline Pink IK for the JZ
robot arms. It does not receive VR packets, publish commands, or talk to the
real robot.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock, Thread
from typing import Iterable
from urllib.parse import parse_qs, urlparse

import numpy as np
import pinocchio as pin
import qpsolvers
import pink
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask

from .model_loader import (
    LEFT_JOINT_NAMES,
    RIGHT_JOINT_NAMES,
    default_package_dir,
    default_urdf_path,
    load_and_validate_model,
    resolve_path,
)


DEFAULT_LEFT_EE_FRAME = "left_arm_link9"
DEFAULT_RIGHT_EE_FRAME = "right_arm_link9"
HOME_NEUTRAL = "neutral"
HOME_PREGRASP = "pregrasp"
HOME_VR_PREGRASP = "vr-pregrasp"

STATIC_MAX_ERROR = 1e-4
SINE_MAX_ERROR = 8e-2
SINE_FINAL_ERROR = 5e-2
NON_ARM_DELTA_LIMIT = 1e-8
LIMIT_TOLERANCE = 1e-6
SLIDER_MODE = "sliders"
ARC_MODE = "arc"
POSTURE_HOME_BIAS_FULL_ERROR = 0.005
POSTURE_HOME_BIAS_DISABLE_ERROR = 0.05

PREGRASP_JOINT_POSITIONS = {
    "left_joint1": -0.30,
    "left_joint2": 0.30,
    "left_joint3": 0.0,
    "left_joint4": -0.90,
    "left_joint5": 0.0,
    "left_joint6": 0.30,
    "left_joint7": 0.0,
    "right_joint1": 0.30,
    "right_joint2": -0.30,
    "right_joint3": 0.0,
    "right_joint4": 0.90,
    "right_joint5": 0.0,
    "right_joint6": -0.30,
    "right_joint7": 0.0,
}

VR_PREGRASP_JOINT_POSITIONS = {
    "left_joint1": -0.050562189728566756,
    "left_joint2": -1.2208577585068117,
    "left_joint3": 3.4906586697860414e-05,
    "left_joint4": -1.4085680047799567,
    "left_joint5": 1.2473406318607232,
    "left_joint6": -0.0006981316851932723,
    "left_joint7": 0.0009773844112854462,
    "right_joint1": 0.05035274875358883,
    "right_joint2": 1.2209974413059863,
    "right_joint3": 3.4906586697860414e-05,
    "right_joint4": 1.4087774623996916,
    "right_joint5": -1.2472645986116015,
    "right_joint6": 0.00024434610282136155,
    "right_joint7": -0.0016057029149556751,
}


@dataclass(frozen=True)
class IkResult:
    """Result returned by one Pink IK step."""

    q: np.ndarray
    left_joint_positions: np.ndarray
    right_joint_positions: np.ndarray
    left_ee_current_pose: pin.SE3
    right_ee_current_pose: pin.SE3
    left_position_error: float
    right_position_error: float


@dataclass(frozen=True)
class RunStats:
    """Acceptance statistics collected by a CLI run."""

    steps: int
    max_left_position_error: float
    max_right_position_error: float
    final_left_position_error: float
    final_right_position_error: float
    max_non_arm_joint_delta: float


class SliderControlState:
    """Thread-safe target offsets updated by Meshcat slider callbacks."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._offsets = {
            "left_x": 0.0,
            "left_y": 0.0,
            "left_z": 0.0,
            "right_x": 0.0,
            "right_y": 0.0,
            "right_z": 0.0,
        }
        self._reset_counter = 0

    def set_offset(self, name: str, value: float) -> None:
        with self._lock:
            if name not in self._offsets:
                raise KeyError(name)
            self._offsets[name] = value

    def reset(self) -> None:
        with self._lock:
            for name in self._offsets:
                self._offsets[name] = 0.0
            self._reset_counter += 1

    def snapshot(self) -> tuple[dict[str, float], int]:
        with self._lock:
            return dict(self._offsets), self._reset_counter


class SliderControlServer:
    """Tiny local HTTP server serving a browser target-control page."""

    def __init__(
        self,
        state: SliderControlState,
        slider_range: float,
        slider_step: float,
        host: str = "127.0.0.1",
        port: int = 0,
    ) -> None:
        self.state = state
        self.slider_range = slider_range
        self.slider_step = slider_step
        self.httpd = self._make_server(host, port)
        self.thread = Thread(target=self.httpd.serve_forever, daemon=True)

    def _make_server(self, host: str, port: int) -> ThreadingHTTPServer:
        state = self.state
        slider_range = self.slider_range
        slider_step = self.slider_step

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802 - stdlib callback name.
                parsed = urlparse(self.path)
                query = parse_qs(parsed.query)
                try:
                    if parsed.path in {"/", "/index.html"}:
                        self._send_html(control_page_html(slider_range, slider_step))
                    elif parsed.path == "/set":
                        name = query.get("name", [""])[0]
                        value = float(query.get("value", ["0"])[0])
                        state.set_offset(name, value)
                        self._send_json({"ok": True})
                    elif parsed.path == "/reset":
                        state.reset()
                        self._send_json({"ok": True})
                    elif parsed.path == "/state":
                        offsets, reset_counter = state.snapshot()
                        self._send_json({"ok": True, "offsets": offsets, "reset_counter": reset_counter})
                    else:
                        self.send_error(404)
                except Exception as exc:  # noqa: BLE001 - return a clear slider error.
                    self.send_response(400)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(json.dumps({"ok": False, "error": str(exc)}).encode("utf-8"))

            def log_message(self, format: str, *args: object) -> None:
                return

            def _send_json(self, data: dict[str, object]) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps(data).encode("utf-8"))

            def _send_html(self, html: str) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(html.encode("utf-8"))

        return ThreadingHTTPServer((host, port), Handler)

    @property
    def url(self) -> str:
        host, port = self.httpd.server_address
        return f"http://{host}:{port}"

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()


class PinkTeleopSolver:
    """Pink solver for left and right arm end-effector targets."""

    def __init__(
        self,
        robot: pin.RobotWrapper,
        q0: np.ndarray,
        left_ee_frame: str = DEFAULT_LEFT_EE_FRAME,
        right_ee_frame: str = DEFAULT_RIGHT_EE_FRAME,
        left_joint_names: list[str] | None = None,
        right_joint_names: list[str] | None = None,
        solver: str | None = None,
        posture_cost: float = 1e-3,
        home_name: str = HOME_NEUTRAL,
        posture_current_ratio: float = 0.98,
        joint_motion_cost: float = 0.0,
    ) -> None:
        self.robot = robot
        self.full_model = robot.model
        self.full_data = robot.data
        self.q0 = q0.copy()
        self.home_q = q0.copy()
        self.left_ee_frame = left_ee_frame
        self.right_ee_frame = right_ee_frame
        self.left_joint_names = left_joint_names or LEFT_JOINT_NAMES
        self.right_joint_names = right_joint_names or RIGHT_JOINT_NAMES
        self.arm_joint_names = self.left_joint_names + self.right_joint_names
        self.solver = solver or choose_qp_solver()
        self.home_name = home_name
        self.posture_cost = posture_cost
        self.posture_current_ratio = posture_current_ratio
        self.joint_motion_cost = joint_motion_cost

        self._validate_inputs()
        self.full_arm_q_indices = self._q_indices(self.full_model, self.arm_joint_names)
        self.full_left_q_indices = self._q_indices(self.full_model, self.left_joint_names)
        self.full_right_q_indices = self._q_indices(self.full_model, self.right_joint_names)
        self.non_arm_q_indices = self._non_arm_q_indices()
        self.locked_joint_ids = self._locked_joint_ids()
        self.model = pin.buildReducedModel(self.full_model, self.locked_joint_ids, self.q0)
        self.data = self.model.createData()
        self.q0_reduced = self.home_q[self.full_arm_q_indices].copy()
        self.left_q_indices = self._q_indices(self.model, self.left_joint_names)
        self.right_q_indices = self._q_indices(self.model, self.right_joint_names)

        self.configuration = pink.Configuration(self.model, self.data, self.q0_reduced)
        self.left_task = FrameTask(
            self.left_ee_frame,
            position_cost=1.0,
            orientation_cost=0.05,
            lm_damping=1e-4,
        )
        self.right_task = FrameTask(
            self.right_ee_frame,
            position_cost=1.0,
            orientation_cost=0.05,
            lm_damping=1e-4,
        )
        self.posture_task = PostureTask(cost=posture_cost, lm_damping=1e-6)
        self.motion_task = PostureTask(cost=joint_motion_cost, lm_damping=0.0)
        self.tasks = [self.left_task, self.right_task, self.posture_task]
        if joint_motion_cost > 0.0:
            self.tasks.append(self.motion_task)

        self.left_target: pin.SE3
        self.right_target: pin.SE3
        self.set_targets(*self.get_current_ee_poses())
        self.posture_task.set_target(self.q0_reduced)
        self.motion_task.set_target(self.configuration.q)

    def _validate_inputs(self) -> None:
        if self.q0.shape != (self.full_model.nq,):
            raise ValueError(
                f"q0 shape mismatch: got {self.q0.shape}, expected {(self.full_model.nq,)}"
            )
        if not np.all(np.isfinite(self.q0)):
            raise ValueError("q0 contains NaN or Inf")
        for frame in [self.left_ee_frame, self.right_ee_frame]:
            if not self.full_model.existFrame(frame):
                raise ValueError(f"frame not found: {frame}")
        for joint_name in self.arm_joint_names:
            if not self.full_model.existJointName(joint_name):
                raise ValueError(f"required arm joint not found: {joint_name}")
            joint_id = self.full_model.getJointId(joint_name)
            if self.full_model.nqs[joint_id] != 1 or self.full_model.nvs[joint_id] != 1:
                raise ValueError(f"expected 1-DoF arm joint, got {joint_name}")

    def _q_indices(self, model: pin.Model, joint_names: Iterable[str]) -> np.ndarray:
        indices = []
        for joint_name in joint_names:
            joint_id = model.getJointId(joint_name)
            indices.append(model.idx_qs[joint_id])
        return np.array(indices, dtype=int)

    def _non_arm_q_indices(self) -> np.ndarray:
        arm_set = set(self.arm_joint_names)
        indices: list[int] = []
        for joint_id, joint_name in enumerate(self.full_model.names):
            if joint_id == 0 or joint_name in arm_set:
                continue
            for offset in range(self.full_model.nqs[joint_id]):
                indices.append(self.full_model.idx_qs[joint_id] + offset)
        return np.array(indices, dtype=int)

    def _locked_joint_ids(self) -> list[int]:
        arm_set = set(self.arm_joint_names)
        return [
            joint_id
            for joint_id, joint_name in enumerate(self.full_model.names)
            if joint_id != 0 and joint_name not in arm_set
        ]

    def set_targets(self, left_target: pin.SE3, right_target: pin.SE3) -> None:
        """Set left and right end-effector targets in the world frame."""

        self.left_target = left_target.copy()
        self.right_target = right_target.copy()
        self.left_task.set_target(self.left_target)
        self.right_task.set_target(self.right_target)

    def step(self, dt: float) -> IkResult:
        """Advance IK by one timestep and return the updated state."""

        if dt <= 0.0 or not math.isfinite(dt):
            raise ValueError(f"invalid dt: {dt}")

        home_ratio = self._dynamic_posture_home_ratio()
        posture_target = (1.0 - home_ratio) * self.configuration.q + home_ratio * self.q0_reduced
        self.posture_task.set_target(posture_target)
        self.motion_task.set_target(self.configuration.q)

        try:
            velocity = solve_ik(
                self.configuration,
                self.tasks,
                dt,
                solver=self.solver,
                damping=1e-8,
            )
        except Exception as exc:  # noqa: BLE001 - include solver context for CLI failures.
            raise RuntimeError(f"Pink solve_ik failed with solver {self.solver}: {exc}") from exc

        if not np.all(np.isfinite(velocity)):
            raise ValueError("Pink solve_ik returned NaN or Inf velocity")

        self.configuration.integrate_inplace(velocity, dt)
        self._validate_configuration()
        return self._make_result()

    def _dynamic_posture_home_ratio(self) -> float:
        """Return a small home bias only when end-effector tracking is close."""

        max_home_ratio = 1.0 - self.posture_current_ratio
        if max_home_ratio <= 0.0:
            return 0.0

        left_pose, right_pose = self.get_current_ee_poses()
        error = max(
            position_error(left_pose, self.left_target),
            position_error(right_pose, self.right_target),
        )
        if error >= POSTURE_HOME_BIAS_DISABLE_ERROR:
            return 0.0
        if error <= POSTURE_HOME_BIAS_FULL_ERROR:
            return max_home_ratio

        scale = (POSTURE_HOME_BIAS_DISABLE_ERROR - error) / (
            POSTURE_HOME_BIAS_DISABLE_ERROR - POSTURE_HOME_BIAS_FULL_ERROR
        )
        return float(np.clip(max_home_ratio * scale, 0.0, max_home_ratio))

    def _validate_configuration(self) -> None:
        if not np.all(np.isfinite(self.configuration.q)):
            raise ValueError("q contains NaN or Inf")
        try:
            self.configuration.check_limits(tol=LIMIT_TOLERANCE)
        except Exception as exc:  # noqa: BLE001 - keep the message concrete for acceptance runs.
            raise ValueError(f"q violates URDF joint limits: {exc}") from exc
        non_arm_delta = self.max_non_arm_joint_delta()
        if non_arm_delta >= NON_ARM_DELTA_LIMIT:
            raise ValueError(
                "non-arm joints moved: "
                f"max delta {non_arm_delta:.12g} >= {NON_ARM_DELTA_LIMIT:.12g}"
            )

    def _make_result(self) -> IkResult:
        left_pose, right_pose = self.get_current_ee_poses()
        q_full = self.get_q()
        return IkResult(
            q=q_full,
            left_joint_positions=self.configuration.q[self.left_q_indices].copy(),
            right_joint_positions=self.configuration.q[self.right_q_indices].copy(),
            left_ee_current_pose=left_pose,
            right_ee_current_pose=right_pose,
            left_position_error=position_error(left_pose, self.left_target),
            right_position_error=position_error(right_pose, self.right_target),
        )

    def get_current_ee_poses(self) -> tuple[pin.SE3, pin.SE3]:
        """Return current left and right end-effector poses."""

        return (
            self.configuration.get_transform_frame_to_world(self.left_ee_frame),
            self.configuration.get_transform_frame_to_world(self.right_ee_frame),
        )

    def get_q(self) -> np.ndarray:
        """Return the current full model configuration."""

        q_full = self.home_q.copy()
        q_full[self.full_arm_q_indices] = self.configuration.q.copy()
        return q_full

    def max_non_arm_joint_delta(self) -> float:
        """Return max absolute q delta outside the 14 arm joints."""

        if self.non_arm_q_indices.size == 0:
            return 0.0
        q_full = self.get_q()
        delta = q_full[self.non_arm_q_indices] - self.q0[self.non_arm_q_indices]
        return float(np.max(np.abs(delta)))


def choose_qp_solver() -> str:
    """Choose a QP solver, preferring daqp when available."""

    available = list(qpsolvers.available_solvers)
    if not available:
        raise RuntimeError("no qpsolvers solver is available")
    if "daqp" in available:
        return "daqp"
    return available[0]


def make_home_configuration(model: pin.Model, neutral_q: np.ndarray, home: str) -> np.ndarray:
    """Return a full-model home configuration for IK demos."""

    q_home = neutral_q.copy()
    if home == HOME_NEUTRAL:
        return q_home
    if home == HOME_PREGRASP:
        joint_positions = PREGRASP_JOINT_POSITIONS
    elif home == HOME_VR_PREGRASP:
        joint_positions = VR_PREGRASP_JOINT_POSITIONS
    else:
        raise ValueError(f"unsupported home posture: {home}")

    for joint_name, value in joint_positions.items():
        if not model.existJointName(joint_name):
            raise ValueError(f"{home} joint not found: {joint_name}")
        joint_id = model.getJointId(joint_name)
        q_index = model.idx_qs[joint_id]
        lower = model.lowerPositionLimit[q_index]
        upper = model.upperPositionLimit[q_index]
        if value < lower - LIMIT_TOLERANCE or value > upper + LIMIT_TOLERANCE:
            raise ValueError(
                f"{home} value for {joint_name}={value} violates limits [{lower}, {upper}]"
            )
        q_home[q_index] = value
    return q_home


def position_error(current: pin.SE3, target: pin.SE3) -> float:
    """Return Euclidean position error between two transforms."""

    return float(np.linalg.norm(current.translation - target.translation))


def make_sine_targets(
    left_home: pin.SE3,
    right_home: pin.SE3,
    elapsed: float,
    amplitude: float,
) -> tuple[pin.SE3, pin.SE3]:
    """Create small smooth target motions around the initial EE poses."""

    phase = 2.0 * math.pi * 0.2 * elapsed
    offset = np.array(
        [
            amplitude * math.sin(phase),
            0.5 * amplitude * math.sin(phase + math.pi / 2.0),
            0.5 * amplitude * math.cos(phase),
        ]
    )
    left = left_home.copy()
    right = right_home.copy()
    left.translation = left.translation + offset
    right.translation = right.translation + np.array([offset[0], -offset[1], -offset[2]])
    return left, right


def make_line_targets(
    left_home: pin.SE3,
    right_home: pin.SE3,
    elapsed: float,
    distance: float,
    axis: str,
    period: float = 8.0,
) -> tuple[pin.SE3, pin.SE3]:
    """Move targets smoothly out and back along one world-frame axis."""

    axis_vectors = {
        "x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0]),
    }
    if axis not in axis_vectors:
        raise ValueError(f"unsupported target axis: {axis}")

    phase = 2.0 * math.pi * (elapsed % period) / period
    alpha = 0.5 - 0.5 * math.cos(phase)
    offset = distance * alpha * axis_vectors[axis]
    left = left_home.copy()
    right = right_home.copy()
    left.translation = left.translation + offset
    right.translation = right.translation + offset
    return left, right


def make_arc_targets(
    left_home: pin.SE3,
    right_home: pin.SE3,
    elapsed: float,
    radius: float,
    lift: float,
    lateral: float,
    period: float,
) -> tuple[pin.SE3, pin.SE3]:
    """Move both targets through a smooth forward arc and back."""

    if period <= 0.0:
        raise ValueError(f"arc period must be positive, got {period}")

    phase = 2.0 * math.pi * (elapsed % period) / period
    forward = radius * math.sin(phase)
    up = lift * (1.0 - math.cos(phase))
    side = lateral * math.sin(phase)

    left = left_home.copy()
    right = right_home.copy()
    left.translation = left.translation + np.array([forward, side, up])
    right.translation = right.translation + np.array([forward, -side, up])
    return left, right


def limit_target_step(current: pin.SE3, desired: pin.SE3, max_step: float) -> pin.SE3:
    """Limit one target translation step while preserving target orientation."""

    if max_step <= 0.0:
        return desired.copy()

    delta = desired.translation - current.translation
    distance = float(np.linalg.norm(delta))
    if distance <= max_step:
        return desired.copy()

    limited = desired.copy()
    limited.translation = current.translation + delta * (max_step / distance)
    return limited


def make_offset_targets(
    left_home: pin.SE3,
    right_home: pin.SE3,
    offsets: dict[str, float],
) -> tuple[pin.SE3, pin.SE3]:
    """Create targets from slider offsets in the world frame."""

    left = left_home.copy()
    right = right_home.copy()
    left.translation = left.translation + np.array(
        [offsets["left_x"], offsets["left_y"], offsets["left_z"]]
    )
    right.translation = right.translation + np.array(
        [offsets["right_x"], offsets["right_y"], offsets["right_z"]]
    )
    return left, right


def control_page_html(slider_range: float, slider_step: float) -> str:
    """Return the standalone browser page used to move target offsets."""

    names = ["left_x", "left_y", "left_z", "right_x", "right_y", "right_z"]
    rows = "\n".join(
        f"""
        <label class="row" for="{name}">
          <span class="name">{name}</span>
          <input id="{name}" type="range" min="{-slider_range:g}" max="{slider_range:g}"
                 step="{slider_step:g}" value="0" data-name="{name}">
          <output id="{name}_value">0.000 m / 0.0 cm</output>
        </label>
        """
        for name in names
    )
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Pink Target Control</title>
  <style>
    :root {{
      color-scheme: light dark;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }}
    body {{
      margin: 0;
      padding: 24px;
      background: #f6f7f9;
      color: #15171a;
    }}
    main {{
      max-width: 820px;
      margin: 0 auto;
    }}
    h1 {{
      margin: 0 0 6px;
      font-size: 24px;
      font-weight: 700;
    }}
    p {{
      margin: 0 0 20px;
      color: #59616d;
    }}
    .panel {{
      display: grid;
      gap: 12px;
      padding: 18px;
      border: 1px solid #d8dde5;
      border-radius: 8px;
      background: #ffffff;
    }}
    .row {{
      display: grid;
      grid-template-columns: 84px minmax(180px, 1fr) 150px;
      gap: 14px;
      align-items: center;
      min-height: 38px;
    }}
    .name {{
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 14px;
    }}
    input[type="range"] {{
      width: 100%;
    }}
    output {{
      font-variant-numeric: tabular-nums;
      text-align: right;
    }}
    .actions {{
      display: flex;
      gap: 10px;
      margin-top: 16px;
    }}
    button {{
      border: 1px solid #b9c1cc;
      border-radius: 6px;
      background: #ffffff;
      color: #15171a;
      padding: 9px 14px;
      font: inherit;
      cursor: pointer;
    }}
    button:hover {{
      background: #eef2f7;
    }}
    .status {{
      margin-top: 14px;
      min-height: 22px;
      color: #59616d;
      font-variant-numeric: tabular-nums;
    }}
    @media (max-width: 720px) {{
      body {{
        padding: 16px;
      }}
      .row {{
        grid-template-columns: 1fr;
        gap: 6px;
      }}
      output {{
        text-align: left;
      }}
    }}
  </style>
</head>
<body>
  <main>
    <h1>Pink Target Control</h1>
    <p>Move target offsets in the world frame. Keep Meshcat open in another tab.</p>
    <section class="panel">
      {rows}
      <div class="actions">
        <button type="button" id="reset">Reset Targets</button>
      </div>
    </section>
    <div class="status" id="status">Ready</div>
  </main>
  <script>
    const sliders = Array.from(document.querySelectorAll('input[type="range"]'));
    const statusEl = document.getElementById('status');

    function formatValue(value) {{
      const meters = Number(value);
      return meters.toFixed(3) + ' m / ' + (meters * 100).toFixed(1) + ' cm';
    }}

    async function setOffset(name, value) {{
      const response = await fetch('/set?name=' + encodeURIComponent(name) +
        '&value=' + encodeURIComponent(value));
      const data = await response.json();
      if (!data.ok) {{
        throw new Error(data.error || 'set failed');
      }}
      statusEl.textContent = 'Updated ' + name + ' = ' + formatValue(value);
    }}

    sliders.forEach((slider) => {{
      const output = document.getElementById(slider.id + '_value');
      output.value = formatValue(slider.value);
      slider.addEventListener('input', () => {{
        output.value = formatValue(slider.value);
        setOffset(slider.dataset.name, slider.value).catch((error) => {{
          statusEl.textContent = 'Error: ' + error.message;
        }});
      }});
    }});

    document.getElementById('reset').addEventListener('click', async () => {{
      await fetch('/reset');
      sliders.forEach((slider) => {{
        slider.value = '0';
        document.getElementById(slider.id + '_value').value = formatValue(0);
      }});
      statusEl.textContent = 'Reset all target offsets';
    }});
  </script>
</body>
</html>
"""


def print_joint_positions(solver: PinkTeleopSolver, result: IkResult) -> None:
    """Print left and right arm joint positions in command order."""

    print("[pink_solver] left joints:")
    for name, value in zip(solver.left_joint_names, result.left_joint_positions):
        print(f"  {name} = {value:.9f}")
    print("[pink_solver] right joints:")
    for name, value in zip(solver.right_joint_names, result.right_joint_positions):
        print(f"  {name} = {value:.9f}")


def run_loop(
    solver: PinkTeleopSolver,
    mode: str,
    duration: float,
    frequency: float,
    target_amplitude: float,
    target_distance: float,
    target_axis: str,
    arc_radius: float,
    arc_lift: float,
    arc_lateral: float,
    arc_period: float,
    target_max_speed: float,
    slider_range: float,
    slider_step: float,
    no_meshes: bool,
    arm_meshes_only: bool,
    visualize: bool,
    urdf_path: Path,
    package_dir: Path,
) -> RunStats:
    """Run a static or sine IK acceptance loop."""

    if mode not in {"static", "sine", "line", ARC_MODE, SLIDER_MODE}:
        raise ValueError(f"unsupported mode: {mode}")
    if mode == SLIDER_MODE and not visualize:
        raise ValueError("slider mode requires --visualize")
    if duration <= 0.0:
        raise ValueError(f"duration must be positive, got {duration}")
    if frequency <= 0.0:
        raise ValueError(f"frequency must be positive, got {frequency}")
    if target_max_speed < 0.0:
        raise ValueError(f"target_max_speed must be non-negative, got {target_max_speed}")

    visualizer = None
    target_pair_type = None
    slider_state = None
    slider_server = None
    if visualize:
        from .visualizer import TargetPair, TeleopVisualizer, parse_visual_meshes

        visual_meshes, missing_visual_meshes = parse_visual_meshes(
            urdf_path,
            package_dir,
            show_meshes=not no_meshes,
        )
        if arm_meshes_only:
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
        )
        target_pair_type = TargetPair
        print(f"[pink_solver] Meshcat URL: {visualizer.url()}")
        if no_meshes:
            print("[pink_solver] visual meshes disabled")
        elif arm_meshes_only:
            print(f"[pink_solver] arm visual meshes loaded: {len(visual_meshes)}")
        else:
            print(f"[pink_solver] visual meshes loaded: {len(visual_meshes)}")
        if missing_visual_meshes:
            print(f"[pink_solver] warning: {len(missing_visual_meshes)} visual mesh assets are missing")
        if mode == SLIDER_MODE:
            slider_state = SliderControlState()
            slider_server = SliderControlServer(
                slider_state,
                slider_range=slider_range,
                slider_step=slider_step,
            )
            slider_server.start()
            print(f"[pink_solver] Control page URL: {slider_server.url}/")
            print("[pink_solver] open Meshcat for visualization, then use the control page sliders")

    left_home, right_home = solver.get_current_ee_poses()
    solver.set_targets(left_home, right_home)

    steps = int(round(duration * frequency))
    dt = 1.0 / frequency
    max_left_error = 0.0
    max_right_error = 0.0
    max_non_arm_delta = 0.0
    result = solver._make_result()
    last_left_target = left_home.copy()
    last_right_target = right_home.copy()
    start = time.monotonic()

    try:
        for step_index in range(steps):
            elapsed = step_index * dt
            if mode == "sine":
                left_target, right_target = make_sine_targets(
                    left_home,
                    right_home,
                    elapsed,
                    target_amplitude,
                )
            elif mode == SLIDER_MODE:
                if slider_state is None or slider_server is None:
                    raise RuntimeError("slider state was not initialized")
                offsets, _ = slider_state.snapshot()
                left_target, right_target = make_offset_targets(left_home, right_home, offsets)
            elif mode == "line":
                left_target, right_target = make_line_targets(
                    left_home,
                    right_home,
                    elapsed,
                    target_distance,
                    target_axis,
                )
            elif mode == ARC_MODE:
                left_target, right_target = make_arc_targets(
                    left_home,
                    right_home,
                    elapsed,
                    arc_radius,
                    arc_lift,
                    arc_lateral,
                    arc_period,
                )
            else:
                left_target = left_home
                right_target = right_home

            if target_max_speed > 0.0:
                max_step = target_max_speed * dt
                left_target = limit_target_step(last_left_target, left_target, max_step)
                right_target = limit_target_step(last_right_target, right_target, max_step)
            solver.set_targets(left_target, right_target)
            last_left_target = left_target.copy()
            last_right_target = right_target.copy()

            result = solver.step(dt)
            max_left_error = max(max_left_error, result.left_position_error)
            max_right_error = max(max_right_error, result.right_position_error)
            max_non_arm_delta = max(max_non_arm_delta, solver.max_non_arm_joint_delta())

            if visualizer is not None and target_pair_type is not None:
                visualizer.display(
                    result.q,
                    target_pair_type(solver.left_target, solver.right_target),
                    show_targets=True,
                )
                sleep_until = start + (step_index + 1) * dt
                time.sleep(max(0.0, sleep_until - time.monotonic()))
    finally:
        if slider_server is not None:
            slider_server.close()

    return RunStats(
        steps=steps,
        max_left_position_error=max_left_error,
        max_right_position_error=max_right_error,
        final_left_position_error=result.left_position_error,
        final_right_position_error=result.right_position_error,
        max_non_arm_joint_delta=max_non_arm_delta,
    )


def load_solver_from_args(args: argparse.Namespace) -> tuple[PinkTeleopSolver, Path, Path]:
    """Load the model and create a PinkTeleopSolver from CLI arguments."""

    urdf_path = resolve_path(args.urdf)
    package_dir = resolve_path(args.package_dir)
    load_result = load_and_validate_model(urdf_path, package_dir, load_geometry=False)
    home_q = make_home_configuration(load_result.robot.model, load_result.configuration.q, args.home)
    solver = PinkTeleopSolver(
        robot=load_result.robot,
        q0=home_q,
        left_ee_frame=args.left_ee_frame,
        right_ee_frame=args.right_ee_frame,
        solver=args.solver,
        posture_cost=args.posture_cost,
        home_name=args.home,
    )
    return solver, urdf_path, package_dir


def print_check_report(solver: PinkTeleopSolver) -> None:
    """Print solver initialization acceptance details."""

    left_pose, right_pose = solver.get_current_ee_poses()
    solver.set_targets(left_pose, right_pose)
    result = solver._make_result()

    print(f"[pink_solver] loaded model: nq={solver.full_model.nq} nv={solver.full_model.nv}")
    print(f"[pink_solver] reduced IK model: nq={solver.model.nq} nv={solver.model.nv}")
    print(f"[pink_solver] left_ee_frame: {solver.left_ee_frame}")
    print(f"[pink_solver] right_ee_frame: {solver.right_ee_frame}")
    print(f"[pink_solver] home: {solver.home_name}")
    print(f"[pink_solver] qp solver: {solver.solver}")
    print(f"[pink_solver] posture_cost: {solver.posture_cost:g}")
    print("[pink_solver] tasks: left_frame, right_frame, posture")
    print(f"[pink_solver] left_joint_positions.shape: {result.left_joint_positions.shape}")
    print(f"[pink_solver] right_joint_positions.shape: {result.right_joint_positions.shape}")
    print_joint_positions(solver, result)
    print("[pink_solver] check: OK")


def validate_stats(mode: str, stats: RunStats) -> None:
    """Validate CLI acceptance thresholds."""

    if stats.max_non_arm_joint_delta >= NON_ARM_DELTA_LIMIT:
        raise ValueError(
            "max_non_arm_joint_delta too large: "
            f"{stats.max_non_arm_joint_delta:.12g} >= {NON_ARM_DELTA_LIMIT:.12g}"
        )
    if mode == "static":
        if stats.max_left_position_error >= STATIC_MAX_ERROR:
            raise ValueError(
                f"static left position error too large: {stats.max_left_position_error:.12g}"
            )
        if stats.max_right_position_error >= STATIC_MAX_ERROR:
            raise ValueError(
                f"static right position error too large: {stats.max_right_position_error:.12g}"
            )
    elif mode == "sine":
        if stats.max_left_position_error >= SINE_MAX_ERROR:
            raise ValueError(f"sine left max error too large: {stats.max_left_position_error:.12g}")
        if stats.max_right_position_error >= SINE_MAX_ERROR:
            raise ValueError(
                f"sine right max error too large: {stats.max_right_position_error:.12g}"
            )
        if stats.final_left_position_error >= SINE_FINAL_ERROR:
            raise ValueError(
                f"sine left final error too large: {stats.final_left_position_error:.12g}"
            )
        if stats.final_right_position_error >= SINE_FINAL_ERROR:
            raise ValueError(
                f"sine right final error too large: {stats.final_right_position_error:.12g}"
            )
    elif mode == "line":
        if stats.max_left_position_error >= SINE_MAX_ERROR:
            raise ValueError(f"line left max error too large: {stats.max_left_position_error:.12g}")
        if stats.max_right_position_error >= SINE_MAX_ERROR:
            raise ValueError(
                f"line right max error too large: {stats.max_right_position_error:.12g}"
            )
    elif mode == ARC_MODE:
        return
    elif mode == SLIDER_MODE:
        return


def print_run_report(mode: str, stats: RunStats, thresholds_checked: bool = True) -> None:
    """Print acceptance statistics from a CLI run."""

    print(f"[pink_solver] mode: {mode}")
    print(f"[pink_solver] steps: {stats.steps}")
    print(f"[pink_solver] max_left_position_error: {stats.max_left_position_error:.9f}")
    print(f"[pink_solver] max_right_position_error: {stats.max_right_position_error:.9f}")
    print(f"[pink_solver] final_left_position_error: {stats.final_left_position_error:.9f}")
    print(f"[pink_solver] final_right_position_error: {stats.final_right_position_error:.9f}")
    print(f"[pink_solver] max_non_arm_joint_delta: {stats.max_non_arm_joint_delta:.12f}")
    if thresholds_checked:
        print("[pink_solver] check: OK")
    else:
        print("[pink_solver] visualizer run: OK")


def build_arg_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", default=str(default_urdf_path()))
    parser.add_argument("--package-dir", default=str(default_package_dir()))
    parser.add_argument("--left-ee-frame", default=DEFAULT_LEFT_EE_FRAME)
    parser.add_argument("--right-ee-frame", default=DEFAULT_RIGHT_EE_FRAME)
    parser.add_argument("--solver", default=None)
    parser.add_argument(
        "--home",
        choices=[HOME_NEUTRAL, HOME_PREGRASP, HOME_VR_PREGRASP],
        default=HOME_NEUTRAL,
        help="Initial and posture-reference arm pose.",
    )
    parser.add_argument(
        "--posture-cost",
        type=float,
        default=1e-3,
        help="Pink PostureTask cost around the selected home pose.",
    )
    parser.add_argument("--check", action="store_true")
    parser.add_argument(
        "--mode",
        choices=["static", "sine", "line", ARC_MODE, SLIDER_MODE],
        default="static",
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--frequency", type=float, default=50.0)
    parser.add_argument("--target-amplitude", type=float, default=0.02)
    parser.add_argument(
        "--target-distance",
        type=float,
        default=0.08,
        help="Maximum line-mode target displacement in meters.",
    )
    parser.add_argument(
        "--target-axis",
        choices=["x", "y", "z"],
        default="x",
        help="World/base axis used by line mode.",
    )
    parser.add_argument(
        "--arc-radius",
        type=float,
        default=0.14,
        help="Arc-mode forward/back amplitude in meters.",
    )
    parser.add_argument(
        "--arc-lift",
        type=float,
        default=0.08,
        help="Arc-mode vertical lift at the top of the arc in meters.",
    )
    parser.add_argument(
        "--arc-lateral",
        type=float,
        default=0.04,
        help="Arc-mode sideways amplitude in meters.",
    )
    parser.add_argument(
        "--arc-period",
        type=float,
        default=10.0,
        help="Arc-mode period in seconds.",
    )
    parser.add_argument(
        "--target-max-speed",
        type=float,
        default=0.0,
        help="Optional target translation speed limit in m/s; 0 disables it.",
    )
    parser.add_argument(
        "--slider-range",
        type=float,
        default=0.15,
        help="Interactive slider min/max target offset in meters.",
    )
    parser.add_argument(
        "--slider-step",
        type=float,
        default=0.005,
        help="Interactive slider step size in meters.",
    )
    parser.add_argument(
        "--no-meshes",
        action="store_true",
        help="Visualizer: draw only skeleton, frames, and target spheres.",
    )
    parser.add_argument(
        "--arm-meshes-only",
        action="store_true",
        help="Visualizer: load only left/right arm meshes.",
    )
    parser.add_argument("--visualize", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""

    args = build_arg_parser().parse_args(argv)
    try:
        solver, urdf_path, package_dir = load_solver_from_args(args)
        if args.check:
            print_check_report(solver)
            return 0

        stats = run_loop(
            solver=solver,
            mode=args.mode,
            duration=args.duration,
            frequency=args.frequency,
            target_amplitude=args.target_amplitude,
            target_distance=args.target_distance,
            target_axis=args.target_axis,
            arc_radius=args.arc_radius,
            arc_lift=args.arc_lift,
            arc_lateral=args.arc_lateral,
            arc_period=args.arc_period,
            target_max_speed=args.target_max_speed,
            slider_range=args.slider_range,
            slider_step=args.slider_step,
            no_meshes=args.no_meshes,
            arm_meshes_only=args.arm_meshes_only,
            visualize=args.visualize,
            urdf_path=urdf_path,
            package_dir=package_dir,
        )
        thresholds_checked = not args.visualize
        if thresholds_checked:
            validate_stats(args.mode, stats)
        print_run_report(args.mode, stats, thresholds_checked=thresholds_checked)
    except KeyboardInterrupt:
        print("[pink_solver] interrupted")
        return 130
    except Exception as exc:  # noqa: BLE001 - acceptance CLI should fail loudly and clearly.
        print(f"[pink_solver] ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
