#!/usr/bin/env python3

import argparse
import math
import os
import tkinter as tk
from tkinter import ttk
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


BODY_JOINT_NAMES = {
    "head_joint_1",
    "head_joint_2",
    "waist_joint_3",
    "waist_joint_4",
    "waist_joint_5",
}

BODY_JOINT_NAME_MAP = {
    "head_joint_1": "head_joint_1",
    "head_joint_2": "head_joint_2",
    "waist_joint_3": "waist_joint_3",
    "waist_joint_4": "waist_joint_2",
    "waist_joint_5": "waist_joint_1",
}


def normalize_namespace(robot_namespace: str) -> str:
    namespace = robot_namespace.strip().rstrip("/")
    if not namespace:
        return ""
    if not namespace.startswith("/"):
        namespace = "/" + namespace
    return namespace


def map_live_joint_state(names: list[str], positions: list[float]) -> dict[str, float]:
    mapped: dict[str, float] = {}
    for name, position in zip(names, positions):
        if not math.isfinite(position):
            continue

        urdf_name = ""
        if name.startswith("left_joint"):
            urdf_name = name
        elif name.startswith("right_joint"):
            urdf_name = name
        elif name in BODY_JOINT_NAMES:
            urdf_name = BODY_JOINT_NAME_MAP[name]

        if urdf_name:
            mapped[urdf_name] = float(position)

    return mapped


def format_degrees(radians: float) -> str:
    return f"{math.degrees(radians): .2f} deg"


def should_subscribe_live_topics(mode: str) -> bool:
    return mode == "live"


class JointStateGui(Node):
    def __init__(
        self, urdf_path: str, rate_hz: float, robot_namespace: str, mode: str
    ) -> None:
        super().__init__("simple_joint_state_gui")
        self._mode = mode
        self._joints = self._load_joints(urdf_path)
        self._values: dict[str, tk.DoubleVar] = {}
        self._labels: dict[str, ttk.Label] = {}
        self._positions: dict[str, float] = {
            str(joint["name"]): float(joint["initial"]) for joint in self._joints
        }
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        if should_subscribe_live_topics(mode):
            robot_ns = normalize_namespace(robot_namespace)
            self.create_subscription(
                JointState,
                f"{robot_ns}/arm_left/joint_states",
                self._live_joint_state_cb,
                10,
            )
            self.create_subscription(
                JointState,
                f"{robot_ns}/arm_right/joint_states",
                self._live_joint_state_cb,
                10,
            )
            self.create_subscription(
                JointState, f"{robot_ns}/body/joint_states", self._live_joint_state_cb, 10
            )
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_joint_states)

    @staticmethod
    def _load_joints(urdf_path: str) -> list[dict[str, float | str]]:
        root = ET.parse(urdf_path).getroot()
        joints = []
        for joint in root.findall("joint"):
            joint_type = joint.attrib.get("type", "")
            if joint_type in ("fixed", "floating", "planar"):
                continue

            name = joint.attrib.get("name")
            if not name:
                continue

            lower = -math.pi
            upper = math.pi
            limit = joint.find("limit")
            if limit is not None:
                lower = float(limit.attrib.get("lower", lower))
                upper = float(limit.attrib.get("upper", upper))

            if lower > upper:
                lower, upper = upper, lower

            joints.append(
                {
                    "name": name,
                    "lower": lower,
                    "upper": upper,
                    "initial": min(max(0.0, lower), upper),
                }
            )

        return joints

    def build_window(self) -> tk.Tk:
        root = tk.Tk()
        root.title("JZ Robot Joint State GUI")
        root.geometry("720x820")
        self._values = {
            str(joint["name"]): tk.DoubleVar(value=float(joint["initial"]))
            for joint in self._joints
        }

        container = ttk.Frame(root, padding=10)
        container.pack(fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient=tk.VERTICAL, command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)
        scroll_frame.bind(
            "<Configure>",
            lambda event: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        for index, joint in enumerate(self._joints):
            name = str(joint["name"])
            lower = float(joint["lower"])
            upper = float(joint["upper"])
            value = self._values[name]

            row = ttk.Frame(scroll_frame, padding=(0, 4))
            row.grid(row=index, column=0, sticky="ew")
            row.columnconfigure(1, weight=1)

            ttk.Label(row, text=name, width=28).grid(row=0, column=0, sticky="w")
            scale = ttk.Scale(
                row,
                from_=lower,
                to=upper,
                variable=value,
                orient=tk.HORIZONTAL,
                state=tk.DISABLED if self._mode == "live" else tk.NORMAL,
            )
            scale.grid(row=0, column=1, sticky="ew", padx=(8, 8))
            label = ttk.Label(row, width=12)
            label.grid(row=0, column=2, sticky="e")
            self._labels[name] = label

        self._refresh_window()
        return root

    def _live_joint_state_cb(self, msg: JointState) -> None:
        self._positions.update(map_live_joint_state(list(msg.name), list(msg.position)))

    def _refresh_window(self) -> None:
        for name, value in self._values.items():
            if self._mode == "manual":
                position = value.get()
                self._positions[name] = position
            else:
                position = self._positions.get(name, value.get())
                value.set(position)
            self._labels[name].configure(text=format_degrees(position))

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [str(joint["name"]) for joint in self._joints]
        msg.position = [self._positions.get(name, 0.0) for name in msg.name]
        self._publisher.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True, help="Path to the URDF file.")
    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate in Hz.")
    parser.add_argument(
        "--robot-namespace",
        default=os.environ.get("ROBOT_NS", "robot1"),
        help="Robot namespace used by live joint state topics, for example robot1.",
    )
    parser.add_argument(
        "--mode",
        choices=["live", "manual"],
        default="live",
        help="live subscribes to robot joint states; manual enables sliders.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = JointStateGui(args.urdf, args.rate, args.robot_namespace, args.mode)
    root = node.build_window()
    refresh_ms = max(10, int(1000.0 / args.rate))

    def spin_once() -> None:
        rclpy.spin_once(node, timeout_sec=0.0)
        node._refresh_window()
        root.after(refresh_ms, spin_once)

    root.after(refresh_ms, spin_once)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
