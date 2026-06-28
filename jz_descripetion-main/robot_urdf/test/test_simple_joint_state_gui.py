import math
import sys
import types
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

fake_rclpy = types.ModuleType("rclpy")
fake_rclpy.node = types.ModuleType("rclpy.node")
fake_rclpy.node.Node = object
sys.modules["rclpy"] = fake_rclpy
sys.modules["rclpy.node"] = fake_rclpy.node

fake_sensor_msgs = types.ModuleType("sensor_msgs")
fake_sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
fake_sensor_msgs.msg.JointState = object
sys.modules["sensor_msgs"] = fake_sensor_msgs
sys.modules["sensor_msgs.msg"] = fake_sensor_msgs.msg

from simple_joint_state_gui import (
    format_degrees,
    map_live_joint_state,
    should_subscribe_live_topics,
)


URDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "urdf"
    / "robot urdf.10.8.SLDASM.urdf"
)


def test_keeps_live_arm_joint_names_matching_urdf_joint_names():
    mapped = map_live_joint_state(
        ["left_joint1", "left_joint7", "right_joint2"],
        [0.1, -0.7, 1.2],
    )

    assert mapped == {
        "left_joint1": 0.1,
        "left_joint7": -0.7,
        "right_joint2": 1.2,
    }


def test_maps_body_joint_names_without_renaming():
    mapped = map_live_joint_state(
        [
            "head_joint_1",
            "head_joint_2",
            "waist_joint_3",
            "waist_joint_4",
            "waist_joint_5",
            "unknown",
        ],
        [0.1, -0.2, 0.3, 0.4, 0.5, 9.9],
    )

    assert mapped == {
        "head_joint_1": 0.1,
        "head_joint_2": -0.2,
        "waist_joint_3": 0.3,
        "waist_joint_2": 0.4,
        "waist_joint_1": 0.5,
    }


def test_ignores_non_finite_positions():
    mapped = map_live_joint_state(
        ["left_joint1", "right_joint1", "waist_joint_3"],
        [math.nan, math.inf, -math.inf],
    )

    assert mapped == {}


def test_formats_radians_as_degrees():
    assert format_degrees(math.pi / 2) == " 90.00 deg"
    assert format_degrees(-math.pi) == "-180.00 deg"


def test_manual_mode_does_not_subscribe_live_topics():
    assert should_subscribe_live_topics("manual") is False
    assert should_subscribe_live_topics("live") is True


def test_urdf_joint_names_match_armcontrol_names():
    import xml.etree.ElementTree as ET

    root = ET.parse(URDF_PATH).getroot()
    joint_names = {joint.attrib["name"] for joint in root.findall("joint")}

    expected = {
        *(f"left_joint{i}" for i in range(1, 8)),
        *(f"right_joint{i}" for i in range(1, 8)),
        "head_joint_1",
        "head_joint_2",
        "waist_joint_1",
        "waist_joint_2",
        "waist_joint_3",
    }
    old_names = {
        *(f"left_arm_joint{i}" for i in range(1, 8)),
        *(f"right_arm_joint{i}" for i in range(1, 8)),
    }

    assert expected.issubset(joint_names)
    assert joint_names.isdisjoint(old_names)


def test_urdf_head_joint_names_match_physical_axes():
    import xml.etree.ElementTree as ET

    root = ET.parse(URDF_PATH).getroot()
    joints = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        joints[(parent.attrib["link"], child.attrib["link"])] = joint.attrib["name"]

    assert joints[("body_link3", "body_link4")] == "head_joint_2"
    assert joints[("body_link4", "head_link")] == "head_joint_1"


def test_urdf_waist_joint_names_match_physical_axes():
    import xml.etree.ElementTree as ET

    root = ET.parse(URDF_PATH).getroot()
    joints = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        joints[(parent.attrib["link"], child.attrib["link"])] = joint.attrib["name"]

    assert joints[("base_link", "body_link1")] == "waist_joint_1"
    assert joints[("body_link1", "body_link2")] == "waist_joint_2"
    assert joints[("body_link2", "body_link3")] == "waist_joint_3"
