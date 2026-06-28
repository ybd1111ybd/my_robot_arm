#include "teleop_vr_recv/vr_data_parser.h"
#include <cstring>
#include <iostream>

namespace teleop_vr {

std::optional<VrDataPacket> VrDataParser::parse(const std::vector<uint8_t>& data, size_t size) {
    if (size < MIN_PACKET_SIZE) {
        return std::nullopt;
    }

    if (data[0] != FRAME_HEADER_0 || data[1] != FRAME_HEADER_1) {
        return std::nullopt;
    }

    const uint8_t data_length_units = data[2];
    const size_t declared_total_size = HEADER_SIZE + LENGTH_SIZE + static_cast<size_t>(data_length_units) * 4U;
    if (declared_total_size > size) {
        std::cerr << "Warning: packet shorter than declared length, drop packet" << std::endl;
        return std::nullopt;
    }

    if (data_length_units != EXPECTED_DATA_LENGTH_OLD && data_length_units != EXPECTED_DATA_LENGTH_NEW) {
        std::cerr << "Warning: unexpected length field " << static_cast<int>(data_length_units)
                  << " (accept with strict value validation)" << std::endl;
    }

    VrDataPacket packet;

    int16_t torque_raw = parseInt16(&data[3]);
    packet.torque = torque_raw / 10.0f;

    size_t offset = HEADER_SIZE + LENGTH_SIZE + TORQUE_SIZE;
    for (size_t i = 0; i < 16; ++i) {
        if (offset + sizeof(float) > size) {
            std::cerr << "Warning: incomplete joint angle data, drop packet" << std::endl;
            return std::nullopt;
        }
        packet.angles[i] = parseFloat(&data[offset]);
        offset += sizeof(float);
    }

    const size_t remaining = size - offset;
    if (remaining == 0) {
        if (!validatePacketValues(packet)) {
            return std::nullopt;
        }
        return packet;
    }

    if (remaining < VR_DEVICES_SIZE + 2 * CONTROLLER_INPUT_SIZE) {
        std::cerr << "Warning: incomplete extended VR data, drop packet" << std::endl;
        return std::nullopt;
    }

    if (!parseVrDevicePose(data, offset, size, packet.left_controller)) {
        std::cerr << "Warning: incomplete left controller pose data, drop packet" << std::endl;
        return std::nullopt;
    }
    offset += VR_DEVICE_SIZE;

    if (!parseVrDevicePose(data, offset, size, packet.right_controller)) {
        std::cerr << "Warning: incomplete right controller pose data, drop packet" << std::endl;
        return std::nullopt;
    }
    offset += VR_DEVICE_SIZE;

    if (!parseVrDevicePose(data, offset, size, packet.headset)) {
        std::cerr << "Warning: incomplete headset pose data, drop packet" << std::endl;
        return std::nullopt;
    }
    offset += VR_DEVICE_SIZE;
    packet.has_vr_device_poses = true;

    if (!parseControllerInput(data, offset, size, packet.left_input)) {
        std::cerr << "Warning: incomplete left controller input data, drop packet" << std::endl;
        return std::nullopt;
    }
    offset += CONTROLLER_INPUT_SIZE;

    if (!parseControllerInput(data, offset, size, packet.right_input)) {
        std::cerr << "Warning: incomplete right controller input data, drop packet" << std::endl;
        return std::nullopt;
    }
    packet.has_controller_inputs = true;

    if (!validatePacketValues(packet)) {
        return std::nullopt;
    }

    return packet;
}

float VrDataParser::parseFloat(const uint8_t* bytes) {
    float value;
    std::memcpy(&value, bytes, sizeof(float));
    return value;
}

int16_t VrDataParser::parseInt16(const uint8_t* bytes) {
    return static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
}

bool VrDataParser::parseVrDevicePose(const std::vector<uint8_t>& data, size_t offset, size_t size, VrDevicePose& pose) {
    if (offset + VR_DEVICE_SIZE > size) {
        return false;
    }

    pose.position[0] = parseFloat(&data[offset]);
    pose.position[1] = parseFloat(&data[offset + 4]);
    pose.position[2] = parseFloat(&data[offset + 8]);

    pose.rotation[0] = parseFloat(&data[offset + 12]);
    pose.rotation[1] = parseFloat(&data[offset + 16]);
    pose.rotation[2] = parseFloat(&data[offset + 20]);
    pose.rotation[3] = parseFloat(&data[offset + 24]);

    return true;
}

bool VrDataParser::parseControllerInput(const std::vector<uint8_t>& data, size_t offset, size_t size, ControllerInput& input) {
    if (offset + CONTROLLER_INPUT_SIZE > size) {
        return false;
    }

    input.trigger = parseFloat(&data[offset]);
    offset += 4;

    input.grip_button = static_cast<bool>(data[offset]);
    offset += 1;

    input.primary_button = static_cast<bool>(data[offset]);
    offset += 1;

    input.secondary_button = static_cast<bool>(data[offset]);
    offset += 1;

    input.menu_button = static_cast<bool>(data[offset]);
    offset += 1;

    input.joystick_x = parseFloat(&data[offset]);
    offset += 4;

    input.joystick_y = parseFloat(&data[offset]);

    return true;
}

bool VrDataParser::isFiniteAndInRange(float value, float min_value, float max_value) {
    return std::isfinite(value) && value >= min_value && value <= max_value;
}

bool VrDataParser::validatePacketValues(const VrDataPacket& packet) {
    if (!isFiniteAndInRange(packet.torque, -MAX_ABS_TORQUE, MAX_ABS_TORQUE)) {
        std::cerr << "Warning: invalid torque value, drop packet" << std::endl;
        return false;
    }

    for (size_t i = 0; i < packet.angles.size(); ++i) {
        if (!isFiniteAndInRange(packet.angles[i], -MAX_ABS_ANGLE_DEG, MAX_ABS_ANGLE_DEG)) {
            std::cerr << "Warning: invalid joint angle at index " << i << ", drop packet" << std::endl;
            return false;
        }
    }

    const auto validate_pose = [](const VrDevicePose& pose) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!isFiniteAndInRange(pose.position[axis], -MAX_ABS_POSITION_M, MAX_ABS_POSITION_M)) {
                return false;
            }
        }

        const float qx = pose.rotation[0];
        const float qy = pose.rotation[1];
        const float qz = pose.rotation[2];
        const float qw = pose.rotation[3];
        if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw)) {
            return false;
        }
        const float norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        return norm > 1e-6f && norm < 10.0f;
    };

    if (packet.has_vr_device_poses) {
        if (!validate_pose(packet.left_controller) ||
            !validate_pose(packet.right_controller) ||
            !validate_pose(packet.headset)) {
            std::cerr << "Warning: invalid VR pose value(s), drop packet" << std::endl;
            return false;
        }
    }

    const auto validate_input = [](const ControllerInput& input) {
        if (!isFiniteAndInRange(input.trigger, -0.1f, 1.1f)) {
            return false;
        }
        if (!isFiniteAndInRange(input.joystick_x, -MAX_ABS_JOYSTICK, MAX_ABS_JOYSTICK) ||
            !isFiniteAndInRange(input.joystick_y, -MAX_ABS_JOYSTICK, MAX_ABS_JOYSTICK)) {
            return false;
        }
        return true;
    };

    if (packet.has_controller_inputs) {
        if (!validate_input(packet.left_input) || !validate_input(packet.right_input)) {
            std::cerr << "Warning: invalid controller input value(s), drop packet" << std::endl;
            return false;
        }
    }

    return true;
}

} // namespace teleop_vr
