#pragma once

#include <vector>
#include <array>
#include <optional>
#include <cstdint>
#include <cmath>

namespace teleop_vr {

/**
 * @brief VR设备位姿数据
 */
struct VrDevicePose {
    float position[3];     // 位置 (x, y, z)
    float rotation[4];     // 旋转四元数 (x, y, z, w)

    VrDevicePose() {
        position[0] = position[1] = position[2] = 0.0f;
        rotation[0] = rotation[1] = rotation[2] = 0.0f;
        rotation[3] = 1.0f;  // 默认单位四元数
    }
};

/**
 * @brief 手柄输入数据
 */
struct ControllerInput {
    float trigger;              // 扳机值 (0.0 - 1.0)
    bool grip_button;           // Grab抓取按钮
    bool primary_button;        // 主按钮 (左手X / 右手A)
    bool secondary_button;      // 副按钮 (左手Y / 右手B)
    bool menu_button;           // 菜单/Home按钮
    float joystick_x;           // 摇杆X轴 (-1.0 - 1.0)
    float joystick_y;           // 摇杆Y轴 (-1.0 - 1.0)

    ControllerInput()
        : trigger(0.0f)
        , grip_button(false)
        , primary_button(false)
        , secondary_button(false)
        , menu_button(false)
        , joystick_x(0.0f)
        , joystick_y(0.0f) {}
};

/**
 * @brief VR数据包结构
 */
struct VrDataPacket {
    float torque;                      // 扭矩值
    std::array<float, 16> angles;      // 16个关节角度(度)

    // VR设备位姿数据
    VrDevicePose left_controller;      // 左手柄位姿
    VrDevicePose right_controller;     // 右手柄位姿
    VrDevicePose headset;              // 头盔位姿

    // 手柄输入数据
    ControllerInput left_input;        // 左手柄输入
    ControllerInput right_input;       // 右手柄输入

    bool has_vr_device_poses;          // 是否包含VR设备位姿
    bool has_controller_inputs;        // 是否包含手柄输入

    VrDataPacket()
        : torque(0.0f)
        , angles{}
        , left_controller()
        , right_controller()
        , headset()
        , left_input()
        , right_input()
        , has_vr_device_poses(false)
        , has_controller_inputs(false)
    {
    }

    /**
     * @brief 获取左臂角度数据(J1-J7)
     * @return 左臂7个关节角度
     */
    std::array<float, 7> getLeftArmAngles() const {
        std::array<float, 7> left;
        std::copy(angles.begin(), angles.begin() + 7, left.begin());
        return left;
    }

    /**
     * @brief 获取右臂角度数据(J1-J7)
     * @return 右臂7个关节角度
     */
    std::array<float, 7> getRightArmAngles() const {
        std::array<float, 7> right;
        std::copy(angles.begin() + 8, angles.begin() + 15, right.begin());
        return right;
    }

    /**
     * @brief 获取左臂夹爪值
     * @return 左臂夹爪值
     */
    float getLeftGripper() const {
        return angles[7];
    }

    /**
     * @brief 获取右臂夹爪值
     * @return 右臂夹爪值
     */
    float getRightGripper() const {
        return angles[15];
    }
};

/**
 * @brief VR数据解析器
 */
class VrDataParser {
public:
    static constexpr uint8_t FRAME_HEADER_0 = 0xAA;
    static constexpr uint8_t FRAME_HEADER_1 = 0xBB;

    // 数据包各部分的偏移和大小
    static constexpr size_t HEADER_SIZE = 2;           // 帧头
    static constexpr size_t LENGTH_SIZE = 1;           // 数据长度字段
    static constexpr size_t TORQUE_SIZE = 2;           // 扭矩
    static constexpr size_t ANGLES_SIZE = 64;          // 16个关节角度 (16*4)
    static constexpr size_t VR_DEVICE_SIZE = 28;       // 单个VR设备 (3*4 + 4*4)
    static constexpr size_t VR_DEVICES_SIZE = 84;      // 3个VR设备 (3*28)
    static constexpr size_t CONTROLLER_INPUT_SIZE = 16;// 单个手柄输入

    static constexpr size_t MIN_PACKET_SIZE = HEADER_SIZE + LENGTH_SIZE + TORQUE_SIZE + ANGLES_SIZE;  // 最小69字节
    static constexpr size_t FULL_PACKET_SIZE = MIN_PACKET_SIZE + VR_DEVICES_SIZE + 2 * CONTROLLER_INPUT_SIZE;  // 完整185字节

    static constexpr uint8_t EXPECTED_DATA_LENGTH_OLD = 16;
    static constexpr uint8_t EXPECTED_DATA_LENGTH_NEW = 62;

    static constexpr float MAX_ABS_TORQUE = 500.0f;
    static constexpr float MAX_ABS_ANGLE_DEG = 720.0f;
    static constexpr float MAX_ABS_POSITION_M = 5.0f;
    static constexpr float MAX_ABS_JOYSTICK = 1.2f;

    /**
     * @brief 解析UDP数据包
     * @param data 原始数据
     * @param size 数据大小
     * @return 解析后的数据包,解析失败返回nullopt
     */
    static std::optional<VrDataPacket> parse(const std::vector<uint8_t>& data, size_t size);

private:
    /**
     * @brief 解析小端序float
     * @param bytes 4字节数据
     * @return float值
     */
    static float parseFloat(const uint8_t* bytes);

    /**
     * @brief 解析小端序int16
     * @param bytes 2字节数据
     * @return int16值
     */
    static int16_t parseInt16(const uint8_t* bytes);

    /**
     * @brief 解析VR设备位姿
     * @param data 数据指针
     * @param offset 起始偏移
     * @param size 数据总大小
     * @param pose 输出的位姿数据
     * @return 是否成功解析
     */
    static bool parseVrDevicePose(const std::vector<uint8_t>& data, size_t offset, size_t size, VrDevicePose& pose);

    /**
     * @brief 解析手柄输入数据
     * @param data 数据指针
     * @param offset 起始偏移
     * @param size 数据总大小
     * @param input 输出的输入数据
     * @return 是否成功解析
     */
    static bool parseControllerInput(const std::vector<uint8_t>& data, size_t offset, size_t size, ControllerInput& input);

    /**
     * @brief 数值有效性检查
     */
    static bool isFiniteAndInRange(float value, float min_value, float max_value);

    /**
     * @brief 校验完整数据包的数值安全性
     */
    static bool validatePacketValues(const VrDataPacket& packet);
};

} // namespace teleop_vr
