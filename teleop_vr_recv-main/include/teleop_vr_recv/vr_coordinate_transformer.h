#pragma once

#include <array>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "teleop_vr_recv/workspace_limiter.h"

namespace teleop_vr {

/**
 * @brief 机器人末端目标位姿
 */
struct RobotEndEffectorPose {
    double position[3];      // 位置 [x, y, z] (米)
    double orientation[4];   // 姿态四元数 [qx, qy, qz, qw]

    RobotEndEffectorPose() {
        position[0] = position[1] = position[2] = 0.0;
        orientation[0] = orientation[1] = orientation[2] = 0.0;
        orientation[3] = 1.0;  // 单位四元数
    }
};

/**
 * @brief VR手柄坐标到机器人末端位姿的转换器
 *
 * 功能：
 * 1. 相对运动映射：VR手柄的移动增量 → 机器人末端移动增量
 * 2. 坐标轴映射：处理VR坐标系和机器人坐标系轴向不一致的问题
 * 3. 比例缩放：VR空间到机器人工作空间的缩放
 *
 * 使用流程：
 * 1. setRobotHomePose() - 设置机器人Home位姿（启动时的确定位置）
 * 2. setScaleFactor() - 设置比例因子（可选）
 * 3. setAxisMapping() - 设置坐标轴映射（可选）
 * 4. lockVrReference() - 按下手柄按钮时锁定VR起点
 * 5. transform() - 实时转换VR位姿到机器人目标位姿
 */
class VrCoordinateTransformer {
public:
    /**
     * @brief 构造函数
     * @param scale_factor 比例缩放因子（默认1.0）
     */
    VrCoordinateTransformer(double scale_factor = 1.0);

    /**
     * @brief 设置机器人Home位姿（启动时的确定位置）
     * @param position 机器人末端Home位置 [x, y, z] (米)
     * @param orientation 机器人末端Home姿态四元数 [qx, qy, qz, qw]
     *
     * 这个Home位姿是你每次启动时让机器人移动到的确定位置
     */
    void setRobotHomePose(const double position[3], const double orientation[4]);

    /**
     * @brief 锁定当前VR手柄位姿作为参考起点
     * @param vr_position VR手柄当前位置 [x, y, z]
     * @param vr_orientation VR手柄当前姿态四元数 [qx, qy, qz, qw]
     *
     * 通常在按下手柄按钮（如Grip）时调用
     *
     * 行为：
     * - 首次锁定：记录VR起点，机器人从Home位姿开始
     * - 重新锁定：记录新的VR起点，机器人从上次的目标位姿继续
     *   （实现连续的增量控制，松开按钮后机器人保持在当前位置）
     */
    void lockVrReference(const float vr_position[3], const float vr_orientation[4]);

    /**
     * @brief 将VR手柄位姿转换为机器人末端目标位姿
     * @param vr_position VR手柄当前位置 [x, y, z]
     * @param vr_orientation VR手柄当前姿态四元数 [qx, qy, qz, qw]
     * @return 机器人末端目标位姿
     *
     * 计算公式：
     * target_position = robot_reference_position + scale * (vr_position - vr_reference_position)
     * target_orientation = robot_reference_orientation * (vr_orientation * vr_reference_orientation^-1)
     *
     * 行为：
     * - 已锁定：计算并返回基于VR增量的目标位姿
     * - 未锁定：返回上次的目标位姿（保持不动，不跳回Home）
     */
    RobotEndEffectorPose transform(const float vr_position[3],
                                   const float vr_orientation[4]);

    /**
     * @brief 设置比例缩放因子
     * @param scale_factor 比例因子
     *
     * 例如：scale_factor = 0.5 表示VR手柄移动1米，机器人末端移动0.5米
     * 建议范围：0.3 - 1.0
     */
    void setScaleFactor(double scale_factor);

    /**
     * @brief 设置坐标轴映射（使用1-based索引）
     * @param mapping 坐标轴映射规则 [x_axis, y_axis, z_axis]
     *
     * 说明：
     * - 使用1-based索引：1=X轴, 2=Y轴, 3=Z轴
     * - 负数表示反向：-1=-X轴, -2=-Y轴, -3=-Z轴
     * - mapping[i] 表示 VR的第i个轴 映射到 机器人的第mapping[i]个轴
     *
     * 例如：
     * {1, 2, 3}   - 不做映射（默认）：VR的XYZ → 机器人的XYZ
     * {1, 3, 2}   - VR的Y和Z互换：VR的X→机器人X, VR的Y→机器人Z, VR的Z→机器人Y
     * {2, 1, 3}   - VR的X和Y互换：VR的X→机器人Y, VR的Y→机器人X, VR的Z→机器人Z
     * {1, 2, -3}  - VR的Z反向：VR的Z轴翻转
     * {-1, 2, 3}  - VR的X反向：VR的X轴翻转（现在支持了！）
     * {3, -1, 2}  - VR的X→机器人Z, VR的Y→机器人-X, VR的Z→机器人Y
     */
    void setAxisMapping(const std::array<int, 3>& mapping);

    /**
     * @brief 设置工具姿态偏置
     * @param offset 从 IK 目标帧到“手柄握持/工具”虚拟坐标系的固定旋转
     *
     * 用于把控制器的相对旋转解释在工具坐标系中，而不是直接解释在
     * link8 的原始局部坐标系中。左右臂通常需要不同的固定偏移。
     */
    void setToolOrientationOffset(const Eigen::Quaterniond& offset);

    /**
     * @brief 重置VR参考起点（解除锁定）
     */
    void resetVrReference();

    /**
     * @brief 重置到Home位置（按A/X键时调用）
     *
     * 此方法会：
     * 1. 解除VR参考锁定
     * 2. 将机器人参考位姿重置为Home
     * 3. 将上次目标位姿重置为Home
     * 4. 清空VR参考起点
     *
     * 这样确保下次按Grip键开始控制时，是从Home位置开始的，不会出现位置跳变
     */
    void resetToHome();

    /**
     * @brief 检查是否已锁定VR参考起点
     */
    bool isReferenceLocked() const { return reference_locked_; }

    /**
     * @brief 获取当前比例因子
     */
    double getScaleFactor() const { return scale_factor_; }

    /**
     * @brief 获取当前坐标轴映射
     */
    std::array<int, 3> getAxisMapping() const { return axis_mapping_; }

    /**
     * @brief 设置工作空间边界限制
     * @param enable 是否启用边界限制
     * @param max_radius 最大工作半径（米）- 从基座到末端的距离
     * @param boundary_type 边界类型: "clamp"(硬限制) 或 "saturate"(平滑限制)
     *
     * 设置机器人的工作空间边界，防止VR手柄控制超出机器人实际可达范围
     */
    void setWorkspaceLimits(bool enable, double max_radius, const std::string& boundary_type = "clamp");

    /**
     * @brief 获取工作空间限制配置
     */
    bool isWorkspaceLimitEnabled() const { return workspace_limiter_.isEnabled(); }
    double getMaxWorkspaceRadius() const { return workspace_limiter_.getMaxRadius(); }
    std::string getBoundaryType() const { return workspace_limiter_.getBoundaryType(); }

    /**
     * @brief 设置工作空间球心（与目标位姿相同坐标系）
     * @param center 球心 [x, y, z]
     */
    void setWorkspaceCenter(const double center[3]);

    /**
     * @brief 设置工作空间球心（Eigen版本）
     */
    void setWorkspaceCenter(const Eigen::Vector3d& center);

    /**
     * @brief 获取当前工作空间球心
     */
    Eigen::Vector3d getWorkspaceCenter() const { return workspace_limiter_.getCenter(); }

    /**
     * @brief 获取机器人Home位姿
     * @return 包含位置和姿态的结构体
     */
    RobotEndEffectorPose getRobotHomePose() const;

private:
    /**
     * @brief 检查轴映射是否合法（每个轴只能出现一次，取值范围[-3,-1]∪[1,3]）
     */
    static bool isValidAxisMapping(const std::array<int, 3>& mapping);

    /**
     * @brief 应用坐标轴映射到位置向量
     */
    Eigen::Vector3d applyAxisMapping(const Eigen::Vector3d& vec) const;

    /**
     * @brief 应用坐标轴映射到四元数（对应的旋转变换）
     */
    Eigen::Quaterniond applyAxisMappingToQuaternion(const Eigen::Quaterniond& quat) const;

    // 配置参数
    double scale_factor_;                      // 比例缩放因子
    std::array<int, 3> axis_mapping_;          // 坐标轴映射（1-based索引）
    Eigen::Quaterniond tool_orientation_offset_;  // 目标帧 -> 工具坐标系固定旋转

    // 工作空间边界限制
    WorkspaceLimiter workspace_limiter_;

    // 机器人Home位姿（启动时的初始位置）
    Eigen::Vector3d robot_home_position_;
    Eigen::Quaterniond robot_home_orientation_;

    // 机器人参考位姿（锁定时的起点，初始=Home，重新锁定时=上次目标位姿）
    Eigen::Vector3d robot_reference_position_;
    Eigen::Quaterniond robot_reference_orientation_;

    // VR参考起点（相对运动的基准）
    bool reference_locked_;
    Eigen::Vector3d vr_reference_position_;
    Eigen::Quaterniond vr_reference_orientation_;

    // 上次的目标位姿（用于未锁定时保持不动）
    mutable Eigen::Vector3d last_target_position_;
    mutable Eigen::Quaterniond last_target_orientation_;
};

} // namespace teleop_vr
