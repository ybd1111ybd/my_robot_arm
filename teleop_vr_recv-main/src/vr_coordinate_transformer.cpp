#include "teleop_vr_recv/vr_coordinate_transformer.h"
#include <cmath>
#include <iostream>

namespace teleop_vr {

VrCoordinateTransformer::VrCoordinateTransformer(double scale_factor)
    : scale_factor_(scale_factor)
    , axis_mapping_{{1, 2, 3}}  // 默认不映射（1-based索引）
    , tool_orientation_offset_(Eigen::Quaterniond::Identity())
    , robot_home_position_(Eigen::Vector3d::Zero())
    , robot_home_orientation_(Eigen::Quaterniond::Identity())
    , robot_reference_position_(Eigen::Vector3d::Zero())
    , robot_reference_orientation_(Eigen::Quaterniond::Identity())
    , reference_locked_(false)
    , vr_reference_position_(Eigen::Vector3d::Zero())
    , vr_reference_orientation_(Eigen::Quaterniond::Identity())
    , last_target_position_(Eigen::Vector3d::Zero())
    , last_target_orientation_(Eigen::Quaterniond::Identity())
{
}

void VrCoordinateTransformer::setWorkspaceCenter(const double center[3])
{
    workspace_limiter_.setCenter(center);
}

void VrCoordinateTransformer::setWorkspaceCenter(const Eigen::Vector3d& center)
{
    workspace_limiter_.setCenter(center);
}

void VrCoordinateTransformer::setRobotHomePose(const double position[3],
                                               const double orientation[4])
{
    robot_home_position_ = Eigen::Vector3d(position[0], position[1], position[2]);
    robot_home_orientation_ = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]);
    robot_home_orientation_.normalize();

    // 初始化机器人参考位姿和上次目标位姿为Home
    robot_reference_position_ = robot_home_position_;
    robot_reference_orientation_ = robot_home_orientation_;
    last_target_position_ = robot_home_position_;
    last_target_orientation_ = robot_home_orientation_;
}

void VrCoordinateTransformer::lockVrReference(const float vr_position[3],
                                              const float vr_orientation[4])
{
    // 记录当前VR位姿作为新的参考起点
    vr_reference_position_ = Eigen::Vector3d(vr_position[0], vr_position[1], vr_position[2]);
    vr_reference_orientation_ = Eigen::Quaterniond(vr_orientation[3], vr_orientation[0],
                                                   vr_orientation[1], vr_orientation[2]);
    vr_reference_orientation_.normalize();

    // 将上次的目标位姿作为新的机器人参考起点
    // 这样重新锁定时，机器人从当前位置继续，而不是跳回Home
    robot_reference_position_ = last_target_position_;
    robot_reference_orientation_ = last_target_orientation_;

    reference_locked_ = true;
}

RobotEndEffectorPose VrCoordinateTransformer::transform(const float vr_position[3],
                                                        const float vr_orientation[4])
{
    RobotEndEffectorPose result;

    // 如果没有锁定参考起点，返回上次的目标位姿（保持不动）
    if (!reference_locked_) {
        result.position[0] = last_target_position_.x();
        result.position[1] = last_target_position_.y();
        result.position[2] = last_target_position_.z();
        result.orientation[0] = last_target_orientation_.x();
        result.orientation[1] = last_target_orientation_.y();
        result.orientation[2] = last_target_orientation_.z();
        result.orientation[3] = last_target_orientation_.w();
        return result;
    }

    // 1. 计算VR位置相对于参考起点的增量
    Eigen::Vector3d vr_current_position(vr_position[0], vr_position[1], vr_position[2]);
    Eigen::Vector3d vr_position_delta = vr_current_position - vr_reference_position_;

    // 2. 应用坐标轴映射
    Eigen::Vector3d mapped_delta = applyAxisMapping(vr_position_delta);

    // 3. 应用比例缩放
    Eigen::Vector3d scaled_delta = mapped_delta * scale_factor_;

    // 4. 计算目标位置 = 机器人参考位置 + 缩放后的增量
    Eigen::Vector3d target_position = robot_reference_position_ + scaled_delta;

    // 4.5 应用工作空间边界限制
    target_position = workspace_limiter_.apply(target_position);

    // 5. 计算VR姿态相对于参考起点的增量旋转
    Eigen::Quaterniond vr_current_orientation(vr_orientation[3], vr_orientation[0],
                                              vr_orientation[1], vr_orientation[2]);
    vr_current_orientation.normalize();

    // delta_rotation = vr_current * vr_reference^-1
    Eigen::Quaterniond vr_rotation_delta = vr_current_orientation * vr_reference_orientation_.inverse();

    // 6. 应用坐标轴映射到旋转
    Eigen::Quaterniond mapped_rotation_delta = applyAxisMappingToQuaternion(vr_rotation_delta);

    // 7. 在工具/握持坐标系中解释控制器相对旋转，而不是直接作用在
    // link8 原始坐标轴上。这样左右手柄“指向”的方向能与机械臂工具保持一致。
    const Eigen::Quaterniond tool_frame_rotation_delta =
        tool_orientation_offset_ * mapped_rotation_delta *
        tool_orientation_offset_.conjugate();

    // 8. 计算目标姿态 = 机器人参考姿态 * 工具坐标系中的增量旋转
    Eigen::Quaterniond target_orientation =
        robot_reference_orientation_ * tool_frame_rotation_delta;
    target_orientation.normalize();

    // 9. 更新上次的目标位姿（用于下次未锁定时返回）
    last_target_position_ = target_position;
    last_target_orientation_ = target_orientation;

    // 10. 填充结果
    result.position[0] = target_position.x();
    result.position[1] = target_position.y();
    result.position[2] = target_position.z();
    result.orientation[0] = target_orientation.x();
    result.orientation[1] = target_orientation.y();
    result.orientation[2] = target_orientation.z();
    result.orientation[3] = target_orientation.w();

    return result;
}

void VrCoordinateTransformer::setScaleFactor(double scale_factor)
{
    if (!std::isfinite(scale_factor) || scale_factor <= 0.0) {
        std::cerr << "Warning: invalid scale_factor " << scale_factor
                  << ", keep previous value " << scale_factor_ << std::endl;
        return;
    }
    scale_factor_ = scale_factor;
}

void VrCoordinateTransformer::setAxisMapping(const std::array<int, 3>& mapping)
{
    if (!isValidAxisMapping(mapping)) {
        std::cerr << "Warning: invalid axis_mapping, keep previous mapping"
                  << " [" << axis_mapping_[0] << ", " << axis_mapping_[1] << ", " << axis_mapping_[2] << "]"
                  << std::endl;
        return;
    }
    axis_mapping_ = mapping;
}

void VrCoordinateTransformer::setToolOrientationOffset(
    const Eigen::Quaterniond& offset)
{
    if (!offset.coeffs().allFinite() || offset.norm() < 1e-9) {
        std::cerr << "Warning: invalid tool_orientation_offset, keep previous value"
                  << std::endl;
        return;
    }

    tool_orientation_offset_ = offset.normalized();
}

void VrCoordinateTransformer::resetVrReference()
{
    reference_locked_ = false;
    vr_reference_position_ = Eigen::Vector3d::Zero();
    vr_reference_orientation_ = Eigen::Quaterniond::Identity();
}

void VrCoordinateTransformer::resetToHome()
{
    // 1. 解除VR参考锁定
    reference_locked_ = false;

    // 2. 清空VR参考起点
    vr_reference_position_ = Eigen::Vector3d::Zero();
    vr_reference_orientation_ = Eigen::Quaterniond::Identity();

    // 3. 将机器人参考位姿重置为Home
    robot_reference_position_ = robot_home_position_;
    robot_reference_orientation_ = robot_home_orientation_;

    // 4. 将上次目标位姿重置为Home（关键！）
    // 这样确保下次按Grip键锁定时，lockVrReference()会将Home设为新的参考起点
    last_target_position_ = robot_home_position_;
    last_target_orientation_ = robot_home_orientation_;
}

Eigen::Vector3d VrCoordinateTransformer::applyAxisMapping(const Eigen::Vector3d& vec) const
{
    Eigen::Vector3d result;

    for (int i = 0; i < 3; ++i) {
        int target_axis_1based = axis_mapping_[i];  // 1-based: 1,2,3 或 -1,-2,-3
        int abs_axis_1based = std::abs(target_axis_1based);  // 1,2,3
        int abs_axis_0based = abs_axis_1based - 1;  // 转为 0-based: 0,1,2
        double sign = (target_axis_1based > 0) ? 1.0 : -1.0;

        result[abs_axis_0based] = vec[i] * sign;
    }

    return result;
}

Eigen::Quaterniond VrCoordinateTransformer::applyAxisMappingToQuaternion(
    const Eigen::Quaterniond& quat) const
{
    // 将四元数转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

    // 应用坐标轴映射到旋转矩阵
    Eigen::Matrix3d mapped_matrix = Eigen::Matrix3d::Zero();

    for (int i = 0; i < 3; ++i) {
        int target_axis_1based = axis_mapping_[i];
        int abs_axis_1based = std::abs(target_axis_1based);
        int abs_axis_0based = abs_axis_1based - 1;
        double sign = (target_axis_1based > 0) ? 1.0 : -1.0;

        // 映射行和列
        for (int j = 0; j < 3; ++j) {
            int target_j_1based = axis_mapping_[j];
            int abs_j_1based = std::abs(target_j_1based);
            int abs_j_0based = abs_j_1based - 1;
            double sign_j = (target_j_1based > 0) ? 1.0 : -1.0;

            mapped_matrix(abs_axis_0based, abs_j_0based) = rotation_matrix(i, j) * sign * sign_j;
        }
    }

    // 转换回四元数
    Eigen::Quaterniond result(mapped_matrix);
    result.normalize();

    return result;
}

void VrCoordinateTransformer::setWorkspaceLimits(bool enable, double max_radius, const std::string& boundary_type)
{
    workspace_limiter_.setLimits(enable, max_radius, boundary_type);
}

bool VrCoordinateTransformer::isValidAxisMapping(const std::array<int, 3>& mapping)
{
    bool used_axis[3] = {false, false, false};
    for (int axis : mapping) {
        const int abs_axis = std::abs(axis);
        if (abs_axis < 1 || abs_axis > 3) {
            return false;
        }
        if (used_axis[abs_axis - 1]) {
            return false;
        }
        used_axis[abs_axis - 1] = true;
    }
    return true;
}

RobotEndEffectorPose VrCoordinateTransformer::getRobotHomePose() const
{
    RobotEndEffectorPose home_pose;
    home_pose.position[0] = robot_home_position_.x();
    home_pose.position[1] = robot_home_position_.y();
    home_pose.position[2] = robot_home_position_.z();
    home_pose.orientation[0] = robot_home_orientation_.x();
    home_pose.orientation[1] = robot_home_orientation_.y();
    home_pose.orientation[2] = robot_home_orientation_.z();
    home_pose.orientation[3] = robot_home_orientation_.w();
    return home_pose;
}

} // namespace teleop_vr
