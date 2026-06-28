#pragma once

#include <string>

#include <Eigen/Dense>

namespace teleop_vr {

class WorkspaceLimiter {
public:
    WorkspaceLimiter();

    void setLimits(bool enable, double max_radius, const std::string& boundary_type = "clamp");

    bool isEnabled() const { return workspace_limit_enabled_; }
    double getMaxRadius() const { return max_workspace_radius_; }
    std::string getBoundaryType() const { return boundary_type_; }

    void setCenter(const double center[3]);
    void setCenter(const Eigen::Vector3d& center);
    Eigen::Vector3d getCenter() const { return workspace_center_position_; }

    Eigen::Vector3d apply(const Eigen::Vector3d& position) const;

private:
    Eigen::Vector3d clampToWorkspace(const Eigen::Vector3d& position) const;
    Eigen::Vector3d saturateToWorkspace(const Eigen::Vector3d& position) const;

    bool workspace_limit_enabled_;
    double max_workspace_radius_;
    std::string boundary_type_;
    Eigen::Vector3d workspace_center_position_;
};

} // namespace teleop_vr
