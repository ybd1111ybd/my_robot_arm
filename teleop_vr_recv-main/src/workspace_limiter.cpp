#include "teleop_vr_recv/workspace_limiter.h"

#include <cmath>
#include <iostream>

namespace teleop_vr {

WorkspaceLimiter::WorkspaceLimiter()
    : workspace_limit_enabled_(false)
    , max_workspace_radius_(1.0)
    , boundary_type_("clamp")
    , workspace_center_position_(Eigen::Vector3d::Zero())
{
}

void WorkspaceLimiter::setLimits(bool enable, double max_radius, const std::string& boundary_type)
{
    workspace_limit_enabled_ = enable;

    if (!std::isfinite(max_radius) || max_radius <= 0.0) {
        std::cerr << "Warning: invalid max_workspace_radius " << max_radius
                  << ", fallback to 1.0" << std::endl;
        max_workspace_radius_ = 1.0;
    } else {
        max_workspace_radius_ = max_radius;
    }

    if (boundary_type == "clamp" || boundary_type == "saturate") {
        boundary_type_ = boundary_type;
    } else {
        std::cerr << "Warning: invalid boundary_type '" << boundary_type
                  << "', fallback to clamp" << std::endl;
        boundary_type_ = "clamp";
    }
}

void WorkspaceLimiter::setCenter(const double center[3])
{
    setCenter(Eigen::Vector3d(center[0], center[1], center[2]));
}

void WorkspaceLimiter::setCenter(const Eigen::Vector3d& center)
{
    if (!center.allFinite()) {
        std::cerr << "Warning: invalid workspace_center (contains NaN/Inf), keep previous center ["
                  << workspace_center_position_.x() << ", "
                  << workspace_center_position_.y() << ", "
                  << workspace_center_position_.z() << "]"
                  << std::endl;
        return;
    }
    workspace_center_position_ = center;
}

Eigen::Vector3d WorkspaceLimiter::apply(const Eigen::Vector3d& position) const
{
    if (!workspace_limit_enabled_) {
        return position;
    }

    if (boundary_type_ == "clamp") {
        return clampToWorkspace(position);
    }
    if (boundary_type_ == "saturate") {
        return saturateToWorkspace(position);
    }
    return clampToWorkspace(position);
}

Eigen::Vector3d WorkspaceLimiter::clampToWorkspace(const Eigen::Vector3d& position) const
{
    Eigen::Vector3d position_from_center = position - workspace_center_position_;
    const double distance = position_from_center.norm();

    if (distance > max_workspace_radius_) {
        position_from_center = position_from_center.normalized() * max_workspace_radius_;
    }

    return workspace_center_position_ + position_from_center;
}

Eigen::Vector3d WorkspaceLimiter::saturateToWorkspace(const Eigen::Vector3d& position) const
{
    Eigen::Vector3d position_from_center = position - workspace_center_position_;
    const double distance = position_from_center.norm();

    if (distance < 1e-6) {
        return position;
    }

    const double normalized = std::max(0.0, distance / max_workspace_radius_);
    const double saturated_distance = max_workspace_radius_ * std::tanh(normalized);
    position_from_center = position_from_center.normalized() * saturated_distance;

    return workspace_center_position_ + position_from_center;
}

} // namespace teleop_vr
