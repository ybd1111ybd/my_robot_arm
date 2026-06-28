#pragma once

#include <array>

#include "teleop_vr_recv/data_smoother.h"
#include "teleop_vr_recv/vr_data_parser.h"

namespace teleop_vr {

struct SmoothedVrPoses {
    std::array<float, 3> left_position;
    std::array<float, 4> left_rotation;

    std::array<float, 3> right_position;
    std::array<float, 4> right_rotation;

    std::array<float, 3> headset_position;
    std::array<float, 4> headset_rotation;
};

struct SmoothedControllerInputs {
    std::array<float, 7> left_input;
    std::array<float, 7> right_input;
};

class VrSmoothingPipeline {
public:
    explicit VrSmoothingPipeline(VrDataSmoother& smoother)
        : smoother_(smoother)
    {
    }

    SmoothedVrPoses smoothPoses(const VrDataPacket& packet);
    SmoothedControllerInputs smoothInputs(const VrDataPacket& packet);

private:
    static std::array<float, 7> toRawInputArray(const ControllerInput& input);

    VrDataSmoother& smoother_;
};

} // namespace teleop_vr

