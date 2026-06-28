#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <utility>
#include <Eigen/Geometry>

namespace teleop_vr {

/**
 * @brief 平滑滤波器类型
 */
enum class SmootherType {
    NONE,           // 不滤波
    MOVING_AVG,     // 简单移动平均
    EXP_MOVING_AVG, // 指数移动平均（EMA，推荐）
    KALMAN          // 简化卡尔曼滤波
};

/**
 * @brief 一维数据的指数移动平均滤波器
 */
class ExponentialMovingAverage {
public:
    explicit ExponentialMovingAverage(float alpha = 0.3f)
        : alpha_(std::clamp(alpha, 0.0f, 1.0f))
        , initialized_(false)
        , smoothed_value_(0.0f)
    {
    }

    float filter(float raw_value) {
        if (!initialized_) {
            smoothed_value_ = raw_value;
            initialized_ = true;
        } else {
            smoothed_value_ = alpha_ * raw_value + (1.0f - alpha_) * smoothed_value_;
        }
        return smoothed_value_;
    }

    void reset() {
        initialized_ = false;
        smoothed_value_ = 0.0f;
    }

    void setAlpha(float alpha) {
        alpha_ = std::clamp(alpha, 0.0f, 1.0f);
    }

private:
    float alpha_;
    bool initialized_;
    float smoothed_value_;
};

/**
 * @brief 简单移动平均滤波器
 */
class MovingAverage {
public:
    explicit MovingAverage(std::size_t window_size = 5)
        : window_size_(std::max<std::size_t>(1, window_size))
        , sum_(0.0f)
    {
    }

    float filter(float raw_value) {
        buffer_.push_back(raw_value);
        sum_ += raw_value;

        if (buffer_.size() > window_size_) {
            sum_ -= buffer_.front();
            buffer_.pop_front();
        }

        return sum_ / static_cast<float>(buffer_.size());
    }

    void reset() {
        buffer_.clear();
        sum_ = 0.0f;
    }

    void setWindowSize(std::size_t size) {
        window_size_ = std::max<std::size_t>(1, size);
        reset();
    }

private:
    std::size_t window_size_;
    std::deque<float> buffer_;
    float sum_;
};

/**
 * @brief 一维简化卡尔曼滤波器
 */
class SimpleKalman1D {
public:
    explicit SimpleKalman1D(float process_noise = 1e-4f, float measurement_noise = 5e-3f)
        : q_(std::max(process_noise, 1e-8f))
        , r_(std::max(measurement_noise, 1e-8f))
        , initialized_(false)
        , x_(0.0f)
        , p_(1.0f)
    {
    }

    float filter(float measurement) {
        if (!initialized_) {
            x_ = measurement;
            initialized_ = true;
            return x_;
        }

        p_ += q_;
        const float k = p_ / (p_ + r_);
        x_ = x_ + k * (measurement - x_);
        p_ = (1.0f - k) * p_;
        return x_;
    }

    void reset() {
        initialized_ = false;
        x_ = 0.0f;
        p_ = 1.0f;
    }

    void setFromAlpha(float alpha) {
        const float bounded_alpha = std::clamp(alpha, 0.0f, 1.0f);
        q_ = std::max(1e-6f, bounded_alpha * 1e-2f);
        r_ = std::max(1e-6f, (1.0f - bounded_alpha + 1e-3f) * 1e-1f);
    }

private:
    float q_;
    float r_;
    bool initialized_;
    float x_;
    float p_;
};

/**
 * @brief 标量滤波器封装，真正根据 smoother_type_ 切换算法
 */
class ScalarSmoother {
public:
    ScalarSmoother(SmootherType type, float alpha)
        : type_(type)
        , alpha_(std::clamp(alpha, 0.0f, 1.0f))
    {
        rebuildFilter();
    }

    float filter(float value) {
        switch (type_) {
            case SmootherType::NONE:
                return value;
            case SmootherType::MOVING_AVG:
                return moving_avg_->filter(value);
            case SmootherType::EXP_MOVING_AVG:
                return ema_->filter(value);
            case SmootherType::KALMAN:
                return kalman_->filter(value);
            default:
                return value;
        }
    }

    void reset() {
        if (ema_) {
            ema_->reset();
        }
        if (moving_avg_) {
            moving_avg_->reset();
        }
        if (kalman_) {
            kalman_->reset();
        }
    }

    void setAlpha(float alpha) {
        alpha_ = std::clamp(alpha, 0.0f, 1.0f);
        if (ema_) {
            ema_->setAlpha(alpha_);
        }
        if (moving_avg_) {
            moving_avg_->setWindowSize(alphaToWindowSize(alpha_));
        }
        if (kalman_) {
            kalman_->setFromAlpha(alpha_);
        }
    }

private:
    static std::size_t alphaToWindowSize(float alpha) {
        const float bounded_alpha = std::clamp(alpha, 0.01f, 1.0f);
        const int window = static_cast<int>(std::round((1.0f / bounded_alpha) + 1.0f));
        return static_cast<std::size_t>(std::clamp(window, 2, 30));
    }

    void rebuildFilter() {
        switch (type_) {
            case SmootherType::MOVING_AVG:
                moving_avg_ = std::make_unique<MovingAverage>(alphaToWindowSize(alpha_));
                break;
            case SmootherType::EXP_MOVING_AVG:
                ema_ = std::make_unique<ExponentialMovingAverage>(alpha_);
                break;
            case SmootherType::KALMAN:
                kalman_ = std::make_unique<SimpleKalman1D>();
                kalman_->setFromAlpha(alpha_);
                break;
            case SmootherType::NONE:
            default:
                break;
        }
    }

    SmootherType type_;
    float alpha_;
    std::unique_ptr<ExponentialMovingAverage> ema_;
    std::unique_ptr<MovingAverage> moving_avg_;
    std::unique_ptr<SimpleKalman1D> kalman_;
};

class VrDataSmoother {
public:
    explicit VrDataSmoother(
        bool enabled = true,
        SmootherType type = SmootherType::EXP_MOVING_AVG,
        float position_alpha = 0.3f,
        float rotation_alpha = 0.5f,
        float input_alpha = 0.2f
    )
        : enabled_(enabled)
        , smoother_type_(type)
        , position_alpha_(std::clamp(position_alpha, 0.0f, 1.0f))
        , rotation_alpha_(std::clamp(rotation_alpha, 0.0f, 1.0f))
        , input_alpha_(std::clamp(input_alpha, 0.0f, 1.0f))
    {
        for (std::size_t i = 0; i < 3; ++i) {
            left_position_smoother_[i] = std::make_unique<ScalarSmoother>(smoother_type_, position_alpha_);
            right_position_smoother_[i] = std::make_unique<ScalarSmoother>(smoother_type_, position_alpha_);
            headset_position_smoother_[i] = std::make_unique<ScalarSmoother>(smoother_type_, position_alpha_);
        }

        for (std::size_t i = 0; i < 7; ++i) {
            left_input_smoother_[i] = std::make_unique<ScalarSmoother>(smoother_type_, input_alpha_);
            right_input_smoother_[i] = std::make_unique<ScalarSmoother>(smoother_type_, input_alpha_);
        }
    }

    std::pair<std::array<float, 3>, std::array<float, 4>> smoothLeftController(
        const float position[3],
        const float rotation[4]
    ) {
        return smoothPose(left_position_smoother_, left_rotation_state_, position, rotation);
    }

    std::pair<std::array<float, 3>, std::array<float, 4>> smoothRightController(
        const float position[3],
        const float rotation[4]
    ) {
        return smoothPose(right_position_smoother_, right_rotation_state_, position, rotation);
    }

    std::pair<std::array<float, 3>, std::array<float, 4>> smoothHeadset(
        const float position[3],
        const float rotation[4]
    ) {
        return smoothPose(headset_position_smoother_, headset_rotation_state_, position, rotation);
    }

    std::array<float, 7> smoothLeftInput(const float input[7]) {
        if (!enabled_ || smoother_type_ == SmootherType::NONE) {
            return {input[0], input[1], input[2], input[3], input[4], input[5], input[6]};
        }

        std::array<float, 7> smoothed;
        for (std::size_t i = 0; i < 7; ++i) {
            smoothed[i] = left_input_smoother_[i]->filter(input[i]);
        }
        return smoothed;
    }

    std::array<float, 7> smoothRightInput(const float input[7]) {
        if (!enabled_ || smoother_type_ == SmootherType::NONE) {
            return {input[0], input[1], input[2], input[3], input[4], input[5], input[6]};
        }

        std::array<float, 7> smoothed;
        for (std::size_t i = 0; i < 7; ++i) {
            smoothed[i] = right_input_smoother_[i]->filter(input[i]);
        }
        return smoothed;
    }

    void reset() {
        resetScalarGroup(left_position_smoother_);
        resetScalarGroup(right_position_smoother_);
        resetScalarGroup(headset_position_smoother_);
        resetScalarGroup(left_input_smoother_);
        resetScalarGroup(right_input_smoother_);

        left_rotation_state_.initialized = false;
        right_rotation_state_.initialized = false;
        headset_rotation_state_.initialized = false;
    }

    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

    void setPositionAlpha(float alpha) {
        position_alpha_ = std::clamp(alpha, 0.0f, 1.0f);
        setAlphaForGroup(left_position_smoother_, position_alpha_);
        setAlphaForGroup(right_position_smoother_, position_alpha_);
        setAlphaForGroup(headset_position_smoother_, position_alpha_);
    }

    void setRotationAlpha(float alpha) {
        rotation_alpha_ = std::clamp(alpha, 0.0f, 1.0f);
    }

    void setInputAlpha(float alpha) {
        input_alpha_ = std::clamp(alpha, 0.0f, 1.0f);
        setAlphaForGroup(left_input_smoother_, input_alpha_);
        setAlphaForGroup(right_input_smoother_, input_alpha_);
    }

private:
    struct QuaternionSmootherState {
        bool initialized = false;
        Eigen::Quaternionf value = Eigen::Quaternionf::Identity();
    };

    template<std::size_t N>
    static void resetScalarGroup(std::array<std::unique_ptr<ScalarSmoother>, N>& group) {
        for (auto& smoother : group) {
            smoother->reset();
        }
    }

    template<std::size_t N>
    static void setAlphaForGroup(std::array<std::unique_ptr<ScalarSmoother>, N>& group, float alpha) {
        for (auto& smoother : group) {
            smoother->setAlpha(alpha);
        }
    }

    std::array<float, 4> smoothQuaternion(QuaternionSmootherState& state, const float rotation[4]) {
        Eigen::Quaternionf current(rotation[3], rotation[0], rotation[1], rotation[2]);
        if (!std::isfinite(current.x()) || !std::isfinite(current.y()) ||
            !std::isfinite(current.z()) || !std::isfinite(current.w())) {
            if (!state.initialized) {
                return {0.0f, 0.0f, 0.0f, 1.0f};
            }
            return {state.value.x(), state.value.y(), state.value.z(), state.value.w()};
        }

        const float norm = current.norm();
        if (norm < 1e-6f) {
            if (!state.initialized) {
                return {0.0f, 0.0f, 0.0f, 1.0f};
            }
            return {state.value.x(), state.value.y(), state.value.z(), state.value.w()};
        }

        current.normalize();

        if (!state.initialized || smoother_type_ == SmootherType::NONE) {
            state.value = current;
            state.initialized = true;
            return {current.x(), current.y(), current.z(), current.w()};
        }

        if (state.value.dot(current) < 0.0f) {
            current.coeffs() *= -1.0f;
        }

        const float blend = std::clamp(rotation_alpha_, 0.0f, 1.0f);
        state.value = state.value.slerp(blend, current);
        state.value.normalize();
        return {state.value.x(), state.value.y(), state.value.z(), state.value.w()};
    }

    std::pair<std::array<float, 3>, std::array<float, 4>> smoothPose(
        std::array<std::unique_ptr<ScalarSmoother>, 3>& position_smoother,
        QuaternionSmootherState& quaternion_state,
        const float position[3],
        const float rotation[4]
    ) {
        std::array<float, 3> smoothed_pos = {position[0], position[1], position[2]};
        if (enabled_ && smoother_type_ != SmootherType::NONE) {
            for (std::size_t i = 0; i < 3; ++i) {
                smoothed_pos[i] = position_smoother[i]->filter(position[i]);
            }
        }

        std::array<float, 4> smoothed_rot = smoothQuaternion(quaternion_state, rotation);
        return {smoothed_pos, smoothed_rot};
    }

    bool enabled_;
    SmootherType smoother_type_;
    float position_alpha_;
    float rotation_alpha_;
    float input_alpha_;

    std::array<std::unique_ptr<ScalarSmoother>, 3> left_position_smoother_;
    std::array<std::unique_ptr<ScalarSmoother>, 3> right_position_smoother_;
    std::array<std::unique_ptr<ScalarSmoother>, 3> headset_position_smoother_;

    std::array<std::unique_ptr<ScalarSmoother>, 7> left_input_smoother_;
    std::array<std::unique_ptr<ScalarSmoother>, 7> right_input_smoother_;

    QuaternionSmootherState left_rotation_state_;
    QuaternionSmootherState right_rotation_state_;
    QuaternionSmootherState headset_rotation_state_;
};

} // namespace teleop_vr

