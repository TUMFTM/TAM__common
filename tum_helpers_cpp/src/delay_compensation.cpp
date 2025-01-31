// Copyright 2023 Marcel Weinmann
#include "tum_helpers_cpp/delay_compensation.hpp"

tam::helpers::DelayCompensation::DelayCompensation(int buffer_size, double step_size)
{
    if (step_size > 1e-06) {
        step_size_ = step_size;
        init_cycle_length_ = buffer_size;
    } else {
        throw std::invalid_argument(
        "[DelayCompensation]: step_size has to be larger than 1e-06");
    }
    position_fifo_.resize(buffer_size);
    orientation_fifo_.resize(buffer_size);
}

/**
 * @brief Update the internal deque containing the last N state estimation outputs
 *
 * @param[in] input         - tam::types::control::Odometry:
 *                            current state estimation output
 */
void tam::helpers::DelayCompensation::update_vehicle_odometry(
    const tam::types::control::Odometry & se_odometry)
{
    if (init_cycle_length_ != 0) { init_cycle_length_--; }

    // update the state estimation FiFo buffer
    position_fifo_.push_front(se_odometry.position_m);
    position_fifo_.pop_back();

    orientation_fifo_.push_front(se_odometry.orientation_rad);
    orientation_fifo_.pop_back();
}

/**
 * @brief Compensate the delay of the input odometry
 *
 * @param[in] input         - tam::types::control::Odometry:
 *                            current state estimation output
 * 
 * @param[in] delta_t_s     - double:
 *                            time delay to compensate in seconds
 */
bool tam::helpers::DelayCompensation::compensate(
    tam::types::control::Odometry & input_odometry, double delta_t_s)
{
    int step_delay = static_cast<int>(delta_t_s / step_size_);
    double fractional_step_delay = delta_t_s / step_size_ - step_delay;

    // catch negative delta_t_s
    if (step_delay < 0 || fractional_step_delay < 0.0 || init_cycle_length_ != 0) {
        return true;
    }

    if (position_fifo_.size() > static_cast<size_t>(step_delay + 1)) {
        // compensate delay in position
        input_odometry.position_m.x += position_fifo_[0].x - position_fifo_[step_delay].x
            + (position_fifo_[step_delay].x - position_fifo_[step_delay + 1].x)
            * fractional_step_delay;
        input_odometry.position_m.y += position_fifo_[0].y - position_fifo_[step_delay].y
            + (position_fifo_[step_delay].y - position_fifo_[step_delay + 1].y)
            * fractional_step_delay;
        input_odometry.position_m.z += position_fifo_[0].z - position_fifo_[step_delay].z
            + (position_fifo_[step_delay].z - position_fifo_[step_delay + 1].z)
            * fractional_step_delay;

        // compensate delay in orientation
        input_odometry.orientation_rad.x = tam::helpers::geometry::normalize_angle(
            input_odometry.orientation_rad.x + orientation_fifo_[0].x
            - orientation_fifo_[step_delay].x + tam::helpers::geometry::normalize_angle(
                (orientation_fifo_[step_delay].x - orientation_fifo_[step_delay + 1].x))
            * fractional_step_delay);
        input_odometry.orientation_rad.y = tam::helpers::geometry::normalize_angle(
            input_odometry.orientation_rad.y + orientation_fifo_[0].y
            - orientation_fifo_[step_delay].y + tam::helpers::geometry::normalize_angle(
                (orientation_fifo_[step_delay].y - orientation_fifo_[step_delay + 1].y))
            * fractional_step_delay);
        input_odometry.orientation_rad.z = tam::helpers::geometry::normalize_angle(
            input_odometry.orientation_rad.z + orientation_fifo_[0].z
            - orientation_fifo_[step_delay].z + tam::helpers::geometry::normalize_angle(
                (orientation_fifo_[step_delay].z - orientation_fifo_[step_delay + 1].z))
            * fractional_step_delay);

        // delay was successfully compensated
        return true;
    } else {
        return false;
    }
}
