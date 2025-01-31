// Copyright 2023 Marcel Weinmann
#pragma once

#include <deque>
#include <memory>

// tum helper
#include "tum_helpers_cpp/geometry/geometry.hpp"

// tum types
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

namespace tam::helpers
{
class DelayCompensation
{
private:
  // Variables
  /**
   * @brief FiFo containing the last N state estimation outputs for the position
   */
  std::deque<tam::types::common::Vector3D<double>> position_fifo_;

  /**
   * @brief FiFo containing the last N state estimation outputs for the orientation
   */
  std::deque<tam::types::common::Vector3D<double>> orientation_fifo_;

  /**
   * @brief step size of the state estimation input
   */
  double step_size_;

  /**
   * @brief cycles needed to fully initialize the FiFo
   */
  int init_cycle_length_;

public:
  /**
   * @brief Constructor
   */
  explicit DelayCompensation(int buffer_size = 100, double step_size = 0.01);

  /**
   * @brief Update the internal deque containing the last N state estimation outputs
   *
   * @param[in] input         - tam::types::control::Odometry:
   *                            current state estimation output
   */
  void update_vehicle_odometry(const tam::types::control::Odometry & se_odometry);

  /**
   * @brief Compensate the delay of the input odometry
   *
   * @param[in] input         - tam::types::control::Odometry:
   *                            current state estimation output
   * 
   * @param[in] delta_t_s     - double:
   *                            time delay to compensate in seconds
   * 
   * @param[out] result       - bool:
   *                            true if the delay was successfully compensated 
   */
  bool compensate(tam::types::control::Odometry & input_odometry, double delta_t_s);
};
}  // namespace tam::helpers
