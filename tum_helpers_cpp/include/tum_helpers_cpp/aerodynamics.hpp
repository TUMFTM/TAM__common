// Copyright 2024 Maximilian Leitenstern
#pragma once

#include "tum_helpers_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"
#include "tum_types_cpp/vehicle.hpp"
namespace tam::helpers::aeordynamics
{
inline tam::types::vehicle_params::AeroModelOutput eval_model(
  const tam::types::common::Vector3D<double> & velocity,
  const tam::types::vehicle_params::Aero & params)
{
  tam::types::vehicle_params::AeroModelOutput out;

  out.force_cog.x = -0.5 * params.drag_coeff * params.air_density * params.cross_track_area *
            std::pow(velocity.x, 2) * tam::helpers::sgn<double>(velocity.x);
  // To be precise the area of calculating the drag here is different for lateral movement
  // but this is neglected here
  out.force_cog.y = -0.5 * params.drag_coeff * params.air_density * params.cross_track_area *
            std::pow(velocity.y, 2) * tam::helpers::sgn<double>(velocity.y);
  out.force_cog.z = 0.5 * params.lift_coeff * params.air_density * params.cross_track_area *
            std::pow(velocity.x, 2);

  // Aero Balance
  out.torque.y = -out.force_cog.x * params.diff_cog_z - out.force_cog.z * params.diff_cog_x;
  return out;
}
}  // namespace tam::helpers::aeordynamics
