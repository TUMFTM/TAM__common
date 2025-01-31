// Copyright 2024 Maximilian Leitenstern
#pragma once

#include <vector>
namespace tam::types::tire_params
{
struct VelocityScaling
{
  std::vector<double> velocity{};
  std::vector<double> factor{};
};
struct Radius
{
  double radius_20mps{0.0};
  VelocityScaling velocity_scaling;
};
struct Setup
{
  double camber{0.0};
  double caster{0.0};
  double toe{0.0};
};
struct Tire
{
  Radius radius;
  Setup setup;
};
}  // namespace tam::types::tire_params
