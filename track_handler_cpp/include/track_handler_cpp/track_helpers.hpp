// Copyright 2023 Simon Hoffmann
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "tum_helpers_cpp/geometry/geometry.hpp"
namespace tam::helpers::track
{
/**
 * @brief Returns normalized s in the range [0, track_length]
 *
 * @param s
 * @return Eigen::MatrixXd
 */
inline double s_mod(const double & s, const double & s_max)
{
  if (s < 0) {
    return s_max - std::fmod(std::abs(s), s_max);
  }
  return std::fmod(s, s_max);
}
inline Eigen::MatrixXd s_mod(const Eigen::Ref<const Eigen::MatrixXd> s, const double & s_max)
{
  Eigen::MatrixXd f;
  f.resize(s.rows(), s.cols());
  auto f_it = f.reshaped();
  auto s_it = s.reshaped();
  for (int i = 0; i < f.size(); ++i) {
    f_it(i) = tam::helpers::track::s_mod(s_it(i), s_max);
  }
  return f_it.reshaped(s.rows(), s.cols());
}
inline std::vector<double> s_mod(const std::vector<double> & s, const double & s_max)
{
  std::vector<double> f;
  f.reserve(s.size());
  for (const auto & s_ : s) {
    f.push_back(tam::helpers::track::s_mod(s_, s_max));
  }
  return f;
}
inline bool ahead_of_ref(const double ref_s, const double s, const double s_max)
{
  double interval_begin = s_mod(ref_s, s_max);
  double interval_end = s_mod(interval_begin + 0.5 * s_max, s_max);
  if (tam::helpers::geometry::in_interval(s, interval_begin, interval_end)) {
    return true;
  }
  return false;  // if not ahead -> behind
}
template <typename T>
inline bool has_equal_sized_data(const T& data){
  for (size_t i = 0; i < data.size() - 1; i++) {
    if (data.at(i).size() != data.at(i + 1).size()) {
      return false;
    }
  }
  return true;
}
}  // namespace tam::helpers::track
