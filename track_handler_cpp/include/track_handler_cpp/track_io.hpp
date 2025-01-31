// Copyright 2023 Simon Hoffmann
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <optional>
#include <vector>

#include "track_handler_cpp/track_types.hpp"
#include "tum_helpers_cpp/file_handling.hpp"
namespace tam::common
{
static std::optional<RacelineData::Key> get_raceline_key_from_header(const std::string & header)
{
  if (header == "s_ref_rl_m") {
    return RacelineData::s;
  }
  if (header == "v_rl_mps") {
    return RacelineData::v;
  }
  if (header == "n_rl_m") {
    return RacelineData::n;
  }
  if (header == "chi_rl_rad") {
    return RacelineData::chi;
  }
  if (header == "ax_rl_mps2") {
    return RacelineData::ax;
  }
  if (header == "ay_rl_mps2") {
    return RacelineData::ay;
  }
  if (header == "jx_rl_mps3") {
    return RacelineData::jx;
  }
  if (header == "jy_rl_mps3") {
    return RacelineData::jy;
  }
  return std::nullopt;
}
static std::optional<TrackData::Key> get_track_key_from_header(
  const std::string & header, TrackReferenceLines ref_line = TrackReferenceLines::RACELINE)
{
  std::string ref = "rl";
  switch (ref_line) {
    case TrackReferenceLines::RACELINE:
      ref = "rl";
      break;
    case TrackReferenceLines::CENTERLINE:
      ref = "cl";
      break;
    default:
      throw std::invalid_argument(
        "[track_handler_cpp]: Invalid Reference line specified for loading Track");
      break;
  }

  if (header == "s_ref_" + ref + "_m") {
    return TrackData::s;
  }
  if (header == "x_ref_" + ref + "_m") {
    return TrackData::x;
  }
  if (header == "y_ref_" + ref + "_m") {
    return TrackData::y;
  }
  if (header == "z_ref_" + ref + "_m") {
    return TrackData::z;
  }
  if (header == "theta_ref_" + ref + "_rad") {
    return TrackData::theta;
  }
  if (header == "mu_ref_" + ref + "_rad") {
    return TrackData::mu;
  }
  if (header == "phi_ref_" + ref + "_rad") {
    return TrackData::phi;
  }
  if (header == "dtheta_ref_" + ref + "_radpm") {
    return TrackData::dtheta;
  }
  if (header == "dmu_ref_" + ref + "_radpm") {
    return TrackData::dmu;
  }
  if (header == "dphi_ref_" + ref + "_radpm") {
    return TrackData::dphi;
  }
  if (header == "omega_x_ref_" + ref + "_radpm") {
    return TrackData::omega_x;
  }
  if (header == "omega_y_ref_" + ref + "_radpm") {
    return TrackData::omega_y;
  }
  if (header == "omega_z_ref_" + ref + "_radpm") {
    return TrackData::omega_z;
  }
  if (header == "w_tr_right_ref_" + ref + "_m") {
    return TrackData::w_right;
  }
  if (header == "w_tr_left_ref_" + ref + "_m") {
    return TrackData::w_left;
  }

  return std::nullopt;
}
static RacelineData get_raceline_from_file(const std::string & path)
{
  Eigen::MatrixXd test = tam::helpers::files::load_csv<Eigen::MatrixXd>(path);
  std::vector<std::string> header = tam::helpers::files::get_csv_header(path);

  RacelineData data_;
  for (size_t i = 0; i < header.size(); i++) {
    auto key = get_raceline_key_from_header(header.at(i));
    if (key.has_value()) {
      // Remove nan
      auto iterator = std::find_if(
        test.col(i).begin(), test.col(i).end(), [](auto & i) { return std::isnan(i); });
      int idx = std::distance(test.col(i).begin(), iterator);
      data_.data.at(key.value()) = test.col(i).head(idx);
    }
  }
  return data_;
}
static TrackData get_track_from_file(
  const std::string & path, TrackReferenceLines ref = TrackReferenceLines::RACELINE)
{
  Eigen::MatrixXd test = tam::helpers::files::load_csv<Eigen::MatrixXd>(path);
  std::vector<std::string> header = tam::helpers::files::get_csv_header(path);

  TrackData data_;
  for (size_t i = 0; i < header.size(); i++) {
    auto key = get_track_key_from_header(header.at(i), ref);
    if (key.has_value()) {
      // Remove nan
      auto iterator = std::find_if(
        test.col(i).begin(), test.col(i).end(), [](auto & i) { return std::isnan(i); });
      int idx = std::distance(test.col(i).begin(), iterator);
      data_.data.at(key.value()) = test.col(i).head(idx);
    }
  }
  return data_;
}
}  // namespace tam::common