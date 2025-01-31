// Copyright 2023 Simon Hoffmann
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

enum class TrackReferenceLines{
  RACELINE,
  CENTERLINE,
};
struct RacelineData
{
  enum Key { s, v, n, chi, ax, ay, jx, jy, NUM_KEYS };
  std::array<Eigen::VectorXd, Key::NUM_KEYS> data;
};
struct TrackData
{
  enum Key {
    s,
    x,
    y,
    z,
    theta,
    mu,
    phi,
    dtheta,
    dmu,
    dphi,
    omega_x,
    omega_y,
    omega_z,
    domega_x,
    domega_y,
    domega_z,
    tb_left_x,
    tb_left_y,
    tb_left_z,
    tb_right_x,
    tb_right_y,
    tb_right_z,
    normal_x,
    normal_y,
    normal_z,
    w_right,
    w_left,
    NUM_KEYS
  };
  std::array<Eigen::VectorXd, Key::NUM_KEYS> data;
};