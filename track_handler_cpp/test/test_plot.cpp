// Copyright 2023 Simon Hoffmann
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "matplotlib_cpp/matplotlibcpp.hpp"
#include "track_handler_cpp/race_track_handler.hpp"
namespace plt = matplotlibcpp;
int main()
{
  auto raceTrackHandler = tam::common::RaceTrackHandler::from_pkg_config();
  auto th = raceTrackHandler->create_track_prediction();
  auto rl = raceTrackHandler->create_raceline_prediction();
  auto pit = raceTrackHandler->create_pitlane_prediction();

  std::cout << "sim_start_pos = " << raceTrackHandler->get_sim_start_pos(1).at(0) << ", "
            << raceTrackHandler->get_sim_start_pos(1).at(1) << ", "
            << raceTrackHandler->get_sim_start_pos(1).at(2) << std::endl;
  std::cout << "geodetic_origin = " << raceTrackHandler->get_geo_origin().x << ", "
            << raceTrackHandler->get_geo_origin().y << ", " << raceTrackHandler->get_geo_origin().z
            << std::endl;
  std::cout << "initial_heading = " << raceTrackHandler->get_initial_heading() << std::endl;
  // Plot
  plt::figure();
  plt::plot(th->ref_line_x(), th->ref_line_y(), "b-", {{"label", "Track-Refline"}});
  plt::plot(pit->ref_line_x(), pit->ref_line_y(), "b--", {{"label", "Pit-Refine"}});
  plt::plot(th->left_bound_x(), th->left_bound_y(), "k-", {{"label", "LeftBound"}});
  plt::plot(th->right_bound_x(), th->right_bound_y(), "k-", {{"label", "RightBound"}});
  Eigen::MatrixXd rl_xyz = th->sn2cartesian(rl->s(), rl->n());
  plt::plot(rl_xyz.col(0), rl_xyz.col(1), "r-", {{"label", "Raceline"}});
  plt::xlabel("x");
  plt::ylabel("y");
  plt::legend();
  plt::axis("equal");

  plt::show();
  return 0;
}
