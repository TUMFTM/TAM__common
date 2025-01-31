// Copyright 2023 Simon Hoffmann
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "track_handler_cpp/race_track_handler.hpp"
#include "track_handler_cpp/track.hpp"
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::microseconds;
int main()
{
  auto th = tam::common::RaceTrackHandler::from_pkg_config()->create_track();
  // Convert to Frenet
  Eigen::Vector2d sn;
  int samples = 1000;
  auto start = high_resolution_clock::now();
  for (int i = 0; i < samples; i++) {
    sn = th->project_2d_point_on_track(-131.019574, 369.886773);
  }
  auto stop = high_resolution_clock::now();

  auto duration = duration_cast<microseconds>(stop - start);
  // TOdo (Simon H.): calculated s is slightly different from s in trackfile
  std::cout << "s: " << sn[0] << "; n: " << sn[1] << "; idx: " << std::endl;
  std::cout << "Time: " << duration.count() * 1e-6
            << "s ; Duration per Execution: " << duration.count() * 1e-3 / samples << "ms"
            << std::endl;

  // Convert back
  Eigen::Vector3d xy;
  auto start2 = high_resolution_clock::now();
  for (int i = 0; i < samples; i++) {
    xy = th->sn2cartesian(25.992750, 0.0);
  }
  auto stop2 = high_resolution_clock::now();

  auto duration2 = duration_cast<microseconds>(stop2 - start2);
  // TOdo (Simon H.): calculated s is slightly different from s in trackfile
  std::cout << "x: " << xy[0] << "; y: " << xy[1] << std::endl;
  std::cout << "Time: " << duration2.count() * 1e-6
            << "s ; Duration per Execution: " << duration2.count() * 1e-3 / samples << "ms"
            << std::endl;
  return 0;
}
