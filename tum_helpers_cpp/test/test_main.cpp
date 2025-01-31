// Copyright 2023 Simon Hoffmann
#include <chrono>
#include <iostream>

#include "matplotlib_cpp/matplotlibcpp.hpp"
#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"
#include "tum_helpers_cpp/type_conversion.hpp"
namespace plt = matplotlibcpp;
int main()
{
  // Init Cosy
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(20, 0, 100);
  Eigen::VectorXd y = (x.array() * 0.1).sin().matrix() * 5.0;
  Eigen::VectorXd z = Eigen::VectorXd::Zero(x.size());
  auto cosy = tam::helpers::cosy::CurvilinearCosy::create(x, y, z)->build();

  // Convert sn to cartesian
  double d = 5.0;
  Eigen::VectorXd s = Eigen::VectorXd::LinSpaced(100, -0.19, cosy->get_ref_line_length() + 0.19);

  Eigen::VectorXd x_traf = Eigen::VectorXd::Zero(s.size());
  Eigen::VectorXd y_traf = Eigen::VectorXd::Zero(s.size());
  for (int i = 0; i < s.size(); ++i) {
    Eigen::Vector2d pos1 = cosy->convert_to_cartesian(s[i], d);
    x_traf[i] = pos1[0];
    y_traf[i] = pos1[1];
  }

  // Plot
  plt::figure();
  plt::plot(x, y, "x-", {{"label", "Ref-Line"}});
  plt::plot(x_traf, y_traf, "rx-", {{"label", "d=5"}});
  plt::xlabel("x");
  plt::ylabel("y");
  plt::legend();
  plt::axis("equal");

  plt::show();
  return 0;
}
