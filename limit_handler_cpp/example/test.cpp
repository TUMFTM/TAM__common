#include <map>

#include "limit_handler_cpp/helpers.hpp"
#include "matplotlib_cpp/matplotlibcpp.hpp"
#include "tum_types_cpp/control.hpp"

namespace plt = matplotlibcpp;
// plot helper function
void plot_shape(
  const tam::types::control::ControlConstraintPointAsPolygon & interp, std::string format)
{
  std::vector<double> x_int;
  std::vector<double> y_int;
  for (const auto & pt : interp.a_lim) {
    x_int.push_back(pt.x);
    y_int.push_back(pt.y);
  }
  plt::plot(y_int, x_int, format);
}
int main()
{
  // Example Limit point
  tam::types::control::ControlConstraintPoint pt;
  pt.a_x_max_mps2.x = 10;
  pt.a_x_max_mps2.y = 2;
  pt.a_y_max_mps2.x = 1;
  pt.a_y_max_mps2.y = 11;
  pt.a_x_min_mps2.x = -9;
  pt.a_x_min_mps2.y = 1;
  pt.a_y_min_mps2.x = 0;
  pt.a_y_min_mps2.y = -9;
  pt.shape_factor = {1.3, 1.3, 1.3, 1.3};

  // Get the center point of the limit shape
  tam::types::common::Vector2D center = tam::limits::center_of_limit_shape(pt);

  // Cut off a point that lies outside of the limits
  tam::types::common::Vector2D<double> point_outside{-10, -10};
  auto point_cutoff = tam::limits::cut_off_at_limits(pt, point_outside);

  // Create a octagon interpolation of the elliptic shape
  auto pt_interp_octagon = tam::limits::interp_control_constraint_point(pt, 8);

  // Scale limits (Each corner can be scaled individually)
  auto pt_scaled =
    tam::limits::scale_constraint_point(pt, tam::limits::Scaling(0.7, 0.7, 0.7, 0.7));
  // Add margins to limits (Each corner can have a individual margin)
  auto pt_margin =
    tam::limits::constraint_point_with_margin(pt, tam::limits::Margin(0.0, 3.0, 5.0, 3.0));

  // Plotting
  auto pt_interp_full = tam::limits::interp_control_constraint_point(pt, 10000);
  plot_shape(pt_interp_full, "k");
  plot_shape(pt_interp_octagon, "c--");
  auto pt_interp_scaled = tam::limits::interp_control_constraint_point(pt_scaled, 10000);
  plot_shape(pt_interp_scaled, "g:");
  auto pt_interp_margin = tam::limits::interp_control_constraint_point(pt_margin, 10000);
  plot_shape(pt_interp_margin, "m-.");

  // Corner points
  std::vector<double> x_pt;
  x_pt.push_back(pt.a_x_max_mps2.x);
  x_pt.push_back(pt.a_y_max_mps2.x);
  x_pt.push_back(pt.a_x_min_mps2.x);
  x_pt.push_back(pt.a_y_min_mps2.x);
  std::vector<double> y_pt;
  y_pt.push_back(pt.a_x_max_mps2.y);
  y_pt.push_back(pt.a_y_max_mps2.y);
  y_pt.push_back(pt.a_x_min_mps2.y);
  y_pt.push_back(pt.a_y_min_mps2.y);
  plt::plot(y_pt, x_pt, "ko");

  // Plot center point
  plt::plot(std::vector<double>{center.y}, std::vector<double>{center.x}, "bo");

  // Point outside limits with corresponding cutoff point
  plt::plot(
    std::vector<double>{center.y, point_outside.y}, std::vector<double>{center.x, point_outside.x},
    "b:");
  plt::plot(std::vector<double>{point_outside.y}, std::vector<double>{point_outside.x}, "bo");
  plt::plot(std::vector<double>{point_cutoff.y}, std::vector<double>{point_cutoff.x}, "ro");

  // Generate plot
  plt::grid();
  plt::axis("equal");
  plt::show();

  return 0;
}
