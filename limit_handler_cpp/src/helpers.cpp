//  Copyright 2023 Simon Hoffmann
#include "limit_handler_cpp/helpers.hpp"
namespace tam::limits
{
float evaluate_straight_ns(
  const tam::types::control::ControlConstraintPoint & pt, const float ref_x)
{
  tam::types::common::Vector2D<double> start{pt.a_x_min_mps2};
  tam::types::common::Vector2D<double> end{pt.a_x_max_mps2};
  return ((ref_x - end.x) / (start.x - end.x)) * (start.y - end.y) + end.y;
}
float evaluate_straight_we(
  const tam::types::control::ControlConstraintPoint & pt, const float ref_y)
{
  tam::types::common::Vector2D<double> start{pt.a_y_max_mps2};
  tam::types::common::Vector2D<double> end{pt.a_y_min_mps2};
  return ((ref_y - end.y) / (start.y - end.y)) * (start.x - end.x) + end.x;
}
tam::types::common::Vector2D<double> center_of_limit_shape(
  const tam::types::control::ControlConstraintPoint & pt)
{
  // https://walser-h-m.ch/hans/Miniaturen/S/Schwerpunkte_Viereck/Schwerpunkte_Viereck.htm
  tam::types::common::Vector2D<double> out;
  float yA = pt.a_x_max_mps2.y;
  float xA = pt.a_x_max_mps2.x;
  float yB = pt.a_y_max_mps2.y;
  float xB = pt.a_y_max_mps2.x;
  float yC = pt.a_x_min_mps2.y;
  float xC = pt.a_x_min_mps2.x;
  float yD = pt.a_y_min_mps2.y;
  float xD = pt.a_y_min_mps2.x;
  out.x = (((yC - yD) * xB + xD * (yB - yC)) * xA - ((yA - yD) * xB - xD * (yA - yB)) * xC) /
          ((yB - yD) * xA + (-yA + yC) * xB + (-yB + yD) * xC + xD * (yA - yC));
  out.y = (((-xC + xD) * yB - yD * (xB - xC)) * yA + ((xA - xD) * yB - yD * (xA - xB)) * yC) /
          ((-xB + xD) * yA + (xA - xC) * yB + (xB - xD) * yC - yD * (xA - xC));
  return out;
}
tam::types::common::Vector2D<double> cut_off_at_limits(
  const tam::types::control::ControlConstraintPoint & pt,
  const tam::types::common::Vector2D<double> & acc)
{
  tam::types::common::Vector2D<double> middle_point{};
  float norm_y, norm_x, exp;
  float g_ns = evaluate_straight_ns(pt, acc.x);  // North-South
  float g_we = evaluate_straight_we(pt, acc.y);  // West-East
  // 1. Quadrant NorthEast
  if (acc.y >= g_ns && acc.x >= g_we) {
    middle_point = {pt.a_y_max_mps2.x, pt.a_x_max_mps2.y};
    norm_y = std::abs((acc.y - pt.a_x_max_mps2.y) / (pt.a_y_max_mps2.y - pt.a_x_max_mps2.y));
    norm_x = std::abs((acc.x - pt.a_y_max_mps2.x) / (pt.a_x_max_mps2.x - pt.a_y_max_mps2.x));
    exp = pt.shape_factor.at(0);
  }
  // 2. Quadrant SouthEast
  if (acc.y >= g_ns && acc.x <= g_we) {
    middle_point = {pt.a_y_max_mps2.x, pt.a_x_min_mps2.y};
    norm_y = std::abs((acc.y - pt.a_x_min_mps2.y) / (pt.a_y_max_mps2.y - pt.a_x_min_mps2.y));
    norm_x = std::abs((acc.x - pt.a_y_max_mps2.x) / (pt.a_x_min_mps2.x - pt.a_y_max_mps2.x));
    exp = pt.shape_factor.at(1);
  }
  // 3. Quadrant SouthWest
  if (acc.y <= g_ns && acc.x <= g_we) {
    middle_point = {pt.a_y_min_mps2.x, pt.a_x_min_mps2.y};
    norm_y = std::abs((acc.y - pt.a_x_min_mps2.y) / (pt.a_y_min_mps2.y - pt.a_x_min_mps2.y));
    norm_x = std::abs((acc.x - pt.a_y_min_mps2.x) / (pt.a_x_min_mps2.x - pt.a_y_min_mps2.x));
    exp = pt.shape_factor.at(2);
  }
  // 4. Quadrant NorthWest
  if (acc.y <= g_ns && acc.x >= g_we) {
    middle_point = {pt.a_y_min_mps2.x, pt.a_x_max_mps2.y};
    norm_y = std::abs((acc.y - pt.a_x_max_mps2.y) / (pt.a_y_min_mps2.y - pt.a_x_max_mps2.y));
    norm_x = std::abs((acc.x - pt.a_y_min_mps2.x) / (pt.a_x_max_mps2.x - pt.a_y_min_mps2.x));
    exp = pt.shape_factor.at(3);
  }
  double limit = std::pow(std::pow(norm_x, exp) + std::pow(norm_y, exp), 1.0 / exp);
  if (limit > 1.0) {
    return (middle_point + (acc - middle_point) * (1.0 / limit));
  } else {
    return acc;
  }
}
tam::types::control::ControlConstraintPoint scale_constraint_point(
  const tam::types::control::ControlConstraintPoint & point, const Scaling & scaling)
{
  tam::types::common::Vector2D<double> middle_point = center_of_limit_shape(point);
  tam::types::control::ControlConstraintPoint scaled_point = point;
  // ax_min
  tam::types::common::Vector2D<double> direction = point.a_x_min_mps2 - middle_point;
  scaled_point.a_x_min_mps2 = middle_point + direction * scaling.ax_min;
  // ax_max
  direction = point.a_x_max_mps2 - middle_point;
  scaled_point.a_x_max_mps2 = middle_point + direction * scaling.ax_max;
  // ay_max
  direction = point.a_y_max_mps2 - middle_point;
  scaled_point.a_y_max_mps2 = middle_point + direction * scaling.ay_max;
  // ay_min
  direction = point.a_y_min_mps2 - middle_point;
  scaled_point.a_y_min_mps2 = middle_point + direction * scaling.ay_min;
  return scaled_point;
}
tam::types::control::ControlConstraintPoint constraint_point_with_margin(
  const tam::types::control::ControlConstraintPoint & point, const Margin & margins)
{
  tam::types::common::Vector2D<double> middle_point = center_of_limit_shape(point);
  tam::types::control::ControlConstraintPoint scaled_point = point;
  // ax_min
  tam::types::common::Vector2D<double> direction = point.a_x_min_mps2 - middle_point;
  double norm = sqrt(pow(direction.x, 2) + pow(direction.y, 2));
  scaled_point.a_x_min_mps2 =
    norm > 1e-13 ? point.a_x_min_mps2 + direction * (margins.ax_min / norm) : point.a_x_min_mps2;
  // ax_max
  direction = point.a_x_max_mps2 - middle_point;
  norm = sqrt(pow(direction.x, 2) + pow(direction.y, 2));
  scaled_point.a_x_max_mps2 =
    norm > 1e-13 ? point.a_x_max_mps2 + direction * (margins.ax_max / norm) : point.a_x_max_mps2;
  // ay_max
  direction = point.a_y_max_mps2 - middle_point;
  norm = sqrt(pow(direction.x, 2) + pow(direction.y, 2));
  scaled_point.a_y_max_mps2 =
    norm > 1e-13 ? point.a_y_max_mps2 + direction * (margins.ay_max / norm) : point.a_y_max_mps2;
  // ay_min
  direction = point.a_y_min_mps2 - middle_point;
  norm = sqrt(pow(direction.x, 2) + pow(direction.y, 2));
  scaled_point.a_y_min_mps2 =
    norm > 1e-13 ? point.a_y_min_mps2 + direction * (margins.ay_min / norm) : point.a_y_min_mps2;
  return scaled_point;
}
tam::types::control::ControlConstraints constraints_with_margin(
  tam::types::control::ControlConstraints const & constraints, const Margin & margins)
{
  tam::types::control::ControlConstraints scaled_constraints{};
  for (auto const & point : constraints.points) {
    scaled_constraints.points.push_back(constraint_point_with_margin(point, margins));
  }
  scaled_constraints.header = constraints.header;
  return scaled_constraints;
}
double p_limit(
  const tam::types::common::Vector2D<double> & pt1,
  const tam::types::common::Vector2D<double> & pt2, const float p, const double y)
{
  return (pt1.x - pt2.x) *
           pow(std::max((1 - pow(std::max((y - pt1.y) / (pt2.y - pt1.y), 0.0), p)), 0.0), 1 / p) +
         pt2.x;
}
bool pt_within_limits(
  const tam::types::control::ControlConstraintPoint & pt,
  const tam::types::common::Vector2D<double> & acc)
{
  float g_ns = evaluate_straight_ns(pt, acc.x);  // North-South
  float g_we = evaluate_straight_we(pt, acc.y);  // West- East
  // 1. Quadrant NorthEast
  if (acc.y >= g_ns && acc.x >= g_we) {
    float x = p_limit(pt.a_x_max_mps2, pt.a_y_max_mps2, pt.shape_factor.at(0), acc.y);
    return acc.x <= x;
  }
  // 2. Quadrant SouthEast
  if (acc.y >= g_ns && acc.x <= g_we) {
    float x = p_limit(pt.a_x_min_mps2, pt.a_y_max_mps2, pt.shape_factor.at(1), acc.y);
    return acc.x >= x;
  }
  // 3. Quadrant SouthWest
  if (acc.y <= g_ns && acc.x <= g_we) {
    float x = p_limit(pt.a_x_min_mps2, pt.a_y_min_mps2, pt.shape_factor.at(2), acc.y);
    return acc.x >= x;
  }
  // 4. Quadrant NorthWest
  if (acc.y <= g_ns && acc.x >= g_we) {
    float x = p_limit(pt.a_x_max_mps2, pt.a_y_min_mps2, pt.shape_factor.at(3), acc.y);
    return acc.x <= x;
  }
  return false;
}
double get_max_ax_from_ay(
  const tam::types::control::ControlConstraintPoint & pt,
  const tam::types::common::Vector2D<double> & acc)
{
  float g_ns = evaluate_straight_ns(pt, acc.x);  // North-South
  float g_we = evaluate_straight_we(pt, acc.y);  // West- East
  // 1. Quadrant NorthEast
  if (acc.y >= g_ns && acc.x >= g_we) {
    return p_limit(pt.a_x_max_mps2, pt.a_y_max_mps2, pt.shape_factor.at(0), acc.y);
  }
  // 2. Quadrant SouthEast
  if (acc.y >= g_ns && acc.x <= g_we) {
    return p_limit(pt.a_x_min_mps2, pt.a_y_max_mps2, pt.shape_factor.at(1), acc.y);
  }
  // 3. Quadrant SouthWest
  if (acc.y <= g_ns && acc.x <= g_we) {
    return p_limit(pt.a_x_min_mps2, pt.a_y_min_mps2, pt.shape_factor.at(2), acc.y);
  }
  // 4. Quadrant NorthWest
  if (acc.y <= g_ns && acc.x >= g_we) {
    return p_limit(pt.a_x_max_mps2, pt.a_y_min_mps2, pt.shape_factor.at(3), acc.y);
  }
  return -1;
}
tam::types::control::ControlConstraintPointAsPolygon interp_control_constraint_point(
  const tam::types::control::ControlConstraintPoint & pt, const int n_target)
{
  tam::types::control::ControlConstraintPointAsPolygon out;

  std::vector<double> y_data;
  y_data.push_back(pt.a_x_max_mps2.y);
  y_data.push_back(pt.a_y_max_mps2.y);
  y_data.push_back(pt.a_x_min_mps2.y);
  y_data.push_back(pt.a_y_min_mps2.y);
  y_data.push_back(y_data.at(0));

  std::vector<double> n_pts(y_data.size());
  for (std::size_t i = 0; i < y_data.size(); i++) {
    n_pts.at(i) = i / static_cast<double>(y_data.size() - 1);
  }

  std::vector<double> n_target_pts(n_target);
  for (int i = 0; i < n_target; i++) {
    n_target_pts.at(i) = i / static_cast<double>(n_target);
  }

  std::vector<double> y_target_pts = tam::helpers::numerical::interp(n_target_pts, n_pts, y_data);

  for (int i = 0; i < n_target / 4; i++) {
    tam::types::common::Vector2D<double> pt_interp;
    pt_interp.y = y_target_pts.at(i);
    pt_interp.x = p_limit(pt.a_x_max_mps2, pt.a_y_max_mps2, pt.shape_factor.at(0), pt_interp.y);
    out.a_lim.push_back(pt_interp);
  }

  for (int i = n_target / 4; i < n_target / 2; i++) {
    tam::types::common::Vector2D<double> pt_interp;
    pt_interp.y = y_target_pts.at(i);
    pt_interp.x = p_limit(pt.a_x_min_mps2, pt.a_y_max_mps2, pt.shape_factor.at(1), pt_interp.y);
    out.a_lim.push_back(pt_interp);
  }

  for (int i = n_target / 2; i < 3 * n_target / 4; i++) {
    tam::types::common::Vector2D<double> pt_interp;
    pt_interp.y = y_target_pts.at(i);
    pt_interp.x = p_limit(pt.a_x_min_mps2, pt.a_y_min_mps2, pt.shape_factor.at(2), pt_interp.y);
    out.a_lim.push_back(pt_interp);
  }

  for (int i = 3 * n_target / 4; i < n_target; i++) {
    tam::types::common::Vector2D<double> pt_interp;
    pt_interp.y = y_target_pts.at(i);
    pt_interp.x = p_limit(pt.a_x_max_mps2, pt.a_y_min_mps2, pt.shape_factor.at(3), pt_interp.y);
    out.a_lim.push_back(pt_interp);
  }
  out.lateral_error_min_m = pt.lateral_error_min_m;
  out.lateral_error_max_m = pt.lateral_error_max_m;

  return out;
}
tam::types::control::ControlConstraintsPolygon interp_control_constraints(
  const tam::types::control::ControlConstraints & constraints, const int n_points)
{
  // number of desired points
  tam::types::control::ControlConstraintsPolygon out;
  for (const auto & pt : constraints.points) {
    out.points.push_back(interp_control_constraint_point(pt, n_points));
  }
  out.header = constraints.header;
  return out;
}
void update_dynamic_constraints(
  tam::types::control::ControlConstraintsPolygon * const control_constraints_ptr,
  const double v_mps, const double kappa_max_steering_1pm, const double P_VDC_MinVelSlipCalc_mps)
{
  // max lat acc imposed by max steering angle (kinematic model); a_y = kappa*v^2
  double a_y_lim_delta =
    kappa_max_steering_1pm * std::pow(std::max(v_mps, P_VDC_MinVelSlipCalc_mps), 2);

  for (auto contr_constr_pt_vec : control_constraints_ptr->points) {
    for (auto constr_pt : contr_constr_pt_vec.a_lim) {
      // clip lateral accs based on steering angle
      constr_pt.y = tam::helpers::numerical::clip_absolute(constr_pt.y, a_y_lim_delta);
      // low speed: scale down positive long. acc limit points
      if (v_mps < P_VDC_MinVelSlipCalc_mps && constr_pt.x > 0) {
        constr_pt.x /= 3;
      }
    }
  }
}
}  // namespace tam::limits
