//  Copyright 2023 Simon Hoffmann
#pragma once
#include <cmath>

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "tum_helpers_cpp/numerical.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::limits
{
/**
 * @brief Class saving a Margin in mps2 for each corner point in the limit shape
 *
 */
struct Margin
{
  explicit Margin(float margin) : ax_max(margin), ax_min(margin), ay_max(margin), ay_min(margin) {}
  Margin() = default;
  explicit Margin(float ax_max_in, float ax_min_in, float ay_max_in, float ay_min_in)
  : ax_max(ax_max_in), ax_min(ax_min_in), ay_max(ay_max_in), ay_min(ay_min_in)
  {
  }
  float ax_max{0.0};
  float ax_min{0.0};
  float ay_max{0.0};
  float ay_min{0.0};
};
/**
 * @brief Class saving a Scaling in %/100 for each corner point in the limit shape
 *
 */
struct Scaling
{
  explicit Scaling(float scaling)
  : ax_max(scaling), ax_min(scaling), ay_max(scaling), ay_min(scaling)
  {
  }
  Scaling() = default;
  explicit Scaling(float ax_max_in, float ax_min_in, float ay_max_in, float ay_min_in)
  : ax_max(ax_max_in), ax_min(ax_min_in), ay_max(ay_max_in), ay_min(ay_min_in)
  {
  }
  float ax_max{1.0};
  float ax_min{1.0};
  float ay_max{1.0};
  float ay_min{1.0};
};
/**
 * @brief Get a y value for ref_x on the connection line [ax_max, ax_min]
 *
 * @param pt Constraint Shape
 * @param ref_x x position to evaluate the line equation at
 * @return y position
 */
float evaluate_straight_ns(
  const tam::types::control::ControlConstraintPoint & pt, const float ref_x);
/**
 * @brief Get a x value for ref_y on the connection line [ay_max, ay_min]
 *
 * @param pt Constraint Shape
 * @param ref_x y position to evaluate the line equation at
 * @return x position
 */
float evaluate_straight_we(
  const tam::types::control::ControlConstraintPoint & pt, const float ref_y);
/**
 * @brief Get the center of the limit shape. Meaning the intersection of [ay_max, ay_min] and
 * [ax_max, ax_min]. Algorithm according to:
 * https://walser-h-m.ch/hans/Miniaturen/S/Schwerpunkte_Viereck/Schwerpunkte_Viereck.htm
 *
 * @param pt Constraint Shape
 * @return Center of the limit Shape
 */
tam::types::common::Vector2D<double> center_of_limit_shape(
  const tam::types::control::ControlConstraintPoint & pt);
/**
 * @brief If the provided acceleration lies outside of the limits, an alternative acceleration is
 * returned which lies on the intersection of the limit shape and the connection [acc, limit_center]
 *
 * @param pt Constraint Shape
 * @param acc input acceleration
 * @return limited output acceleration
 */
tam::types::common::Vector2D<double> cut_off_at_limits(
  const tam::types::control::ControlConstraintPoint & pt,
  const tam::types::common::Vector2D<double> & acc);
/**
 * @brief Scales the Constraints with the provided scaling input. Scaling of 1.0 results in the same
 * output
 *
 * @param point Constraint Shape
 * @param scaling Scaling Class allows to provide a individual scaling for each corner
 * @return Scaled Constraint Shape
 */
tam::types::control::ControlConstraintPoint scale_constraint_point(
  const tam::types::control::ControlConstraintPoint & point, const Scaling & scaling);
/**
 * @brief Adds a margin to the provided constraint input. Margin of 0.0 results in the same
 * output
 *
 * @param point Constraint Shape
 * @param margins Margin Class allows to provide a individual margin for each corner
 * @return Constraint Shape with margin
 */
tam::types::control::ControlConstraintPoint constraint_point_with_margin(
  const tam::types::control::ControlConstraintPoint & point, const Margin & margins);
/**
 * @brief Adds a margin to the provided constraints input. Margin of 0.0 results in the same
 * output
 *
 * @param point Constraint Shapes
 * @param margins Margin Class allows to provide a individual margin for each corner
 * @return Constraint Shapes with margin
 */
tam::types::control::ControlConstraints constraints_with_margin(
  tam::types::control::ControlConstraints const & constraints, const Margin & margins);
/**
 * @brief Evaluates the limit shape at a specific y coordinate
 *
 * @param pt1 First point that lies on the shape
 * @param pt2 Second point that lies on the shape
 * @param p exponent
 * @param y y-Acceleration to evaluate the limit shape at
 * @return x-Acceleration for the provided y-Acceleration input
 */
float p_limit(
  const tam::types::common::Vector2D<double> & pt1,
  const tam::types::common::Vector2D<double> & pt2, const float p, const float y);
/**
 * @brief Checks if a provided acceleration input lies within the limits
 *
 * @param pt Limit Shape
 * @param acc Input Acceleration
 * @return true: Input Acceleration (acc) lies within the limits
 * @return false: Input Acceleration (acc) not within limits
 */
bool pt_within_limits(
  const tam::types::control::ControlConstraintPoint & pt,
  const tam::types::common::Vector2D<double> & acc);
/**
 * @brief Returns the maximum ax acceleration based on the provided ay_value
 *
 * @param pt Limit Shape
 * @param acc Input Acceleration
 * @return true: ax_max
 */
double get_max_ax_from_ay(
  const tam::types::control::ControlConstraintPoint & pt,
  const tam::types::common::Vector2D<double> & acc);
/**
 * @brief Returns a Polygon of size n_target, that approximates the provided limit shape (pt)
 *
 * @param pt Original Limit Shape
 * @param n_target Number of points used to approximate the original limit shape
 * @return Polygon of size n_target that approximates the original input shape (pt)
 */
tam::types::control::ControlConstraintPointAsPolygon interp_control_constraint_point(
  const tam::types::control::ControlConstraintPoint & pt, const int n_target);
/**
 * @brief Approximates all constraints in input (constraints) with a polygon of size n_target
 *
 * @param constraints Original Limit Shapes
 * @param n_points Number of points used to approximate the original limit shape
 * @return ControlConstraintsPolygon containin approximated limit shapes
 */
tam::types::control::ControlConstraintsPolygon interp_control_constraints(
  const tam::types::control::ControlConstraints & constraints, const int n_points);
void update_dynamic_constraints(
  tam::types::control::ControlConstraintsPolygon * const control_constraints_ptr,
  const double v_mps, const double kappa_max_steering_1pm, const double P_VDC_MinVelSlipCalc_mps);
}  // namespace tam::limits
