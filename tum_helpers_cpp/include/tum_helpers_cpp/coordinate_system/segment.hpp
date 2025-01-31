// Copyright 2023 Simon Hoffmann
//
// Algorithm according to: https://ieeexplore.ieee.org/abstract/document/6856487
// Implementation according to:
// https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker/-/tree/master/cpp/geometry
//
#pragma once
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <utility>
#include <vector>

#include "tum_helpers_cpp/utility.hpp"
namespace tam::helpers::cosy
{
class CurvilinearCosy;
class Segment
{
  friend class CurvilinearCosy;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Segment(
    const Eigen::Vector2d & pt_1, const Eigen::Vector2d & pt_2, const Eigen::Vector2d & t_1,
    const Eigen::Vector2d & t_2);
  Segment() = default;

  Eigen::Vector2d pt_1() const;
  Eigen::Vector2d pt_2() const;
  double length() const;
  Eigen::Vector2d normalSegmentStart() const;
  Eigen::Vector2d normalSegmentEnd() const;
  Eigen::Vector2d normal(double s_local) const;
  Eigen::Vector2d tangent(double s_local) const;
  Eigen::Vector2d t_1() const { return t_1_; }
  Eigen::Vector2d t_2() const { return t_2_; }

protected:
  Eigen::Vector2d convertToCartesianCoords(double lambda, double d) const;
  Eigen::Vector2d convertToCurvilinearCoords(double x, double y) const;

private:
  Eigen::Vector2d convertToCurvilinearCoords(double x, double y, double & lambda) const;
  double computeLambda(const Eigen::Vector2d & p_local) const;
  double computeLambda(double s) const;
  inline Eigen::Vector2d computeBasePoint(double lambda) const;
  inline Eigen::Vector2d computePseudoNormal(
    const Eigen::Vector2d & p_lambda, const Eigen::Vector2d & p) const;

  inline Eigen::Vector2d computePseudoTangent(double lambda) const;
  Eigen::Vector2d computePseudoTangentGlobal(double lambda) const;
  double computeSignedPseudoDistance(
    const Eigen::Vector2d & pseudo_normal, const Eigen::Vector2d & p_local) const;

  inline Eigen::Vector2d rotateToLocalFrame(const Eigen::Vector2d & p) const;
  inline Eigen::Vector2d rotateToGlobalFrame(const Eigen::Vector2d & p_local) const;

private:
  /// start point of segment
  Eigen::Vector2d pt_1_;
  /// end point of segment
  Eigen::Vector2d pt_2_;
  /// tangent of segment at start point
  Eigen::Vector2d t_1_;
  /// tangent of segment at end point
  Eigen::Vector2d t_2_;
  /// normal of segment at start point
  Eigen::Vector2d n_1_;
  /// normal of segment at end point
  Eigen::Vector2d n_2_;

  /// start point in local segment coordinates
  Eigen::Vector2d pt_1_local_;
  /// end point in local segment coordinates
  Eigen::Vector2d pt_2_local_;
  /// local tangent at pt_1_local_
  Eigen::Vector2d t_1_local_;
  /// local tangent at pt_2_local_
  Eigen::Vector2d t_2_local_;

  /// tangent vector of the line connecting pt_1 and pt_2
  Eigen::Vector2d tangent_;
  /// normal vector to the tangent of the line connecting pt_1 and pt_2
  Eigen::Vector2d normal_;

  /// slopes of tangent vectors: t_1_ = (1, m_1_); t_2_ = (1, m_2_)
  double m_1_;
  double m_2_;

  Eigen::Matrix2d rotate_to_local_;
  Eigen::Matrix2d rotate_to_global_;

  /// length of the segment
  double length_;
};
}  // namespace tam::helpers::cosy
