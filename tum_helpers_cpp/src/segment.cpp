// Copyright 2023 TUM
//
// Algorithm according to: https://ieeexplore.ieee.org/abstract/document/6856487
// Implementation according to:
// https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker/-/tree/master/cpp/geometry
//
#include "tum_helpers_cpp/coordinate_system/segment.hpp"

#include <math.h>

#include <algorithm>
#include <iostream>
namespace tam::helpers::cosy
{
Segment::Segment(
  const Eigen::Vector2d & pt_1, const Eigen::Vector2d & pt_2, const Eigen::Vector2d & t_1,
  const Eigen::Vector2d & t_2)
{
  this->pt_1_ = pt_1;
  this->pt_2_ = pt_2;
  this->t_1_ = t_1.normalized();
  this->t_2_ = t_2.normalized();
  this->n_1_ = Eigen::Vector2d(-this->t_1_[1], this->t_1_[0]);
  this->n_2_ = Eigen::Vector2d(-this->t_2_[1], this->t_2_[0]);

  this->tangent_ = (this->pt_2_ - this->pt_1_).normalized();
  this->normal_ = Eigen::Vector2d(-this->tangent_[1], this->tangent_[0]);
  this->length_ = (this->pt_1_ - this->pt_2_).norm();

  rotate_to_local_ << this->tangent_[0], this->tangent_[1], this->normal_[0], this->normal_[1];
  rotate_to_global_ << this->tangent_[0], this->normal_[0], this->tangent_[1], this->normal_[1];

  this->pt_1_local_ = Eigen::Vector2d(0., 0.);
  this->pt_2_local_ = Eigen::Vector2d(0., this->length_);
  this->t_1_local_ = this->rotateToLocalFrame(t_1).normalized();
  this->m_1_ = this->t_1_local_[1] / this->t_1_local_[0];
  this->t_2_local_ = this->rotateToLocalFrame(t_2).normalized();
  this->m_2_ = this->t_2_local_[1] / this->t_2_local_[0];
}
Eigen::Vector2d Segment::pt_1() const { return this->pt_1_; }
Eigen::Vector2d Segment::pt_2() const { return this->pt_2_; }
double Segment::length() const { return this->length_; }
Eigen::Vector2d Segment::convertToCartesianCoords(double lambda, double d) const
{
  // double lambda = s / this->length
  // do not pass s directly lambda can be calculated outside for 3D line
  double s = lambda * this->length_;
  Eigen::Vector2d pseudo_tangent = this->computePseudoTangent(lambda);
  Eigen::Vector2d p_local =
    Eigen::Vector2d(s, 0) + d * Eigen::Vector2d(-pseudo_tangent[1], pseudo_tangent[0]);
  Eigen::Vector2d p = this->rotateToGlobalFrame(p_local);
  return p + this->pt_1_;
}
Eigen::Vector2d Segment::convertToCurvilinearCoords(double x, double y) const
{
  double lambda = 0.;
  return this->convertToCurvilinearCoords(x, y, lambda);
}
Eigen::Vector2d Segment::convertToCurvilinearCoords(double x, double y, double & lambda) const
{
  Eigen::Vector2d p(x, y);
  Eigen::Vector2d p_local = this->rotateToLocalFrame(p - this->pt_1_);
  lambda = this->computeLambda(p_local);
  Eigen::Vector2d p_lambda = this->computeBasePoint(lambda);
  Eigen::Vector2d pseudo_normal = this->computePseudoNormal(p_lambda, p);
  double pseudo_distance = this->computeSignedPseudoDistance(pseudo_normal, p_local);
  return Eigen::Vector2d(lambda * this->length_, pseudo_distance);
}
Eigen::Vector2d Segment::normalSegmentStart() const { return this->n_1_; }
Eigen::Vector2d Segment::normalSegmentEnd() const { return this->n_2_; }
Eigen::Vector2d Segment::normal(double s_local) const
{
  double lambda = this->computeLambda(s_local);
  Eigen::Vector2d pseudo_tangent = this->computePseudoTangentGlobal(lambda);
  Eigen::Vector2d pseudo_normal = Eigen::Vector2d(-pseudo_tangent[1], pseudo_tangent[0]);
  return pseudo_normal;
}
Eigen::Vector2d Segment::tangent(double s_local) const
{
  double lambda = this->computeLambda(s_local);
  Eigen::Vector2d pseudo_tangent = this->computePseudoTangentGlobal(lambda);
  return pseudo_tangent;
}
double Segment::computeLambda(double s) const { return s / this->length_; }
double Segment::computeLambda(const Eigen::Vector2d & p_local) const
{
  double lambda = -1;
  double devider = this->length_ - p_local[1] * (this->m_2_ - this->m_1_);
  if (std::isgreater(std::abs(devider), 0.)) {
    lambda = (p_local[0] + p_local[1] * m_1_) / devider;
  }
  return lambda;
}
Eigen::Vector2d Segment::computeBasePoint(double lambda) const
{
  return lambda * this->pt_2_ + (1. - lambda) * this->pt_1_;
}
Eigen::Vector2d Segment::computePseudoNormal(
  const Eigen::Vector2d & p_lambda, const Eigen::Vector2d & p) const
{
  return p - p_lambda;
}
Eigen::Vector2d Segment::computePseudoTangent(double lambda) const
{
  return (lambda * this->t_2_local_ + (1. - lambda) * this->t_1_local_).normalized();
}
Eigen::Vector2d Segment::computePseudoTangentGlobal(double lambda) const
{
  return (lambda * this->t_2_ + (1. - lambda) * this->t_1_).normalized();
}
double Segment::computeSignedPseudoDistance(
  const Eigen::Vector2d & pseudo_normal, const Eigen::Vector2d & p_local) const
{
  double pseudo_distance = pseudo_normal.norm();
  if (std::isless(p_local[1], 0.)) {
    pseudo_distance = -pseudo_distance;
  }
  return pseudo_distance;
}
Eigen::Vector2d Segment::rotateToLocalFrame(const Eigen::Vector2d & p) const
{
  return rotate_to_local_ * p;
}
Eigen::Vector2d Segment::rotateToGlobalFrame(const Eigen::Vector2d & p_local) const
{
  return rotate_to_global_ * p_local;
}
}  // namespace tam::helpers::cosy
