// Copyright 2023 Simon Hoffmann
//
// Algorithm according to: https://ieeexplore.ieee.org/abstract/document/6856487
// Implementation according to:
// https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker/-/tree/master/cpp/geometry
//
#pragma once
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <numeric>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

#include "iostream"
#include "tum_helpers_cpp/coordinate_system/segment.hpp"
#include "tum_helpers_cpp/geometry/geometry.hpp"
#include "tum_helpers_cpp/utility.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::helpers::cosy
{
class CurvilinearCosyBuilder;  // forward declaration
class CurvilinearCosy
{
  friend class CurvilinearCosyBuilder;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Different Options and combinations to construct Cosy. Use Builder Pattern
  template <typename... Params>
  static std::unique_ptr<CurvilinearCosyBuilder> create(Params &&... params)
  {
    return std::make_unique<CurvilinearCosyBuilder>(std::forward<Params>(params)...);
  }
  Eigen::Vector2d convert_to_sn(const double x, const double y) const;
  Eigen::Vector3d convert_to_sn(const double x, const double y, const double yaw) const;
  Eigen::Vector3d convert_to_sn(const tam::types::control::Odometry & odom) const;
  std::tuple<Eigen::Vector2d, float> convert_to_sn_and_get_idx(
    const double x, const double y) const;
  std::tuple<Eigen::Vector3d, float> convert_to_sn_and_get_idx(
    const tam::types::control::Odometry & odom) const;
  std::tuple<Eigen::Vector3d, float> convert_to_sn_and_get_idx(
    const double x, const double y, const double yaw) const;
  Eigen::Vector2d convert_to_cartesian(const double s, const double d) const;
  Eigen::Vector3d convert_to_cartesian(const double s, const double d, const double yaw) const;
  bool is_closed() { return closed_cosy_; }
  double get_ref_line_length() const { return length_; }
  // Not required for binding
  std::vector<double> get_segment_starting_s() const { return segment_s0_; }
  std::vector<double> get_segment_length() const { return segment_length_; }
  int get_number_of_segments() const { return segment_list_.size(); }
  Segment get_segment(const int idx) const { return segment_list_.at(idx); }
  //
private:
  CurvilinearCosy() = default;
  std::optional<int> find_segment_index(double s) const;
  void create_segments(
    const std::vector<double> & tx, const std::vector<double> & ty, const std::vector<double> & tz);
  void create_segment(
    const Eigen::Vector3d & pt_1, const Eigen::Vector3d & pt_2, const Eigen::Vector3d & t_1,
    const Eigen::Vector3d & t_2);
  int get_closest_point_id(const double x, const double y) const;

  std::vector<double> x_, y_, z_;
  std::vector<double> segment_s0_, segment_length_;
  std::vector<Segment> segment_list_;
  double length_{0};
  Eigen::Vector2d zero_heading_dir_{1, 0};
  int range_to_check_{1};
  bool closed_cosy_{false};
  Segment extrapolate_front;
  Segment extrapolate_back;
};
typedef std::unique_ptr<CurvilinearCosy> CurvilinearCosyPtr;
typedef std::shared_ptr<CurvilinearCosy> CurvilinearCosySharedPtr;
//
//
// COSY Builder
// Different Options and combinations to construct Cosy. Use Builder Pattern
//
class CurvilinearCosyBuilder
{
public:
  CurvilinearCosyBuilder(
    const Eigen::VectorXd & x, const Eigen::VectorXd & y, const Eigen::VectorXd & z)
  {
    impl_ = std::unique_ptr<CurvilinearCosy>(new CurvilinearCosy);  // make_unique breaks friend
    impl_->x_.resize(x.size());
    Eigen::Map<Eigen::VectorXd>(impl_->x_.data(), impl_->x_.size()) = x;
    impl_->y_.resize(y.size());
    Eigen::Map<Eigen::VectorXd>(impl_->y_.data(), impl_->y_.size()) = y;
    impl_->z_.resize(z.size());
    Eigen::Map<Eigen::VectorXd>(impl_->z_.data(), impl_->z_.size()) = z;
    check_point_input();
  }
  CurvilinearCosyBuilder(
    const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & z)
  {
    impl_ = std::unique_ptr<CurvilinearCosy>(new CurvilinearCosy);  // make_unique breaks friend
    impl_->x_ = x;
    impl_->y_ = y;
    impl_->z_ = z;
    check_point_input();
  }
  explicit CurvilinearCosyBuilder(const tam::types::control::Trajectory & traj)
  {
    impl_ = std::unique_ptr<CurvilinearCosy>(new CurvilinearCosy);
    impl_->x_.reserve(traj.points.size());
    impl_->y_.reserve(traj.points.size());
    impl_->z_.reserve(traj.points.size());
    for (const auto & pt : traj.points) {
      impl_->x_.push_back(pt.position_m.x);
      impl_->y_.push_back(pt.position_m.y);
      impl_->z_.push_back(pt.position_m.z);
      // TODO(Simon): Use orientation for tangent vector
    }
    check_point_input();
  }
  explicit CurvilinearCosyBuilder(const Eigen::Ref<const Eigen::MatrixX3d> ref_line)
  {
    impl_ = std::unique_ptr<CurvilinearCosy>(new CurvilinearCosy);  // make_unique breaks friend
    impl_->x_.resize(ref_line.rows());
    Eigen::Map<Eigen::VectorXd>(impl_->x_.data(), impl_->x_.size()) = ref_line.col(0);
    impl_->y_.resize(ref_line.rows());
    Eigen::Map<Eigen::VectorXd>(impl_->y_.data(), impl_->y_.size()) = ref_line.col(1);
    impl_->z_.resize(ref_line.rows());
    Eigen::Map<Eigen::VectorXd>(impl_->z_.data(), impl_->z_.size()) = ref_line.col(2);
    check_point_input();
  }
  CurvilinearCosyBuilder * set_s(Eigen::VectorXd & s)
  {
    s_tmp.resize(s.size());
    Eigen::Map<Eigen::VectorXd>(s_tmp.data(), s_tmp.size()) = s;
    check_s_input();

    return this;
  }
  CurvilinearCosyBuilder * set_s(
    const tam::types::control::AdditionalTrajectoryInfos & additionalInfo)
  {
    s_tmp.resize(additionalInfo.points.size());
    std::transform(
      additionalInfo.points.begin(), additionalInfo.points.end(), s_tmp.begin(),
      std::mem_fn(&tam::types::control::AdditionalInfoPoint::s_local_m));
    check_s_input();

    return this;
  }
  CurvilinearCosyBuilder * set_tangent(
    const Eigen::VectorXd & tx, const Eigen::VectorXd & ty, const Eigen::VectorXd & tz)
  {
    tx_tmp.resize(tx.size());
    Eigen::Map<Eigen::VectorXd>(tx_tmp.data(), tx_tmp.size()) = tx;
    ty_tmp.resize(ty.size());
    Eigen::Map<Eigen::VectorXd>(ty_tmp.data(), ty_tmp.size()) = ty;
    tz_tmp.resize(tz.size());
    Eigen::Map<Eigen::VectorXd>(tz_tmp.data(), tz_tmp.size()) = tz;
    check_tangent_input();
    return this;
  }
  CurvilinearCosyBuilder * set_tangent(const Eigen::Ref<const Eigen::MatrixX3d> tangent)
  {
    tx_tmp.resize(tangent.rows());
    Eigen::Map<Eigen::VectorXd>(tx_tmp.data(), tx_tmp.size()) = tangent.col(0);
    ty_tmp.resize(tangent.rows());
    Eigen::Map<Eigen::VectorXd>(ty_tmp.data(), ty_tmp.size()) = tangent.col(1);
    tz_tmp.resize(tangent.rows());
    Eigen::Map<Eigen::VectorXd>(tz_tmp.data(), tz_tmp.size()) = tangent.col(2);
    check_tangent_input();
    return this;
  }
  std::unique_ptr<CurvilinearCosy> build()
  {
    // s-coordinate (if s is empty)
    if (s_tmp.size() == 0) {
      s_tmp =
        tam::helpers::geometry::create_s_coordinate_from_points(impl_->x_, impl_->y_, impl_->z_);
    }

    // tangent vectors (if tangent is empty)
    if (tx_tmp.size() == 0) {
      create_tangent_from_points();
    }

    // Construct Cosy
    impl_->length_ = s_tmp.back();
    impl_->segment_length_ = tam::helpers::geometry::calc_segment_length(s_tmp);
    s_tmp.pop_back();                       // last point is not the start of a segment
    impl_->segment_s0_ = std::move(s_tmp);  // s_tmp not required after this call
    impl_->create_segments(tx_tmp, ty_tmp, tz_tmp);

    // if closed, delete last points, to not have duplicates for distance check
    if (impl_->closed_cosy_) {
      impl_->x_.pop_back();
      impl_->y_.pop_back();
      impl_->z_.pop_back();
    }
    // Use same tangent vector for first and second point to avoid interpolation of pseudo tangent
    // on extrapolation
    impl_->extrapolate_front = Segment(
      Eigen::Vector2d(impl_->x_.at(0), impl_->y_.at(0)),
      Eigen::Vector2d(impl_->x_.at(1), impl_->y_.at(1)),
      Eigen::Vector2d(tx_tmp.at(0), ty_tmp.at(0)), Eigen::Vector2d(tx_tmp.at(0), ty_tmp.at(0)));
    int n = impl_->x_.size() - 1;
    impl_->extrapolate_back = Segment(
      Eigen::Vector2d(impl_->x_.at(n - 1), impl_->y_.at(n - 1)),
      Eigen::Vector2d(impl_->x_.at(n), impl_->y_.at(n)),
      Eigen::Vector2d(tx_tmp.at(n), ty_tmp.at(n)), Eigen::Vector2d(tx_tmp.at(n), ty_tmp.at(n)));
    // Return Cosy
    return std::move(impl_);
  }

private:
  std::unique_ptr<CurvilinearCosy> impl_;
  std::vector<double> s_tmp, tx_tmp, ty_tmp, tz_tmp;
  void check_s_input()
  {
    // Expecting same data structure as x,y,z
    if (s_tmp.size() != impl_->x_.size()) {
      throw std::invalid_argument(
        "<CurvilinearCoordinateSystem> Provided s has different dimension from points");
    }
  }
  void check_tangent_input()
  {
    if (!(impl_->x_.size() == tx_tmp.size() && impl_->x_.size() == ty_tmp.size() &&
          impl_->x_.size() == tz_tmp.size())) {
      throw std::invalid_argument(
        "<CurvilinearCoordinateSystem> Provided tangent points have different dimension from "
        "points");
    }
  }
  void check_point_input()
  {
    if (!(impl_->x_.size() == impl_->y_.size() && impl_->y_.size() == impl_->z_.size())) {
      throw std::invalid_argument(
        "<CurvilinearCoordinateSystem> Provided points have different dimension");
    }
    if (
      (Eigen::Vector3d(impl_->x_.back(), impl_->y_.back(), impl_->z_.back()) -
       Eigen::Vector3d(impl_->x_.front(), impl_->y_.front(), impl_->z_.front()))
        .norm() < 1e-3) {
      std::cout << "[CurvilinearCosy]: First and last point are equal: Closed Coordinate System \n";
      impl_->closed_cosy_ = true;
    }
  }
  void create_tangent_from_points()
  {
    tx_tmp.reserve(impl_->x_.size());  // output should have same size as points
    ty_tmp.reserve(impl_->x_.size());
    tz_tmp.reserve(impl_->x_.size());
    // if closed last and first point are same, what leads to wrong tangent calc
    int considered_points = impl_->closed_cosy_ ? impl_->x_.size() - 1 : impl_->x_.size();
    tam::helpers::utility::IndexCounter idx(considered_points - 1, impl_->closed_cosy_);
    for (int i = 0; i < static_cast<int>(considered_points); ++i) {
      tx_tmp.push_back(impl_->x_.at(idx(+1)) - impl_->x_.at(idx(-1)));
      ty_tmp.push_back(impl_->y_.at(idx(+1)) - impl_->y_.at(idx(-1)));
      tz_tmp.push_back(impl_->z_.at(idx(+1)) - impl_->z_.at(idx(-1)));
      ++idx;
    }
    if (impl_->closed_cosy_) {  // last equals first if closed
      tx_tmp.push_back(tx_tmp.front());
      ty_tmp.push_back(ty_tmp.front());
      tz_tmp.push_back(tz_tmp.front());
    }
  }
};
}  // namespace tam::helpers::cosy
