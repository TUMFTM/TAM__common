// Copyright 2023 Simon Hoffmann
#pragma once
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "track_handler_cpp/track_helpers.hpp"
#include "track_handler_cpp/track_types.hpp"
#include "tum_helpers_cpp/numerical.hpp"
namespace tam::common
{
using tam::helpers::track::s_mod;
class Raceline
{
protected:
  explicit Raceline(RacelineData && data);

private:
  std::unique_ptr<RacelineData> data_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static std::unique_ptr<Raceline> create_from_csv(const std::string & path);
  // Data Access
  const Eigen::Ref<Eigen::VectorXd> s() const { return data_->data.at(RacelineData::s); }
  const Eigen::Ref<Eigen::VectorXd> v() const { return data_->data.at(RacelineData::v); }
  const Eigen::Ref<Eigen::VectorXd> n() const { return data_->data.at(RacelineData::n); }
  const Eigen::Ref<Eigen::VectorXd> chi() const { return data_->data.at(RacelineData::chi); }
  const Eigen::Ref<Eigen::VectorXd> ax() const { return data_->data.at(RacelineData::ax); }
  const Eigen::Ref<Eigen::VectorXd> ay() const { return data_->data.at(RacelineData::ay); }
  const Eigen::Ref<Eigen::VectorXd> jx() const { return data_->data.at(RacelineData::jx); }
  const Eigen::Ref<Eigen::VectorXd> jy() const { return data_->data.at(RacelineData::jy); }
  /**
   * @brief get velocity interpolated with s
   *
   * @param s
   * @return v
   */
  Eigen::MatrixXd v(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::v));
  }
  std::vector<double> v(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::v));
  }
  double v(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::v));
  }
  /**
   * @brief get n interpolated with s
   *
   * @param s
   * @return n
   */
  Eigen::MatrixXd n(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::n));
  }
  std::vector<double> n(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::n));
  }
  double n(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::n));
  }
  /**
   * @brief
   *
   * @param s
   * @return chi
   */
  Eigen::MatrixXd chi(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::chi));
  }
  std::vector<double> chi(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::chi));
  }
  double chi(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::chi));
  }
  /**
   * @brief get ax interpolated with s
   *
   * @param s
   * @return ax
   */
  Eigen::MatrixXd ax(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::ax));
  }
  std::vector<double> ax(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::ax));
  }
  double ax(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::ax));
  }
  /**
   * @brief get ay interpolated with s
   *
   * @param s
   * @return ay
   */
  Eigen::MatrixXd ay(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::ay));
  }
  std::vector<double> ay(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::ay));
  }
  double ay(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::ay));
  }
  /**
   * @brief get jx interpolated with s
   *
   * @param s
   * @return jx
   */
  Eigen::MatrixXd jx(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::jx));
  }
  std::vector<double> jx(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::jx));
  }
  double jx(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::jx));
  }
  /**
   * @brief get jy interpolated with s
   *
   * @param s
   * @return jy
   */
  Eigen::MatrixXd jy(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::jy));
  }
  std::vector<double> jy(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::jy));
  }
  double jy(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(RacelineData::s)(Eigen::last)), data_->data.at(RacelineData::s),
      data_->data.at(RacelineData::jy));
  }
};
}  // namespace tam::common
// Todo (Simon): function to calc cartesianRaceline with track;
