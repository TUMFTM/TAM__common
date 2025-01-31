// Copyright 2024 Maximilian Leitenstern
#pragma once

#include <math.h>

#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"
#include "tum_types_cpp/tire_model.hpp"
namespace tam::helpers::tire_models
{
/******************************************************
 * @brief Base class for tire models
 ******************************************************/
template <typename T>
class TireModel
{
public:
  /**
   * @brief Definition for longitudinal force of tire
   * @param slip_ratio Slip ratio of the tire
   * @param slip_angle Slip angle of the tire
   * @param F_z Vertical force on the tire
   * @param params Parameters of the tire
   * @return double Longitudinal force
   */
  virtual double lon(
    const double slip_ratio, const double slip_angle, const double F_z,
    const T & params) const = 0;
  /**
   * @brief Definition for lateral force of tire
   * @param slip_ratio Slip ratio of the tire
   * @param slip_angle Slip angle of the tire
   * @param F_z Vertical force on the tire
   * @param params Parameters of the tire
   * @return double Lateral force
   */
  virtual double lat(
    const double slip_ratio, const double slip_angle, const double F_z,
    const T & params) const = 0;
  /**
   * @brief Definition for self aligning torque of tire
   * @param slip_ratio Slip ratio of the tire
   * @param slip_angle Slip angle of the tire
   * @param F_z Vertical force on the tire
   * @param params Parameters of the tire
   * @return double Self aligning torque
   */
  virtual double self_aligning(
    const double slip_ratio, const double slip_angle, const double F_z,
    const T & params) const = 0;

public:
  // Destructor
  TireModel() = default;
  virtual ~TireModel() = default;
  TireModel(const TireModel & other) = default;
  TireModel(TireModel && other) = default;
  TireModel & operator=(const TireModel & other) = default;
  TireModel & operator=(TireModel && other) = default;
  /**
   * @brief Functions overloads for DataPerWheel for forces/moment
   */
  tam::types::common::DataPerWheel<double> lon(
    const tam::types::common::DataPerWheel<double> & slip_ratios,
    const tam::types::common::DataPerWheel<double> & slip_angles,
    const tam::types::common::DataPerWheel<double> & F_z,
    const tam::types::common::DataPerWheel<T> & params) const;
  tam::types::common::DataPerWheel<double> lat(
    const tam::types::common::DataPerWheel<double> & slip_ratios,
    const tam::types::common::DataPerWheel<double> & slip_angles,
    const tam::types::common::DataPerWheel<double> & F_z,
    const tam::types::common::DataPerWheel<T> & params) const;
  tam::types::common::DataPerWheel<double> self_aligning(
    const tam::types::common::DataPerWheel<double> & slip_ratios,
    const tam::types::common::DataPerWheel<double> & slip_angles,
    const tam::types::common::DataPerWheel<double> & F_z,
    const tam::types::common::DataPerWheel<T> & params) const;
  /**
   * @brief Model Output
   */
  tam::types::common::DataPerWheel<tam::types::tire_models::TireModelOutput> eval(
    const tam::types::common::DataPerWheel<double> & slip_ratios,
    const tam::types::common::DataPerWheel<double> & slip_angles,
    const tam::types::common::DataPerWheel<double> & F_z,
    const tam::types::common::DataPerWheel<T> & params) const;
};
/******************************************************
 * MF simple model
 ******************************************************/
class MF_simple : public TireModel<tam::types::tire_models::MF_simple>
{
protected:
  double lon(
    const double slip_ratio, const double slip_angle, const double F_z,
    const tam::types::tire_models::MF_simple & params) const override;
  double lat(
    const double slip_ratio, const double slip_angle, const double F_z,
    const tam::types::tire_models::MF_simple & params) const override;
  double self_aligning(
    const double slip_ratio, const double slip_angle, const double F_z,
    const tam::types::tire_models::MF_simple & params) const override;
};
/******************************************************
 * MF 52 model
 ******************************************************/
class MF_52 : public TireModel<tam::types::tire_models::MF_52>
{
protected:
  double lon(
    const double slip_ratio, const double slip_angle, const double F_z,
    const tam::types::tire_models::MF_52 & params) const override;
  double lat(
    const double slip_ratio, const double slip_angle, const double F_z,
    const tam::types::tire_models::MF_52 & params) const override;
  double self_aligning(
    const double slip_ratio, const double slip_angle, const double F_z,
    const tam::types::tire_models::MF_52 & params) const override;
};
}  // namespace tam::helpers::tire_models
