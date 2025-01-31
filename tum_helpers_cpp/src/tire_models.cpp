// Copyright 2024 Maximilian Leitenstern

#include "tum_helpers_cpp/tire_models.hpp"
namespace tam::helpers::tire_models
{
/******************************************************
 * @brief Base class for tire models
 ******************************************************/
/**
 * @brief Functions overloads of single-wheel functions for DataPerWheel
 */
template <typename T>
tam::types::common::DataPerWheel<double> TireModel<T>::lon(
  const tam::types::common::DataPerWheel<double> & slip_ratios,
  const tam::types::common::DataPerWheel<double> & slip_angles,
  const tam::types::common::DataPerWheel<double> & F_z,
  const tam::types::common::DataPerWheel<T> & params) const
{
  return tam::types::common::DataPerWheel<double>(
    lon(slip_ratios.front_left, slip_angles.front_left, F_z.front_left, params.front_left),
    lon(slip_ratios.front_right, slip_angles.front_right, F_z.front_right, params.front_right),
    lon(slip_ratios.rear_left, slip_angles.rear_left, F_z.rear_left, params.rear_left),
    lon(slip_ratios.rear_right, slip_angles.rear_right, F_z.rear_right, params.rear_right));
}
template <typename T>
tam::types::common::DataPerWheel<double> TireModel<T>::lat(
  const tam::types::common::DataPerWheel<double> & slip_ratios,
  const tam::types::common::DataPerWheel<double> & slip_angles,
  const tam::types::common::DataPerWheel<double> & F_z,
  const tam::types::common::DataPerWheel<T> & params) const
{
  return tam::types::common::DataPerWheel<double>(
    lat(slip_ratios.front_left, slip_angles.front_left, F_z.front_left, params.front_left),
    lat(slip_ratios.front_right, slip_angles.front_right, F_z.front_right, params.front_right),
    lat(slip_ratios.rear_left, slip_angles.rear_left, F_z.rear_left, params.rear_left),
    lat(slip_ratios.rear_right, slip_angles.rear_right, F_z.rear_right, params.rear_right));
}
template <typename T>
tam::types::common::DataPerWheel<double> TireModel<T>::self_aligning(
  const tam::types::common::DataPerWheel<double> & slip_ratios,
  const tam::types::common::DataPerWheel<double> & slip_angles,
  const tam::types::common::DataPerWheel<double> & F_z,
  const tam::types::common::DataPerWheel<T> & params) const
{
  return tam::types::common::DataPerWheel<double>(
    self_aligning(
      slip_ratios.front_left, slip_angles.front_left, F_z.front_left, params.front_left),
    self_aligning(
      slip_ratios.front_right, slip_angles.front_right, F_z.front_right, params.front_right),
    self_aligning(slip_ratios.rear_left, slip_angles.rear_left, F_z.rear_left, params.rear_left),
    self_aligning(
      slip_ratios.rear_right, slip_angles.rear_right, F_z.rear_right, params.rear_right));
}
template <typename T>
tam::types::common::DataPerWheel<tam::types::tire_models::TireModelOutput> TireModel<T>::eval(
  const tam::types::common::DataPerWheel<double> & slip_ratios,
  const tam::types::common::DataPerWheel<double> & slip_angles,
  const tam::types::common::DataPerWheel<double> & F_z,
  const tam::types::common::DataPerWheel<T> & params) const
{
  tam::types::common::DataPerWheel<tam::types::tire_models::TireModelOutput> out;
  tam::types::common::DataPerWheel<double> lon_forces = lon(slip_ratios, slip_angles, F_z, params);
  tam::types::common::DataPerWheel<double> lat_forces = lat(slip_ratios, slip_angles, F_z, params);
  tam::types::common::DataPerWheel<double> self_aligning_moments =
    self_aligning(slip_ratios, slip_angles, F_z, params);

  out.front_left.force =
    tam::types::common::Vector2D<double>(lon_forces.front_left, lat_forces.front_left);
  out.front_right.force =
    tam::types::common::Vector2D<double>(lon_forces.front_right, lat_forces.front_right);
  out.rear_left.force =
    tam::types::common::Vector2D<double>(lon_forces.rear_left, lat_forces.rear_left);
  out.rear_right.force =
    tam::types::common::Vector2D<double>(lon_forces.rear_right, lat_forces.rear_right);
  out.front_left.self_aligning_moment = self_aligning_moments.front_left;
  out.front_right.self_aligning_moment = self_aligning_moments.front_right;
  out.rear_left.self_aligning_moment = self_aligning_moments.rear_left;
  out.rear_right.self_aligning_moment = self_aligning_moments.rear_right;
  return out;
}
/******************************************************
 * @brief MF_simple tire model
 ******************************************************/
double MF_simple::lon(
  const double slip_ratio, const double slip_angle, const double F_z,
  const tam::types::tire_models::MF_simple & params) const
{
  // Combined slip not considered in MF simple model
  (void)slip_angle;
  return F_z * (params.lon.D *
                std::sin(
                  params.lat.C *
                  std::atan(
                    params.lon.B * slip_ratio -
                    params.lon.E * (params.lon.B * slip_ratio -
                                             std::atan(params.lon.B * slip_ratio)))));
}
double MF_simple::lat(
  const double slip_ratio, const double slip_angle, const double F_z,
  const tam::types::tire_models::MF_simple & params) const
{
  // Combined slip not considered in MF simple model
  (void)slip_ratio;
  return F_z *
         (params.lat.D *
          std::sin(
            params.lat.C *
            std::atan(
              params.lat.B * std::tan(slip_angle) -
              params.lat.E * (params.lat.B * std::tan(slip_angle) -
                                       std::atan(params.lat.B * std::tan(slip_angle))))));
}
double MF_simple::self_aligning(
  const double slip_ratio, const double slip_angle, const double F_z,
  const tam::types::tire_models::MF_simple & params) const
{
  // Self aligning moment not considered in MF simple model
  (void)slip_ratio;
  (void)slip_angle;
  (void)F_z;
  (void)params;
  return 0.0;
}
/******************************************************
 * @brief MF_52 tire model
 ******************************************************/
double MF_52::lon(
  const double slip_ratio, const double slip_angle, const double F_z,
  const tam::types::tire_models::MF_52 & params) const
{
  // Camber not modeled currently
  const double camber = 0.0;
  // Pure longitudinal slip
  const double F_z_o_a = params.LFZO * params.FNOMIN;
  const double dfz = (F_z - F_z_o_a) / F_z_o_a;
  const double S_H_x = (params.PHX1 + params.PHX2 * dfz) * params.LHX;
  const double S_V_x =
    F_z * (params.PVX1 + params.PVX2 * dfz) * params.LVX * params.LMUX;
  const double kappa_x = slip_ratio + S_H_x;
  const double C_x = params.PCX1 * params.LCX;
  const double mu_x = (params.PDX1 + params.PDX2 * dfz) * params.LMUX;
  const double D_x = mu_x * F_z;
  const double E_x =
    (params.PEX1 + params.PEX2 * dfz + params.PEX3 * std::pow(dfz, 2)) *
    (1.0 - params.PEX4 * std::copysign(1.0, kappa_x)) * params.LEX;
  const double K_x = F_z * (params.PKX1 + params.PKX2 * dfz) *
                     std::exp(params.PKX3 * dfz) * params.LKX;
  const double B_x = K_x / (C_x * D_x);
  // Longitudinal Force (pure longitudinal slip)
  const double F_x_o =
    D_x *
      std::sin(C_x * std::atan(B_x * kappa_x - E_x * (B_x * kappa_x - std::atan(B_x * kappa_x)))) +
    S_V_x;
  // Combined slip
  const double gamma_s = std::sin(camber);
  const double S_H_x_a = params.RHX1;
  const double E_x_a = params.REX1 + params.REX2 * dfz;
  const double C_x_a = params.RCX1;
  const double B_x_a = (params.RBX1 + params.RBX3 * std::pow(gamma_s, 2)) *
                       std::cos(std::atan(params.RBX2 * slip_ratio)) * params.LXAL;
  const double alpha_S = slip_angle + S_H_x_a;
  const double G_x_a_o = std::cos(
    C_x_a * std::atan(B_x_a * S_H_x_a - E_x_a * (B_x_a * S_H_x_a - std::atan(B_x_a * S_H_x_a))));
  const double G_x_a =
    std::cos(
      C_x_a * std::atan(B_x_a * alpha_S - E_x_a * (B_x_a * alpha_S - std::atan(B_x_a * alpha_S)))) /
    G_x_a_o;
  // Longitudinal Force (combined slip)
  const double F_x = G_x_a * F_x_o;
  return F_x;
}
double MF_52::lat(
  const double slip_ratio, const double slip_angle, const double F_z,
  const tam::types::tire_models::MF_52 & params) const
{
  // Camber not modeled currently
  const double camber = 0.0;
  // Pure lateral slip
  const double gamma_s = std::sin(camber);
  const double F_z_o_a = params.LFZO * params.FNOMIN;
  const double dfz = (F_z - F_z_o_a) / F_z_o_a;
  const double K_y_g_o = F_z * (params.PKY6 + params.PKY7 * dfz) * params.LKYG;
  const double S_V_y_g = F_z * (params.PVY3 + params.PVY4 * dfz) * gamma_s *
                         params.LKYG * params.LMUY;
  const double S_V_y =
    F_z * (params.PVY1 + params.PVY2 * dfz) * params.LVY * params.LMUY +
    S_V_y_g;
  const double K_y =
    params.PKY1 * F_z_o_a *
    std::sin(
      params.PKY4 *
      std::atan(F_z / ((params.PKY2 + params.PKY5 * std::pow(gamma_s, 2)) * F_z_o_a))) /
    (1.0 + params.PKY3 * std::pow(gamma_s, 2)) * params.LKY;
  const double S_H_y = (params.PHY1 + params.PHY2 * dfz) * params.LHY +
                       (K_y_g_o * gamma_s - S_V_y_g) / K_y;
  const double C_y = params.PCY1 * params.LCY;
  const double mu_y = ((params.PDY1 + params.PDY2 * dfz) /
                       (1.0 + params.PDY3 * std::pow(gamma_s, 2))) *
                      params.LMUY;
  const double D_y = mu_y * F_z;
  const double B_y = K_y / (C_y * D_y);
  const double alpha_y = slip_angle + S_H_y;
  const double E_y =
    (params.PEY1 + params.PEY2 * dfz) *
    (1.0 + params.PEY5 * std::pow(gamma_s, 2) -
     (params.PEY3 + params.PEY4 * gamma_s) * std::copysign(1.0, alpha_y)) *
    params.LEY;
  // Lateral Force (pure lateral slip)
  const double F_y_o =
    D_y *
      std::sin(C_y * std::atan(B_y * alpha_y - E_y * (B_y * alpha_y - std::atan(B_y * alpha_y)))) +
    S_V_y;
  // Combined slip
  const double D_V_y_k =
    mu_y * F_z * (params.RVY1 + params.RVY2 * dfz + params.RVY3 * gamma_s) *
    std::cos(std::atan(params.RVY4 * slip_angle));
  const double S_V_y_k =
    D_V_y_k * std::sin(params.RVY5 * std::atan(params.RVY6 * slip_ratio)) *
    params.LVYKA;
  const double S_H_y_k = params.RHY1 + params.RHY2 * dfz;
  const double E_y_k = params.REY1 + params.REY2 * dfz;
  const double C_y_k = params.RCY1;
  const double B_y_k =
    (params.RBY1 + params.RBY4 * std::pow(gamma_s, 2)) *
    std::cos(std::atan(params.RBY2 * (slip_angle - params.RBY3))) * params.LYKA;
  const double kappa_S = slip_ratio + S_H_y_k;
  const double G_y_k_o = std::cos(
    C_y_k * std::atan(B_y_k * S_H_y_k - E_y_k * (B_y_k * S_H_y_k - std::atan(B_y_k * S_H_y_k))));
  const double G_y_k =
    std::cos(
      C_y_k * std::atan(B_y_k * kappa_S - E_y_k * (B_y_k * kappa_S - std::atan(B_y_k * kappa_S)))) /
    G_y_k_o;
  // Lateral Force (combined slip)
  const double F_y = G_y_k * F_y_o + S_V_y_k;
  return F_y;
}
double MF_52::self_aligning(
  const double slip_ratio, const double slip_angle, const double F_z,
  const tam::types::tire_models::MF_52 & params) const
{
  // Self aligning moment not modeled in MF_52
  (void)slip_ratio;
  (void)slip_angle;
  (void)F_z;
  (void)params;
  return 0.0;
}
}  // namespace tam::helpers::tire_models
