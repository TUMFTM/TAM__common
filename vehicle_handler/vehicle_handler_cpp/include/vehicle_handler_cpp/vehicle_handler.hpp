// Copyright 2024 Maximilian Leitenstern
#pragma once

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "param_management_cpp/initialization.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tum_helpers_cpp/aerodynamics.hpp"
#include "tum_helpers_cpp/tire_models.hpp"
#include "tum_helpers_cpp/vehicle_dynamics.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"
#include "tum_types_cpp/tire.hpp"
#include "tum_types_cpp/vehicle.hpp"
namespace tam::common
{
class VehicleHandler
{
  /******************************************************
   * Parameter management
   ******************************************************/
public:
  /**
   * @brief Creates a VehicleHandler with the config provided in config_overwrite/
   * @return std::unique_ptr<VehicleHandler>
   */
  static std::unique_ptr<VehicleHandler> from_pkg_config();
  /**
   * @brief Creates a VehicleHandler with the config provided in path
   * @return std::unique_ptr<VehicleHandler>
   */
  static std::unique_ptr<VehicleHandler> from_path(std::string path, std::string vehicle_name);
  /**
   * @brief Returns the vehicle name
   * @return std::string
   */
  std::string get_vehicle_name() const { return vehicle_name_; }
  /**
   * @brief Return const vehicle struct for parameter access
   * @return tam::types::vehicle_params::Vehicle
   */
  tam::types::vehicle_params::Vehicle vehicle() const { return vehicle_; }
  /**
   * @brief Return tire struct for parameter access
   * @return tam::types::common::DataPerWheel<tam::types::tire_params::Tire>
   */
  tam::types::common::DataPerWheel<tam::types::tire_params::Tire> tires() const { return tires_; }
  /**
   * @brief List all parameters of the vehicle handler
   * @return std::unordered_set<std::string>
   */
  std::unordered_set<std::string> list_parameters() const;
  /**
   * @brief Initialize the parameter backend of the param manager
   * -> required to get parameters during construction of modules
   */
  void init_param_backend() const;
  /**
   * @brief Loads (i.e. overwrites) parameters of your module param manager with the
   * ones of the vehicle handler
   * -> requires that the name of the parameter and its type is the same
   */
  void load_params(tam::pmg::MgmtInterface * pmg) const;
  /**
   * @brief Overwrites a parameter of the vehicle handler
   * !! CAREFUL !! - NOT RECOMMENDED !!
   * -> this will cause differences between multiple vehicle handlers !!
   */
  void overwrite_param(
    const std::string & param_name, const tam::pmg::param_value_variant_t & value);

private:
  /**
   * @brief Constructor to get config folder from install directory and init param manager
   */
  VehicleHandler();
  /**
   * @brief Load vehicle name from config file
   * @param vh
   * @param config_path
   */
  static void load_vehicle_name(
    const std::unique_ptr<VehicleHandler> & vh, const std::string & config_path);
  /**
   * @brief Declare and load parameters from config files
   * @param vh
   * @param config_path
   */
  static void declare_and_load_params(
    const std::unique_ptr<VehicleHandler> & vh, const std::string & config_path);

private:
  /**
   * @brief Class variables
   */
  std::filesystem::path overwrite_path_;
  tam::pmg::ParamReferenceManager::UniquePtr param_manager_;
  std::string vehicle_name_{""};
  tam::types::vehicle_params::Vehicle vehicle_;
  tam::types::common::DataPerWheel<tam::types::tire_params::Tire> tires_;

  /******************************************************
   * Vehicle dynamics calculations
   ******************************************************/
public:
  /**
   * @brief Calculate the aero forces and torques of the vehicle
   * @param velocity -> velocity vector of the vehicle
   * @return std::tuple<tam::types::common::Vector3D<double>, tam::types::common::Vector3D<double>>
   * (force, torque)
   */
  tam::types::vehicle_params::AeroModelOutput calc_aerodynamics(
    const tam::types::common::Vector3D<double> & velocity) const;

  /**
   * @brief Calculate the dynamic tire radius of the vehicle
   * @param wheelspeeds
   * @return tam::types::common::DataPerWheel<double>
   */
  tam::types::common::DataPerWheel<double> calc_dynamic_tire_radius(
    const tam::types::common::Vector3D<double> & velocity) const;

  /**
   * @brief Calculate the slip ratios of the vehicle
   * @param odom
   * @param wheelspeeds
   * @return tam::types::common::DataPerWheel<double>
   */
  tam::types::common::DataPerWheel<double> calc_long_slip(
    const tam::types::control::Odometry & odom, const double & steering_angle,
    const tam::types::common::DataPerWheel<double> & wheelspeeds) const;

  /**
   * @brief Calculate the slip ratios of the tires
   * @param vx std::vector<double> of x velocities
   * @param wheelspeeds_fl std::vector<double> of front left wheel speeds
   * @param wheelspeeds_fr std::vector<double> of front right wheel speeds
   * @param wheelspeeds_rl std::vector<double> of rear left wheel speeds
   * @param wheelspeeds_rr std::vector<double> of rear right wheel speeds
   * @return std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
   * std::vector<double>>
   */
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>
  calc_long_slip_vectorized(
    const std::vector<double> & vx, const std::vector<double> & wheelspeeds_fl,
    const std::vector<double> & wheelspeeds_fr, const std::vector<double> & wheelspeeds_rl,
    const std::vector<double> & wheelspeeds_rr) const;

  /**
   * @brief Calculate the slip angles of the vehicle
   * @param tam::types::control::Odometry
   * @param double steering_angle
   * @return tam::types::common::DataPerWheel<double>
   */
  tam::types::common::DataPerWheel<double> calc_slip_angles(
    const tam::types::control::Odometry & odom, const double steering_angle) const;

  /**
   * @brief Calculate the slip angles of the tires
   * @param vx std::vector<double> of x velocities
   * @param vy std::vector<double> of y velocities
   * @param yawrate std::vector<double> of yaw rates
   * @param steering_angle std::vector<double> of steering angles
   * @return std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
   * std::vector<double>>
   */
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>
  calc_slip_angles_vectorized(
    const std::vector<double> & vx, const std::vector<double> & vy,
    const std::vector<double> & yawrate, const std::vector<double> & steering_angle) const;
};
}  // namespace tam::common
