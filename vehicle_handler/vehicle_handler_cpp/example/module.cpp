// Copyright 2024 Maximilian Leitenstern

#include <iostream>

#include "param_management_cpp/param_value_manager.hpp"
#include "vehicle_handler_cpp/vehicle_handler.hpp"
class SoftwareModule
{
public:
  // Constructor
  SoftwareModule() { set_params(); }
  // Some vehicle dynamics function
  void calc_dynamics()
  {
    // Exemplary calculation of dynamic tire radius using vehicle handler
    // imagine you get the velocity from somewhere and want to calculate the dynamic tire radius
    tam::types::common::Vector3D<double> velocity(75.0, 1.0, 0.0);
    tam::types::common::DataPerWheel<double> tire_radii =
      this->vehicle_->calc_dynamic_tire_radius(velocity);
    std::cout << "Dynamic tire radii: " << tire_radii.front_left << ", " << tire_radii.front_right
              << ", " << tire_radii.rear_left << ", " << tire_radii.rear_right << std::endl;
  }
  // Function to return reference to vehicle handler
  std::unique_ptr<tam::common::VehicleHandler> & vehicle_handler() { return vehicle_; }
  // Function to return param manager
  tam::pmg::MgmtInterface* param_manager() { return module_pmg_.get(); }

private:
  void set_params()
  {
    // Naming of parameters is important for compatibility with the vehicle handler
    // -> naming according to yaml file with dot-indexing
    // clang-format off
    module_pmg_->declare_parameter("vehicle.dimension.track_width_front", 0.0, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
    module_pmg_->declare_parameter("vehicle.dimension.track_width_rear", 0.0, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
    module_pmg_->declare_parameter("vehicle.dimension.cog_height", 0.0, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
    module_pmg_->declare_parameter("tires.front_left.radius.radius_20mps", 0.0, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
    // clang-format on

    // Overload params of module_pmg_ with vehicle handler params
    // -> name of parameters need to match!!!
    vehicle_->load_params(this->param_manager());
  }
  // clang-format off
  std::unique_ptr<tam::common::VehicleHandler> vehicle_ = tam::common::VehicleHandler::from_pkg_config(); // NOLINT
  tam::pmg::ParamValueManager::SharedPtr module_pmg_ = std::make_shared<tam::pmg::ParamValueManager>(); // NOLINT
  // clang-format on
};
int main()
{
  SoftwareModule module;

  // clang-format off
    // Access params directly from vehicle handler -> fast (no lookup required via param manager)
    std::cout << "Track width front: " << module.vehicle_handler()->vehicle().dimension.track_width_front << std::endl; // NOLINT
    std::cout << "Tire radius @ 20mps: " << module.vehicle_handler()->tires().front_left.radius.radius_20mps << std::endl; // NOLINT
    // Access params via param manager -> slow (lookup required)
    std::cout << "Track width rear: " << module.param_manager()->get_value("vehicle.dimension.track_width_rear").as_double() << std::endl; // NOLINT
    std::cout << "Tire radius @ 20mps: " << module.param_manager()->get_value("tires.front_left.radius.radius_20mps").as_double() << std::endl; // NOLINT
    module.calc_dynamics();
  // clang-format on
  return 0;
}
