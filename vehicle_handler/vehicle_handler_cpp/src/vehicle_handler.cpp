// Copyright 2024 Maximilian Leitenstern

#include "vehicle_handler_cpp/vehicle_handler.hpp"

#include "vehicle_handler_cpp/utility.hpp"
namespace tam::common
{
VehicleHandler::VehicleHandler()
{
  // Init overwrite path
  this->overwrite_path_ =
    ament_index_cpp::get_package_share_directory("vehicle_handler_cpp").append("/config_overwrite");

  // Init param managers for vehicle and tires
  this->param_manager_ = std::make_unique<tam::pmg::ParamReferenceManager>();
}
std::unique_ptr<VehicleHandler> VehicleHandler::from_pkg_config()
{
  std::unique_ptr<VehicleHandler> vh = std::unique_ptr<VehicleHandler>(new VehicleHandler());
  load_vehicle_name(vh, vh->overwrite_path_);
  declare_and_load_params(vh, vh->overwrite_path_);
  return vh;
}
std::unique_ptr<VehicleHandler> VehicleHandler::from_path(
  std::string path, std::string vehicle_name)
{
  std::unique_ptr<VehicleHandler> vh = std::unique_ptr<VehicleHandler>(new VehicleHandler());
  vh->vehicle_name_ = vehicle_name;
  declare_and_load_params(vh, path);
  return vh;
}
std::unordered_set<std::string> VehicleHandler::list_parameters() const
{
  // Get interface from param manager of vehicle handler
  const tam::pmg::MgmtInterface * interface = param_manager_.get();
  return interface->list_parameters();
}
void VehicleHandler::init_param_backend() const
{
  // Get interface from param manager of vehicle handler
  const tam::pmg::MgmtInterface * interface = param_manager_.get();
  std::vector<std::pair<std::string, tam::pmg::param_value_variant_t>> params;
  // Loop over all params from vehicle handler
  for (const auto & param : interface->list_parameters()) {
    tam::pmg::ParameterType type = interface->get_type(param);
// clang-format off
    #define ADD_PARAM_TO_BACKEND(enum_val, typename, name)                            \
      if (type == tam::pmg::ParameterType::enum_val) {                                \
        params.push_back(                                                             \
          std::make_pair(param, interface->get_value(param).as_##name())); \
        continue;                                                                     \
      }
    FOR_EVERY_SUPPORTED_PARAM_TYPE(ADD_PARAM_TO_BACKEND);
    #undef ADD_PARAM_TO_BACKEND
    // clang-format on
  }
  // Add all parameters to the backend
  tam::pmg::init(params);
}
void VehicleHandler::load_params(tam::pmg::MgmtInterface * pmg) const
{
  // Get interface from param manager of vehicle handler
  const tam::pmg::MgmtInterface * vh_interface = param_manager_.get();
  // Get inteface from param manager of input
  for (const auto & param : vh_interface->list_parameters()) {
    if (pmg->has_parameter(param)) {
      // Check param type compatibility
      tam::pmg::ParameterType vh_type = vh_interface->get_type(param);
      tam::pmg::ParameterType mod_type = pmg->get_type(param);
      if (vh_type != mod_type) {
        std::cout << "[VehicleHandler]: Unable to overload " << param << "! - Missmatching types!"
                  << std::endl;
        continue;
      }
// clang-format off
      // Set parameter in module depending on type
      #define SET_MODULE_PARAM(enum_val, typename, name)                                \
        if (vh_type == tam::pmg::ParameterType::enum_val) {                             \
          pmg->set_value(param, vh_interface->get_value(param).as_##name());            \
          std::cout << "[VehicleHandler]: Set parameter " << param << " in your module" \
            << std::endl;                                                               \
          continue;                                                                     \
        }
      FOR_EVERY_SUPPORTED_PARAM_TYPE(SET_MODULE_PARAM);
      #undef SET_MODULE_PARAM
      // clang-format on
    }
  }
}
void VehicleHandler::overwrite_param(
  const std::string & param_name, const tam::pmg::param_value_variant_t & value)
{
  tam::pmg::MgmtInterface & interface = *param_manager_;
  if (interface.has_parameter(param_name)) {
    interface.set_value(param_name, value);
    std::cout << "[VehicleHandler]: Parameter " << param_name << " overwritten!" << std::endl;
  } else {
    std::cout << "[VehicleHandler]: Unable to overwrite " << param_name
              << "! - Parameter does not exist in vehicle handler!" << std::endl;
  }
}
void VehicleHandler::load_vehicle_name(
  const std::unique_ptr<VehicleHandler> & vh, const std::string & config_path)
{
  std::filesystem::path config_file_path = std::filesystem::path(config_path).append("config.yaml");
  if (!std::filesystem::exists(config_file_path)) {
    throw std::runtime_error("Config file not found: " + config_file_path.string());
  }
  YAML::Node config = YAML::LoadFile(config_file_path);
  if (config["vehicle"]) {
    vh->vehicle_name_ = config["vehicle"].as<std::string>();
    if (boost::starts_with(vh->vehicle_name_, "$")) {
      // clang-format off
      vh->vehicle_name_.erase(std::remove_if(vh->vehicle_name_.begin(), vh->vehicle_name_.end(), boost::is_any_of("$")), vh->vehicle_name_.end()); // NOLINT
      // clang-format on
      char * env_value = std::getenv(vh->vehicle_name_.c_str());
      if (env_value == nullptr) {
        // clang-format off
        throw std::runtime_error("Environment variable not found \"" + vh->vehicle_name_ + std::string{"\" not found"}); // NOLINT
        // clang-format on
      }
      vh->vehicle_name_ = std::string(env_value);
    }
  } else {
    // clang-format off
    throw std::runtime_error(std::string{"Value "} + std::string{"\"vehicle\""} + std::string{" not found in "} + config_file_path.string());  // NOLINT
    // clang-format on
  }
}
void VehicleHandler::declare_and_load_params(
  const std::unique_ptr<VehicleHandler> & vh, const std::string & config_path)
{
  // Assign vehicle string keys to param manager
  // -> Adjust this function in case you want to add a new parameter
  utils::assign_string_keys_to_storage(vh->param_manager_, vh->vehicle_, vh->tires_);
  utils::set_from_config(vh->param_manager_, config_path, vh->vehicle_name_);
  std::cout << "[VehicleHandler]: Created vehicle " << vh->vehicle_name_ << "!" << std::endl;
}
/******************************************************
 * Vehicle dynamics calculations
 ******************************************************/
tam::types::vehicle_params::AeroModelOutput VehicleHandler::calc_aerodynamics(
  const tam::types::common::Vector3D<double> & velocity) const
{
  return tam::helpers::aeordynamics::eval_model(velocity, vehicle_.aero);
}
tam::types::common::DataPerWheel<double> VehicleHandler::calc_dynamic_tire_radius(
  const tam::types::common::Vector3D<double> & velocity) const
{
  return tam::helpers::vehicle_dynamics::dynamic_tire_radius(velocity.x, tires_);
}
tam::types::common::DataPerWheel<double> VehicleHandler::calc_long_slip(
  const tam::types::control::Odometry & odom, const double & steering_angle,
  const tam::types::common::DataPerWheel<double> & wheelspeeds) const
{
  return tam::helpers::vehicle_dynamics::long_slip(
    odom, steering_angle, wheelspeeds, tires_, vehicle_.dimension);
}
tam::types::common::DataPerWheel<double> VehicleHandler::calc_slip_angles(
  const tam::types::control::Odometry & odom, const double steering_angle) const
{
  return tam::helpers::vehicle_dynamics::slip_angle(odom, steering_angle, vehicle_.dimension);
}
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>
VehicleHandler::calc_slip_angles_vectorized(
  const std::vector<double> & vx, const std::vector<double> & vy,
  const std::vector<double> & yawrate, const std::vector<double> & steering_angle) const
{
  // Step 1: Check if all vectors are of the same length
  size_t size = vx.size();
  if (vy.size() != size || yawrate.size() != size || steering_angle.size() != size) {
    throw std::invalid_argument(
      "All input vectors (vx, vy, yawrate, steering_angle) must be of the same length.");
  }

  // Step 2: Initialize result vectors
  std::vector<double> fl;
  std::vector<double> fr;
  std::vector<double> rl;
  std::vector<double> rr;

  fl.reserve(size);
  fr.reserve(size);
  rl.reserve(size);
  rr.reserve(size);

  // Step 3: Process each set of inputs
  for (size_t i = 0; i < size; ++i) {
    // Create and populate the Odometry message
    tam::types::control::Odometry odom;
    odom.velocity_mps.x = vx[i];
    odom.velocity_mps.y = vy[i];
    odom.angular_velocity_radps.z = yawrate[i];

    double steer = steering_angle[i];

    // Call the existing calc_slip_angles function
    tam::types::common::DataPerWheel<double> slip = calc_slip_angles(odom, steer);

    // Unpack the results into respective vectors
    fl.push_back(slip.front_left);
    fr.push_back(slip.front_right);
    rl.push_back(slip.rear_left);
    rr.push_back(slip.rear_right);
  }

  return std::make_tuple(fl, fr, rl, rr);
}
}  // namespace tam::common
