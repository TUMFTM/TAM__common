// Copyright 2023 Simon Hoffmann
#include "track_handler_cpp/race_track_handler.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
namespace tam::common
{
namespace fs = std::filesystem;
RaceTrackHandler::RaceTrackHandler()
{
  overwrite_path =
    ament_index_cpp::get_package_share_directory("track_handler_cpp").append("/config_overwrite");
}
std::unique_ptr<RaceTrackHandler> RaceTrackHandler::from_pkg_config()
{  // make unique doesn't allow protected access
  std::unique_ptr<RaceTrackHandler> th = std::unique_ptr<RaceTrackHandler>(new RaceTrackHandler());
  load_track_name(th, th->overwrite_path);
  declare_and_load_params(th, th->overwrite_path);
  return th;
}
std::string RaceTrackHandler::get_track_path_default(const std::string & track_key) const
{
  return fs::path(overwrite_path).append(track_key).append("raceline").string();
}
std::string RaceTrackHandler::get_raceline_path_default(const std::string & track_key) const
{
  return fs::path(overwrite_path).append(track_key).append("raceline").string();
}
void RaceTrackHandler::load_track_name(
  std::unique_ptr<RaceTrackHandler> & th, const std::string & config_folder)
{
  fs::path path = fs::path(config_folder).append("config.yml");
  if (!fs::exists(path)) {
    throw std::invalid_argument("Config file not found: " + path.string());
  }
  YAML::Node config = YAML::LoadFile(path.string());
  if (config["track"]) {
    th->track = config["track"].as<std::string>();
    if (boost::starts_with(th->track, "$")) {
      th->track.erase(boost::remove_if(th->track, boost::is_any_of("${}")), th->track.end());
      th->track = load_from_env(th->track);
    }
  } else {
    throw std::invalid_argument(
      std::string{"Value "} + std::string{"\"track\""} + std::string{" not set in file: "} +
      path.string());
  }
}
std::string RaceTrackHandler::load_from_env(const std::string & key)
{
  char * val = getenv(key.c_str());
  if (val == NULL) {
    throw std::invalid_argument(
      std::string{"Environment Variable \""} + key + std::string{"\" not found"});
  }

  return std::string(val);
}
void RaceTrackHandler::declare_and_load_params(
  std::unique_ptr<RaceTrackHandler> & th, const std::string & config_folder)
{
  // Declare
  th->param_manager_->declare_parameter("global.pitlane", " ", tam::pmg::ParameterType::STRING, "");
  th->param_manager_->declare_parameter(
    "global.raceline", " ", tam::pmg::ParameterType::STRING, "");
  th->param_manager_->declare_parameter(
    "global.initial_heading", 0.0, tam::pmg::ParameterType::DOUBLE, "");
  th->param_manager_->declare_parameter(
    "global.geo_origin", std::vector<double>{0.0, 0.0, 0.0}, tam::pmg::ParameterType::DOUBLE_ARRAY,
    "");
  th->param_manager_->declare_parameter(
    "prediction.pitlane", " ", tam::pmg::ParameterType::STRING, "");
  th->param_manager_->declare_parameter(
    "prediction.raceline", " ", tam::pmg::ParameterType::STRING, "");
  // Simulation
  th->param_manager_->declare_parameter(
    "simulation.start_pos_1", std::vector<double>{0.0, 0.0, 0.0},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");
  th->param_manager_->declare_parameter(
    "simulation.start_pos_2", std::vector<double>{0.0, 0.0, 0.0},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");
  th->param_manager_->declare_parameter(
    "simulation.start_pos_3", std::vector<double>{0.0, 0.0, 0.0},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");
  th->param_manager_->declare_parameter(
    "simulation.start_pos_4", std::vector<double>{0.0, 0.0, 0.0},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");

  fs::path path = fs::path(config_folder).append(th->track).append("config.yml");
  tam::pmg::load_overwrites_from_yaml(
    th->param_manager_.get(), path.string(), "/TrackHandler", true, true);
}
std::unique_ptr<Track> RaceTrackHandler::create_track() const
{
  std::cout << "[RaceTrackHandler.create_track()]: DEPRECATED: use "
               "create_raceline_track() "
            << "\n";
  return create_raceline_track();
}
std::unique_ptr<Track> RaceTrackHandler::create_raceline_track() const
{
  fs::path path = fs::path(overwrite_path).append(track).append("raceline").append(get_track_file());
  std::cout << "[RaceTrackHandler]: Loading Track - " << path << "\n";
  return Track::create_from_csv(path, TrackReferenceLines::RACELINE);
}
std::unique_ptr<Track> RaceTrackHandler::create_centerline_track() const
{
  fs::path path = fs::path(overwrite_path).append(track).append("raceline").append(get_track_file());
  std::cout << "[RaceTrackHandler]: Loading Track - " << path << "\n";
  return Track::create_from_csv(path, TrackReferenceLines::CENTERLINE);
}
std::unique_ptr<Track> RaceTrackHandler::create_pitlane() const
{
  fs::path path = fs::path(overwrite_path).append(track).append("raceline").append(get_pit_file());
  std::cout << "[RaceTrackHandler]: Loading Pitlane - " << path << "\n";
  return Track::create_from_csv(path);
}
std::unique_ptr<Raceline> RaceTrackHandler::create_raceline() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_raceline_file());
  std::cout << "[RaceTrackHandler]: Loading Raceline - " << path << "\n";
  return Raceline::create_from_csv(path);
}
std::unique_ptr<Track> RaceTrackHandler::create_track_prediction() const
{
  std::cout << "[RaceTrackHandler.create_track_prediction()]: DEPRECATED: use "
               "create_raceline_track_prediction() "
            << "\n";
  return create_raceline_track_prediction();
}
std::unique_ptr<Track> RaceTrackHandler::create_raceline_track_prediction() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_track_file_pred());
  std::cout << "[RaceTrackHandler]: Loading Prediction Track - " << path << "\n";
  return Track::create_from_csv(path, TrackReferenceLines::RACELINE);
}
std::unique_ptr<Track> RaceTrackHandler::create_centerline_track_prediction() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_track_file_pred());
  std::cout << "[RaceTrackHandler]: Loading Prediction Track - " << path << "\n";
  return Track::create_from_csv(path, TrackReferenceLines::CENTERLINE);
}
std::unique_ptr<Track> RaceTrackHandler::create_pitlane_prediction() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_pit_file_pred());
  std::cout << "[RaceTrackHandler]: Loading Prediction Pitlane - " << path << "\n";
  return Track::create_from_csv(path);
}
std::unique_ptr<Raceline> RaceTrackHandler::create_raceline_prediction() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_raceline_file_pred());
  std::cout << "[RaceTrackHandler]: Loading Prediction Raceline - " << path << "\n";
  return Raceline::create_from_csv(path);
}
std::string RaceTrackHandler::get_track_file() const
{
  return param_manager_->get_value("global.raceline").as_string();
}
std::string RaceTrackHandler::return_raceline_path() const
{
  return fs::path(overwrite_path).append(track).append("raceline").append(get_track_file()).string();
}
std::string RaceTrackHandler::get_pit_file() const
{
  return param_manager_->get_value("global.pitlane").as_string();
}
std::string RaceTrackHandler::return_pitlane_path() const
{
  return fs::path(overwrite_path).append(track).append("raceline").append(get_pit_file()).string();
}
std::string RaceTrackHandler::get_raceline_file() const
{
  return param_manager_->get_value("global.raceline").as_string();
}
double RaceTrackHandler::get_initial_heading() const
{
  return param_manager_->get_value("global.initial_heading").as_double();
}
std::string RaceTrackHandler::get_raceline_file_pred() const
{
  return param_manager_->get_value("prediction.raceline").as_string();
}
std::string RaceTrackHandler::get_track_file_pred() const
{
  return param_manager_->get_value("prediction.raceline").as_string();
}
std::string RaceTrackHandler::get_pit_file_pred() const
{
  return param_manager_->get_value("prediction.pitlane").as_string();
}
tam::types::common::Vector3D<double> RaceTrackHandler::get_geo_origin() const
{
  return tam::types::common::Vector3D<double>(
    param_manager_->get_value("global.geo_origin").as_double_array());
}
std::vector<double> RaceTrackHandler::get_sim_start_pos(int pos_id) const
{
  return param_manager_->get_value("simulation.start_pos_" + std::to_string(pos_id))
    .as_double_array();
}
tam::pmg::ParameterValue RaceTrackHandler::get_param(const std::string & param_name) const
{
  return param_manager_->get_value(param_name);
}
}  // namespace tam::common
