// Copyright 2023 Simon Hoffmann
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>

#include "track_handler_cpp/rotation.hpp"
#include "track_handler_cpp/track.hpp"
class rotation_tests_monza : public ::testing::Test
{
public:
  rotation_tests_monza()
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("track_handler_cpp");

    th = tam::common::Track::create_from_csv(
      package_share_directory + "/test_data/tracks/monza_3d_curbs_v0_1_smoothed.csv");
  }
  std::unique_ptr<tam::common::Track> th;
};
class rotation_tests_monza_2d : public ::testing::Test
{
public:
  rotation_tests_monza_2d()
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("track_handler_cpp");

    th = tam::common::Track::create_from_csv(
      package_share_directory + "/test_data/tracks/monza_2d_curbs_v0_1_smoothed.csv");
  }
  std::unique_ptr<tam::common::Track> th;
};
class rotation_tests_vegas : public ::testing::Test
{
public:
  rotation_tests_vegas()
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("track_handler_cpp");

    th = tam::common::Track::create_from_csv(
      package_share_directory + "/test_data/tracks/LVMS_3d_smoothed.csv");
  }
  std::unique_ptr<tam::common::Track> th;
};
TEST_F(rotation_tests_monza, heading_from_chi)
{
  // calc heading with constant chi
  double des_chi = 0.2;
  Eigen::VectorXd theta = th->theta();
  Eigen::VectorXd chi = Eigen::VectorXd::Zero(theta.rows()).array() + des_chi;
  Eigen::VectorXd heading =
    tam::common::track::calc_2d_heading_from_chi(chi, theta, th->mu(), th->phi());

  // normalize monza heading
  Eigen::VectorXd theta_n = tam::helpers::geometry::normalize_angle(theta);

  // difference between monza heading and calculation should be desired chi
  Eigen::VectorXd diff = tam::helpers::geometry::normalize_angle(theta_n.array() - heading.array());
  Eigen::VectorXd error = (diff.cwiseAbs().array() - des_chi).cwiseAbs();
  ASSERT_LE(error.maxCoeff(), 0.0006);
}
TEST_F(rotation_tests_monza, chi_from_heading)
{
  // calc heading with constant chi
  double des_chi = 0.3;
  Eigen::VectorXd theta = th->theta();
  Eigen::VectorXd heading = theta.array() + des_chi;

  Eigen::VectorXd chi =
    tam::common::track::calc_chi_from_2d_heading(heading, theta, th->mu(), th->phi());
  // difference between monza heading and calculation should be desired chi
  Eigen::VectorXd error = (chi.array() - des_chi).cwiseAbs();
  ASSERT_LE(error.maxCoeff(), 0.0015);
}
TEST_F(rotation_tests_monza_2d, heading_from_chi)
{
  // calc heading with constant chi
  double des_chi = 0.2;
  Eigen::VectorXd theta = th->theta();
  Eigen::VectorXd chi = Eigen::VectorXd::Zero(theta.rows()).array() + des_chi;
  Eigen::VectorXd heading =
    tam::common::track::calc_2d_heading_from_chi(chi, theta, th->mu(), th->phi());

  // normalize monza heading
  Eigen::VectorXd theta_n = tam::helpers::geometry::normalize_angle(theta);

  // difference between monza heading and calculation should be desired chi
  Eigen::VectorXd diff = tam::helpers::geometry::normalize_angle(theta_n.array() - heading.array());
  Eigen::VectorXd error = (diff.cwiseAbs().array() - des_chi).cwiseAbs();
  ASSERT_LE(error.maxCoeff(), 1e-12);
}
TEST_F(rotation_tests_monza_2d, chi_from_heading)
{
  // calc heading with constant chi
  double des_chi = 0.3;
  Eigen::VectorXd theta = th->theta();
  Eigen::VectorXd heading = theta.array() + des_chi;

  Eigen::VectorXd chi =
    tam::common::track::calc_chi_from_2d_heading(heading, theta, th->mu(), th->phi());
  // difference between monza heading and calculation should be desired chi
  Eigen::VectorXd error = (chi.array() - des_chi).cwiseAbs();
  ASSERT_LE(error.maxCoeff(), 1e-12);
}
TEST_F(rotation_tests_vegas, heading_from_chi)
{
  // calc heading with constant chi
  double des_chi = 0.2;
  Eigen::VectorXd theta = th->theta();
  Eigen::VectorXd chi = Eigen::VectorXd::Zero(theta.rows()).array() + des_chi;
  Eigen::VectorXd heading =
    tam::common::track::calc_2d_heading_from_chi(chi, theta, th->mu(), th->phi());

  // normalize monza heading
  Eigen::VectorXd theta_n = tam::helpers::geometry::normalize_angle(theta);

  // difference between monza heading and calculation should be desired chi
  Eigen::VectorXd diff = tam::helpers::geometry::normalize_angle(theta_n.array() - heading.array());
  Eigen::VectorXd error = (diff.cwiseAbs().array() - des_chi).cwiseAbs();
  ASSERT_LE(error.maxCoeff(), 0.012);
}
TEST_F(rotation_tests_vegas, chi_from_heading)
{
  // calc heading with constant chi
  double des_chi = 0.3;
  Eigen::VectorXd theta = th->theta();
  Eigen::VectorXd heading = theta.array() + des_chi;

  Eigen::VectorXd chi =
    tam::common::track::calc_chi_from_2d_heading(heading, theta, th->mu(), th->phi());
  // difference between monza heading and calculation should be desired chi
  Eigen::VectorXd error = (chi.array() - des_chi).cwiseAbs();
  ASSERT_LE(error.maxCoeff(), 0.018);
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
