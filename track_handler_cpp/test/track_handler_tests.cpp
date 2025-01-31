// Copyright 2023 Simon Hoffmann
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>

#include "track_handler_cpp/track.hpp"
class monza_track : public ::testing::Test
{
public:
  monza_track()
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("track_handler_cpp");

    th = tam::common::Track::create_from_csv(
      package_share_directory + "/test_data/tracks/monza_2d_curbs_v0_1_smoothed.csv");
  }
  std::unique_ptr<tam::common::Track> th;
};
TEST_F(monza_track, on_track)
{
  Eigen::Vector2d pt{-180, -95};
  ASSERT_TRUE(!th->on_track(pt[0], pt[1], 0.0));
}
TEST_F(monza_track, sn2cartesian)
{
  Eigen::Vector3d vec = th->sn2cartesian(10, 0.3);
  ASSERT_TRUE((vec(2) == 0.0));

  Eigen::VectorXd s(4);
  s << 2.0, 3.0, 4.0, 5.0;
  Eigen::VectorXd n(4);
  n << 0.1, 0.2, 0.3, 0.4;
  auto xyz = th->sn2cartesian(s, n);
  ASSERT_EQ(xyz.rows(), s.size());
  ASSERT_EQ(xyz.cols(), 3);
}
TEST_F(monza_track, project_2d_point_on_track)
{
  Eigen::Vector2d vec = th->project_2d_point_on_track(-155, 110);
  ASSERT_LE(1.4642644 - vec(1), 1e-5);

  Eigen::VectorXd x(4);
  x << -155, -155, -156, -156;
  Eigen::VectorXd y(4);
  y << 114, 116, 118, 120;
  auto sn = th->project_2d_point_on_track(x, y);
  ASSERT_EQ(sn.rows(), x.size());
  ASSERT_EQ(sn.cols(), 2);
}
TEST_F(monza_track, calc_2d_heading_from_chi)
{
  // Function tested separetly (rotation_tests.cpp) -> just check if different dim are accepted
  double yaw = th->calc_2d_heading_from_chi(10, 0.3);
  ASSERT_FLOAT_EQ(yaw, yaw);
  Eigen::VectorXd s(4);
  s << 2, 3, 4, 5;
  Eigen::VectorXd chi(4);
  chi << 0.1, 0.2, 0.3, 0.4;
  Eigen::VectorXd yaw_vec = th->calc_2d_heading_from_chi(s, chi);
  ASSERT_EQ(yaw_vec.size(), s.size());
}
TEST_F(monza_track, calc_chi_from_2d_heading)
{
  // Function tested separetly (rotation_tests.cpp) -> just check if different dim are accepted
  double chi = th->calc_chi_from_2d_heading(10, 0.3);
  ASSERT_FLOAT_EQ(chi, chi);
  Eigen::VectorXd s(4);
  s << 2, 3, 4, 5;
  Eigen::VectorXd heading(4);
  heading << 0.1, 0.2, 0.3, 0.4;
  Eigen::VectorXd chi_vec = th->calc_chi_from_2d_heading(s, heading);
  ASSERT_EQ(chi_vec.size(), s.size());
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
