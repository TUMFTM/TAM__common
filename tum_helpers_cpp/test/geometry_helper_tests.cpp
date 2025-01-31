// Copyright 2023 Simon Hoffmann
#include <gtest/gtest.h>

#include <vector>

#include "tum_helpers_cpp/geometry/geometry.hpp"
using tam::helpers::geometry::calc_heading;
using tam::helpers::geometry::normalize_angle;
using tam::helpers::geometry::to_deg;
using tam::helpers::geometry::to_rad;
TEST(geometry_helpers, s_coordinate)
{
  std::vector<double> x{1.0, 2.0, 3.0, 4.0, 6.0};
  std::vector<double> y{0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> z{0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> s = tam::helpers::geometry::create_s_coordinate_from_points(x, y, z);
  std::vector<double> segm_length = tam::helpers::geometry::calc_segment_length(s);
  ASSERT_EQ(static_cast<int>(s.size()), 5);
  ASSERT_EQ(static_cast<int>(segm_length.size()), 4);
  ASSERT_FLOAT_EQ(segm_length.back(), 2.0);
}
TEST(geometry_helpers, heading)
{
  ASSERT_FLOAT_EQ(to_deg(calc_heading(Eigen::Vector2d(1, 1))), 45.0);
  ASSERT_FLOAT_EQ(to_deg(calc_heading(Eigen::Vector2d(-1, 1))), 135.0);
  ASSERT_FLOAT_EQ(to_deg(calc_heading(Eigen::Vector2d(-1, -1))), -135.0);
  ASSERT_FLOAT_EQ(to_deg(calc_heading(Eigen::Vector2d(1, -1))), -45.0);
  ASSERT_FLOAT_EQ(to_deg(calc_heading(Eigen::Vector2d(-1, 0))), 180.0);
  ASSERT_FLOAT_EQ(to_deg(calc_heading(Eigen::Vector2d(1, 0))), 0);
  // use different zero axis
  ASSERT_FLOAT_EQ(to_deg(calc_heading(Eigen::Vector2d(0, 1), Eigen::Vector2d(1, 0))), 90);
  ASSERT_FLOAT_EQ(to_deg(calc_heading(Eigen::Vector2d(0, -1), Eigen::Vector2d(1, 0))), -90);
}
TEST(geometry_helpers, angle_conversion)
{
  ASSERT_FLOAT_EQ(to_deg(M_PI), 180);
  ASSERT_FLOAT_EQ(to_rad(180), M_PI);
}
TEST(geometry_helpers, angle_normalization)
{
  ASSERT_FLOAT_EQ(normalize_angle(1.5 * M_PI), -0.5 * M_PI);
  ASSERT_FLOAT_EQ(normalize_angle(0.5 * M_PI), 0.5 * M_PI);
  ASSERT_FLOAT_EQ(normalize_angle(-0.5 * M_PI), -0.5 * M_PI);
  ASSERT_FLOAT_EQ(normalize_angle(-1.5 * M_PI), 0.5 * M_PI);
  ASSERT_FLOAT_EQ(normalize_angle(-2 * M_PI), 0 * M_PI);
  ASSERT_FLOAT_EQ(normalize_angle(2.5 * M_PI), 0.5 * M_PI);
  ASSERT_FLOAT_EQ(normalize_angle(10.5 * M_PI), 0.5 * M_PI);
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
