// Copyright 2023 Simon Hoffmann
#include <gtest/gtest.h>
#include <math.h>

#include <vector>

#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"
#include "tum_helpers_cpp/geometry/geometry.hpp"
class curvilinear_cosy_open : public ::testing::Test
{
public:
  curvilinear_cosy_open()
  {
    Eigen::VectorXd x(4), y(4), z(4);
    x << -1, -1, 1, 1;
    y << 0, 1, 1, 0;
    z << 0, 0, 0, 0;
    cosy = tam::helpers::cosy::CurvilinearCosy::create(x, y, z)->build();
  }
  tam::helpers::cosy::CurvilinearCosyPtr cosy;
};
class curvilinear_cosy_open_straight : public ::testing::Test
{
public:
  curvilinear_cosy_open_straight()
  {
    Eigen::VectorXd x(4), y(4), z(4);
    x << 1, 2, 3, 4;
    y << 0, 0, 0, 0;
    z << 0, 0, 0, 0;
    cosy = tam::helpers::cosy::CurvilinearCosy::create(x, y, z)->build();
  }
  tam::helpers::cosy::CurvilinearCosyPtr cosy;
};
class curvilinear_cosy_closed : public ::testing::Test
{
public:
  curvilinear_cosy_closed()
  {
    Eigen::VectorXd x(5), y(5), z(5);
    x << -1, -1, 1, 1, -1;
    y << 0, 1, 1, 0, 0;
    z << 0, 0, 0, 0, 0;
    cosy = tam::helpers::cosy::CurvilinearCosy::create(x, y, z)->build();
  }
  tam::helpers::cosy::CurvilinearCosyPtr cosy;
};
//
//
// TESTS
//
//
TEST_F(curvilinear_cosy_closed, check_setup)
{
  std::vector<double> segment_start_s = cosy->get_segment_starting_s();
  std::vector<double> segment_length = cosy->get_segment_length();
  double length = cosy->get_ref_line_length();

  ASSERT_TRUE(cosy->is_closed());
  ASSERT_EQ(static_cast<int>(segment_start_s.size()), 4);
  ASSERT_EQ(static_cast<int>(segment_length.size()), 4);
  ASSERT_FLOAT_EQ(segment_start_s.at(3), 4.0);
  ASSERT_FLOAT_EQ(length, 6);
}
TEST_F(curvilinear_cosy_open, check_setup)
{
  Eigen::VectorXd x(4), y(4), z(4);
  x << -1, -1, 1, 1;
  y << 0, 1, 1, 0;
  z << 0, 0, 0, 0;
  auto cosy = tam::helpers::cosy::CurvilinearCosy::create(x, y, z)->build();
  std::vector<double> segment_start_s = cosy->get_segment_starting_s();
  std::vector<double> segment_length = cosy->get_segment_length();

  ASSERT_TRUE(!cosy->is_closed());
  ASSERT_EQ(static_cast<int>(segment_start_s.size()), 3);
  ASSERT_EQ(static_cast<int>(segment_length.size()), 3);
  ASSERT_FLOAT_EQ(segment_start_s.at(2), 3);
  ASSERT_FLOAT_EQ(cosy->get_ref_line_length(), 4);
}
TEST(curvilinear_cosy, set_s_open)
{
  Eigen::VectorXd x(4), y(4), z(4), s(4);
  x << -1, -1, 1, 1;
  y << 0, 1, 1, 0;
  z << 0, 0, 0, 0;
  s << 0, 1.1, 3.1, 4.2;
  auto cosy = tam::helpers::cosy::CurvilinearCosy::create(x, y, z)->set_s(s)->build();
  std::vector<double> segment_start_s = cosy->get_segment_starting_s();
  std::vector<double> segment_length = cosy->get_segment_length();

  ASSERT_TRUE(!cosy->is_closed());
  ASSERT_EQ(static_cast<int>(segment_start_s.size()), 3);
  ASSERT_EQ(static_cast<int>(segment_length.size()), 3);
  ASSERT_FLOAT_EQ(segment_start_s.at(2), 3.1);
  ASSERT_FLOAT_EQ(cosy->get_ref_line_length(), 4.2);
}
TEST(curvilinear_cosy, set_s_closed)
{
  Eigen::VectorXd x(5), y(5), z(5), s(5);
  x << -1, -1, 1, 1, -1;
  y << 0, 1, 1, 0, 0;
  z << 0, 0, 0, 0, 0;
  s << 0, 1.1, 3.1, 4.2, 6.1;
  auto cosy = tam::helpers::cosy::CurvilinearCosy::create(x, y, z)->set_s(s)->build();
  std::vector<double> segment_start_s = cosy->get_segment_starting_s();
  std::vector<double> segment_length = cosy->get_segment_length();

  ASSERT_TRUE(cosy->is_closed());
  ASSERT_EQ(static_cast<int>(segment_start_s.size()), 4);
  ASSERT_EQ(static_cast<int>(segment_length.size()), 4);
  ASSERT_FLOAT_EQ(segment_start_s.at(3), 4.2);
  ASSERT_FLOAT_EQ(cosy->get_ref_line_length(), 6.1);
}
TEST_F(curvilinear_cosy_open_straight, in_bounds)
{
  Eigen::Vector2d pos1 = cosy->convert_to_sn(1.2, 3);
  Eigen::Vector2d pos2 = cosy->convert_to_sn(3.9, -0.5);
  ASSERT_FLOAT_EQ(pos1[0], 0.2);
  ASSERT_FLOAT_EQ(pos1[1], 3);
  ASSERT_FLOAT_EQ(pos2[0], 2.9);
  ASSERT_FLOAT_EQ(pos2[1], -0.5);
}
TEST_F(curvilinear_cosy_open_straight, out_of_bounds)
{
  Eigen::Vector2d pos1 = cosy->convert_to_sn(0.7, 3);
  Eigen::Vector2d pos2 = cosy->convert_to_sn(4.2, -0.5);
  ASSERT_FLOAT_EQ(pos1[0], -0.3);
  ASSERT_FLOAT_EQ(pos1[1], 3);
  ASSERT_FLOAT_EQ(pos2[0], 3.2);
  ASSERT_FLOAT_EQ(pos2[1], -0.5);
}
TEST_F(curvilinear_cosy_closed, in_bounds)
{
  Eigen::Vector2d pos1 = cosy->convert_to_sn(-1, 1);
  Eigen::Vector2d pos2 = cosy->convert_to_sn(-1, 0);
  Eigen::Vector2d pos3 = cosy->convert_to_sn(-1, -1);
  Eigen::Vector2d pos4 = cosy->convert_to_sn(0.8, 1.2);
  ASSERT_FLOAT_EQ(pos1[0], 1);
  ASSERT_FLOAT_EQ(pos1[1], 0);
  ASSERT_FLOAT_EQ(pos2[0], 6);
  ASSERT_FLOAT_EQ(pos2[1], 0);
  ASSERT_FLOAT_EQ(roundf(pos3[0] * 100) / 100, 5.67);
  ASSERT_FLOAT_EQ(roundf(pos3[1] * 100) / 100, 1.05);
  ASSERT_FLOAT_EQ(roundf(pos4[0] * 100) / 100, 2.73);
  ASSERT_FLOAT_EQ(roundf(pos4[1] * 100) / 100, 0.21);
}
TEST_F(curvilinear_cosy_closed, tangent_creation)
{
  auto seg1 = cosy->get_segment(0);
  auto seg2 = cosy->get_segment(1);
  auto seg3 = cosy->get_segment(2);
  auto seg4 = cosy->get_segment(3);

  Eigen::Vector2d dir = Eigen::Vector2d(2, 1).normalized();
  ASSERT_FLOAT_EQ(seg1.t_1()[0], -dir[0]);
  ASSERT_FLOAT_EQ(seg1.t_1()[1], dir[1]);
  ASSERT_FLOAT_EQ(seg1.t_2()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg1.t_2()[1], dir[1]);

  ASSERT_FLOAT_EQ(seg2.t_1()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg2.t_1()[1], dir[1]);
  ASSERT_FLOAT_EQ(seg2.t_2()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg2.t_2()[1], -dir[1]);

  ASSERT_FLOAT_EQ(seg3.t_1()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg3.t_1()[1], -dir[1]);
  ASSERT_FLOAT_EQ(seg3.t_2()[0], -dir[0]);
  ASSERT_FLOAT_EQ(seg3.t_2()[1], -dir[1]);

  ASSERT_FLOAT_EQ(seg4.t_1()[0], -dir[0]);
  ASSERT_FLOAT_EQ(seg4.t_1()[1], -dir[1]);
  ASSERT_FLOAT_EQ(seg4.t_2()[0], -dir[0]);
  ASSERT_FLOAT_EQ(seg4.t_2()[1], dir[1]);
}
TEST_F(curvilinear_cosy_open, tangent_creation)
{
  auto seg1 = cosy->get_segment(0);
  auto seg2 = cosy->get_segment(1);
  auto seg3 = cosy->get_segment(2);

  Eigen::Vector2d dir = Eigen::Vector2d(2, 1).normalized();
  ASSERT_FLOAT_EQ(seg1.t_1()[0], 0);
  ASSERT_FLOAT_EQ(seg1.t_1()[1], 1);
  ASSERT_FLOAT_EQ(seg1.t_2()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg1.t_2()[1], dir[1]);

  ASSERT_FLOAT_EQ(seg2.t_1()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg2.t_1()[1], dir[1]);
  ASSERT_FLOAT_EQ(seg2.t_2()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg2.t_2()[1], -dir[1]);

  ASSERT_FLOAT_EQ(seg3.t_1()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg3.t_1()[1], -dir[1]);
  ASSERT_FLOAT_EQ(seg3.t_2()[0], 0);
  ASSERT_FLOAT_EQ(seg3.t_2()[1], -1);
}
TEST(curvilinear_cosy, set_tangent_closed)
{
  Eigen::Vector2d dir_in = Eigen::Vector2d(3, 1);
  Eigen::Vector2d dir = dir_in.normalized();
  Eigen::VectorXd x(5), y(5), z(5), tx(5), ty(5), tz(5);
  x << -1, -1, 1, 1, -1;
  y << 0, 1, 1, 0, 0;
  z << 0, 0, 0, 0, 0;
  tx << -dir_in[0], dir_in[0], dir_in[0], -dir_in[0], -dir_in[0];
  ty << dir_in[1], dir_in[1], -dir_in[1], -dir_in[1], dir_in[1];
  tz << 0, 0, 0, 0, 0;
  auto cosy =
    tam::helpers::cosy::CurvilinearCosy::create(x, y, z)->set_tangent(tx, ty, tz)->build();

  auto seg1 = cosy->get_segment(0);
  auto seg2 = cosy->get_segment(1);
  auto seg3 = cosy->get_segment(2);
  auto seg4 = cosy->get_segment(3);
  ASSERT_FLOAT_EQ(seg1.t_1()[0], -dir[0]);
  ASSERT_FLOAT_EQ(seg1.t_1()[1], dir[1]);
  ASSERT_FLOAT_EQ(seg1.t_2()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg1.t_2()[1], dir[1]);

  ASSERT_FLOAT_EQ(seg2.t_1()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg2.t_1()[1], dir[1]);
  ASSERT_FLOAT_EQ(seg2.t_2()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg2.t_2()[1], -dir[1]);

  ASSERT_FLOAT_EQ(seg3.t_1()[0], dir[0]);
  ASSERT_FLOAT_EQ(seg3.t_1()[1], -dir[1]);
  ASSERT_FLOAT_EQ(seg3.t_2()[0], -dir[0]);
  ASSERT_FLOAT_EQ(seg3.t_2()[1], -dir[1]);

  ASSERT_FLOAT_EQ(seg4.t_1()[0], -dir[0]);
  ASSERT_FLOAT_EQ(seg4.t_1()[1], -dir[1]);
  ASSERT_FLOAT_EQ(seg4.t_2()[0], -dir[0]);
  ASSERT_FLOAT_EQ(seg4.t_2()[1], dir[1]);
}
TEST_F(curvilinear_cosy_open_straight, convert_to_cartesian)
{
  Eigen::Vector2d pos1 = cosy->convert_to_cartesian(0, 0);
  ASSERT_FLOAT_EQ(pos1[0], 1);  // cosy starts at x=1
  ASSERT_FLOAT_EQ(pos1[1], 0);
  Eigen::Vector2d pos2 = cosy->convert_to_cartesian(0.5, 0.7);
  ASSERT_FLOAT_EQ(pos2[0], 1.5);  // cosy starts at x=1
  ASSERT_FLOAT_EQ(pos2[1], 0.7);
  Eigen::Vector2d pos3 = cosy->convert_to_cartesian(3, 0.7);
  ASSERT_FLOAT_EQ(pos3[0], 4);  // cosy starts at x=1
  ASSERT_FLOAT_EQ(pos3[1], 0.7);
  // Extrapolate
  Eigen::Vector2d pos4 = cosy->convert_to_cartesian(-0.5, 0.7);
  ASSERT_FLOAT_EQ(pos4[0], 0.5);  // cosy starts at x=1
  ASSERT_FLOAT_EQ(pos4[1], 0.7);
  Eigen::Vector2d pos5 = cosy->convert_to_cartesian(cosy->get_ref_line_length() + 1.5, -0.7);
  ASSERT_FLOAT_EQ(pos5[0], 5.5);  // cosy starts at x=1
  ASSERT_FLOAT_EQ(pos5[1], -0.7);
}
TEST_F(curvilinear_cosy_open, heading)
{
  using tam::helpers::geometry::calc_heading;
  using tam::helpers::geometry::to_deg;
  using tam::helpers::geometry::to_rad;
  // to cartesian
  ASSERT_FLOAT_EQ(cosy->convert_to_cartesian(0, 0, to_deg(0))[2], to_rad(90));
  ASSERT_FLOAT_EQ(to_deg(cosy->convert_to_cartesian(0, 0, to_rad(45))[2]), 135);
  ASSERT_FLOAT_EQ(
    cosy->convert_to_cartesian(1, 0, to_rad(0))[2], calc_heading(Eigen::Vector2d(2, 1)));
  ASSERT_FLOAT_EQ(
    cosy->convert_to_cartesian(1, 0, to_rad(-10))[2],
    calc_heading(Eigen::Vector2d(2, 1)) - to_rad(10));
  ASSERT_FLOAT_EQ(cosy->convert_to_cartesian(-1, 0, to_rad(0))[2], to_rad(90));
  ASSERT_FLOAT_EQ(cosy->convert_to_cartesian(5, 0, to_rad(0))[2], to_rad(-90));

  // to sn
  ASSERT_FLOAT_EQ(cosy->convert_to_sn(0, 2, to_rad(-45))[2], to_rad(-45));
  ASSERT_FLOAT_EQ(cosy->convert_to_sn(0, 2, to_rad(20))[2], to_rad(20));
  ASSERT_FLOAT_EQ(cosy->convert_to_sn(0, 2, to_rad(-100))[2], to_rad(-100));
  ASSERT_FLOAT_EQ(
    to_deg(cosy->convert_to_sn(1, 1, to_rad(0))[2]), to_deg(calc_heading(Eigen::Vector2d(2, 1))));
}
TEST_F(curvilinear_cosy_open, return_index)
{
  auto ret = cosy->convert_to_sn_and_get_idx(0.0, 1.0);
  ASSERT_FLOAT_EQ(std::get<1>(ret), 1.5);
  //
  auto ret2 = cosy->convert_to_sn_and_get_idx(2, 0.3);
  ASSERT_FLOAT_EQ(std::get<1>(ret2), 2.9);
  //
  auto ret3 = cosy->convert_to_sn_and_get_idx(-2, -1);
  ASSERT_FLOAT_EQ(std::get<1>(ret3), -1);
  ASSERT_FLOAT_EQ(std::get<0>(ret3)[0], -1);
  ASSERT_FLOAT_EQ(std::get<0>(ret3)[1], 1);
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
