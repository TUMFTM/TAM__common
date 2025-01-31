// Copyright 2023 Simon Hoffmann
#include <gtest/gtest.h>

#include <vector>

#include "tum_helpers_cpp/numerical.hpp"
TEST(numerical, gradient)
{
  using tam::helpers::numerical::gradient;
  Eigen::VectorXd f(6), x(6);
  f << 1, 2, 4, 7, 11, 16;
  x << 0, 2, 4, 6, 8, 10;

  Eigen::VectorXd df = gradient(f, x);
  ASSERT_FLOAT_EQ(df[0], 0.5);
  ASSERT_FLOAT_EQ(df[1], 0.75);
  ASSERT_FLOAT_EQ(df[2], 1.25);
  ASSERT_FLOAT_EQ(df[3], 1.75);
  ASSERT_FLOAT_EQ(df[4], 2.25);
  ASSERT_FLOAT_EQ(df[5], 2.5);
}
TEST(numerical, find_idx_bottom)
{
  using tam::helpers::numerical::find_bottom_idx;
  std::vector<double> s{0, 2, 4, 5, 6, 7, 8};

  ASSERT_EQ(find_bottom_idx(s.begin(), s.end(), 2.2), 1);
  ASSERT_EQ(find_bottom_idx(s.begin(), s.end(), 0), 0);
  ASSERT_EQ(find_bottom_idx(s.begin(), s.end(), 2), 0);
  ASSERT_EQ(find_bottom_idx(s.begin(), s.end(), 6), 3);
  ASSERT_EQ(find_bottom_idx(s.begin(), s.end(), 9), 5);
  ASSERT_EQ(find_bottom_idx(s.begin(), s.end(), -2), 0);

  Eigen::VectorXd m(7);
  m << 0, 2, 4, 5, 6, 7, 8;
  ASSERT_EQ(find_bottom_idx(m.begin(), m.end(), 2.2), 1);
}
TEST(numerical, interp)
{
  using tam::helpers::numerical::interp;
  std::vector<double> sp{0, 2, 4, 6, 8, 10, 12};
  std::vector<double> wp{1, 2, 3, 4, 5, 6, 7};
  // pass scalar and vector
  ASSERT_FLOAT_EQ(interp(3, sp, wp), 2.5);
  ASSERT_FLOAT_EQ(interp(5.5, sp, wp), 3.75);
  ASSERT_FLOAT_EQ(interp(14, sp, wp), 8);
  ASSERT_FLOAT_EQ(interp(-4, sp, wp), -1);

  Eigen::VectorXd m_sp(7), m_wp(7);
  m_sp << 0, 2, 4, 6, 8, 10, 12;
  m_wp << 1, 2, 3, 4, 5, 6, 7;

  // pass scalar and eigen
  ASSERT_EQ(interp(3, m_sp, m_wp), 2.5);

  std::vector<double> s{3, 5.5, 14, -4};
  // pass vector and eigen
  ASSERT_FLOAT_EQ(interp(s, m_sp, m_wp).at(1), 3.75);
  // // pass vector and vector
  ASSERT_FLOAT_EQ(interp(s, sp, wp).at(1), 3.75);

  Eigen::VectorXd m_s(4);
  m_s << 3, 5.5, 14, -4;
  //// pass eigen and vector
  ASSERT_FLOAT_EQ(interp(m_s, sp, wp)(1), 3.75);
  //// pass eigen and eigen
  Eigen::VectorXd res2 = interp(m_s, m_sp, m_wp);
  ASSERT_FLOAT_EQ(res2[1], 3.75);

  // Matrix Shaped
  Eigen::MatrixXd m_s22(2, 2);
  m_s22 << 3, 5.5, 14, -4;
  ASSERT_FLOAT_EQ(interp(m_s22, sp, wp)(0, 1), 3.75);
  ASSERT_EQ(m_s22.rows(), 2);
  ASSERT_EQ(m_s22.cols(), 2);
}
TEST(numerical, interp_idx)
{
  using tam::helpers::numerical::interp_from_idx;
  std::vector<double> sp{0, 2, 4, 6, 8, 10, 12};
  // pass scalar and vector
  ASSERT_FLOAT_EQ(interp_from_idx(sp, 2.5), 5.0);
  ASSERT_FLOAT_EQ(interp_from_idx(sp, 3.2), 6.4);
  // Extrapolate
  ASSERT_FLOAT_EQ(interp_from_idx(sp, 6.5), 13);
  ASSERT_FLOAT_EQ(interp_from_idx(sp, -1), -2);

  Eigen::VectorXd m_sp(7), m_wp(7);
  m_sp << 0, 2, 4, 6, 8, 10, 12;
  ASSERT_FLOAT_EQ(interp_from_idx(m_sp, 2.5), 5.0);
  ASSERT_FLOAT_EQ(interp_from_idx(m_sp, 3.2), 6.4);
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
