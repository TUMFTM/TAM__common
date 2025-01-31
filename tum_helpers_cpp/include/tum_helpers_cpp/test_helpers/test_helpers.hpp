// Copyright 2023 Simon Hoffmann
#include <gtest/gtest.h>

#include <eigen3/Eigen/Dense>
inline void ASSERT_EIGEN_MAT(
  const Eigen::Ref<const Eigen::MatrixXd> val1, const Eigen::Ref<const Eigen::MatrixXd> val2)
{
  ASSERT_EQ(val1.rows(), val2.rows());
  ASSERT_EQ(val1.cols(), val2.cols());
  for (int i = 0; i < val1.rows(); i++) {
    for (int j = 0; j < val1.cols(); j++) {
      ASSERT_FLOAT_EQ(val1(i, j), val2(i, j));
    }
  }
}
