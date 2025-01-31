// Copyright 2023 Simon Hoffmann
#include <gtest/gtest.h>

#include <vector>

#include "track_handler_cpp/track_helpers.hpp"
TEST(geometry_helpers, s_coordinate)
{
  ASSERT_TRUE(!tam::helpers::track::ahead_of_ref(100, 50, 1000));
  ASSERT_TRUE(tam::helpers::track::ahead_of_ref(100, 150, 1000));
  ASSERT_TRUE(!tam::helpers::track::ahead_of_ref(100, 990, 1000));

  ASSERT_TRUE(tam::helpers::track::ahead_of_ref(10, 50, 1000));
  ASSERT_TRUE(tam::helpers::track::ahead_of_ref(900, 50, 1000));
  ASSERT_TRUE(!tam::helpers::track::ahead_of_ref(500, 50, 1000));
  ASSERT_TRUE(!tam::helpers::track::ahead_of_ref(499, 50, 1000));
  ASSERT_TRUE(tam::helpers::track::ahead_of_ref(550, 50, 1000));
  ASSERT_TRUE(tam::helpers::track::ahead_of_ref(49, 50, 1000));
  ASSERT_TRUE(!tam::helpers::track::ahead_of_ref(51, 50, 1000));
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
