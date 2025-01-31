// Copyright 2023 Marcel Weinmann
#include <gtest/gtest.h>
#include <chrono>
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_helpers_cpp/delay_compensation.hpp"
#include "tum_helpers_cpp/geometry/geometry.hpp"

TEST(ExecutionUnitTests, ExecutionTime) {
    // Create an instance of tam::helpers::DelayCompensation::DelayCompensation
    tam::helpers::DelayCompensation delay_handler(100, 0.01);
    int64_t execution_time = 0;

    for (int i = 0; i < 1000; i++) {
        // start time of the function execution
        auto start = std::chrono::high_resolution_clock::now();

        // set some random values
        tam::types::control::Odometry se_odometry;
        tam::types::control::Odometry input_odometry;

        // update the state estimation deque
        delay_handler.update_vehicle_odometry(se_odometry);

        // compensate some random delay
        delay_handler.compensate(input_odometry, 0.05);

        // end time
        auto end = std::chrono::high_resolution_clock::now();

        // Calculate the execution time in microseconds
        execution_time +=
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }

    // run the test (divide by 1000 to get the mean)
    // one step souldn't take more than 0.1 microseconds
    EXPECT_LE(execution_time / 1e3, 0.1);
}

TEST(FunctionUnitTests, PositionUpdate) {
    // Create an instance of tam::helpers::DelayCompensation::DelayCompensation
    tam::helpers::DelayCompensation delay_handler(100, 0.01);
    tam::types::control::Odometry se_odometry;

    // fill the deque
    for (int i = 0; i < 100; i++) {

        se_odometry.position_m.x = i;
        se_odometry.position_m.y = i;

        // update the state estimation deque
        delay_handler.update_vehicle_odometry(se_odometry);
    }
    tam::types::control::Odometry input_odometry;
    input_odometry.position_m.x = 65.0;
    input_odometry.position_m.y = 65.0;

    // compensate a delay of 401 ms
    delay_handler.compensate(input_odometry, 0.401);

    EXPECT_FLOAT_EQ(input_odometry.position_m.x, 105.1);
    EXPECT_FLOAT_EQ(input_odometry.position_m.y, 105.1);
}

TEST(FunctionUnitTests, OrientationUpdate) {
    // Create an instance of tam::helpers::DelayCompensation::DelayCompensation
    tam::helpers::DelayCompensation delay_handler(100, 0.01);
    tam::types::control::Odometry se_odometry;

    // fill the deque
    for (int i = 0; i < 100; i++) {

        se_odometry.orientation_rad.x = tam::helpers::geometry::normalize_angle(i * 0.1);
        se_odometry.orientation_rad.y = tam::helpers::geometry::normalize_angle(i * 0.1);
        se_odometry.orientation_rad.z = tam::helpers::geometry::normalize_angle(i * 0.1);

        // update the state estimation deque
        delay_handler.update_vehicle_odometry(se_odometry);
    }
    tam::types::control::Odometry input_odometry;
    input_odometry.orientation_rad.x = 3.00;
    input_odometry.orientation_rad.y = 3.00;
    input_odometry.orientation_rad.z = 3.00;

    // compensate a delay of 401 ms
    delay_handler.compensate(input_odometry, 0.05);

    EXPECT_NEAR(input_odometry.orientation_rad.x, -2.78, 1e-2);
    EXPECT_NEAR(input_odometry.orientation_rad.y, -2.78, 1e-2);
    EXPECT_NEAR(input_odometry.orientation_rad.z, -2.78, 1e-2);
}
