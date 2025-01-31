cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME ros2_watchdog_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tum_helpers_cpp REQUIRED)
find_package(tum_types_cpp REQUIRED)
find_package(tum_type_conversions_ros_cpp REQUIRED)
find_package(param_management_cpp REQUIRED)
find_package(param_management_ros2_integration_cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/topic_watchdog.cpp
  src/node_monitor.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp diagnostic_msgs param_management_ros2_integration_cpp tum_types_cpp tum_helpers_cpp param_management_cpp tum_type_conversions_ros_cpp)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

add_executable(example_monitor
  src/example_monitor.cpp
)
target_include_directories(example_monitor
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(example_monitor rclcpp std_msgs diagnostic_msgs tum_helpers_cpp tum_types_cpp tum_type_conversions_ros_cpp)
target_link_libraries(example_monitor ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS example_monitor DESTINATION lib/${PROJECT_NAME}
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(diagnostic_msgs param_management_ros2_integration_cpp tum_types_cpp param_management_cpp tum_helpers_cpp tum_type_conversions_ros_cpp) # other dependencies so client does not need to call find_package on them

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(unit_tests  # unit tests
#     test/unit_test.cpp 
#   )
#   target_link_libraries(unit_tests ${PROJECT_NAME}) # make our library available to the test executable
# endif()

ament_package()