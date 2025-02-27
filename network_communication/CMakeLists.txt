cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME network_communication)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tum_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(${PROJECT_NAME} STATIC 
    src/udp_sender.cpp
    src/udp_receiver.cpp
    src/base_protocol.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} rclcpp tum_msgs)

install(
  DIRECTORY include/
  DESTINATION include
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries
ament_export_dependencies(tum_msgs) # other dependencies so client does not need to call find_package on them

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()