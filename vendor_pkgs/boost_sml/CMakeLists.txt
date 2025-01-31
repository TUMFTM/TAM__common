cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME boost_sml)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


# Get boost sml header library via fetch content
include(FetchContent) # once in the project to include the module

set(SML_BUILD_BENCHMARKS OFF CACHE BOOL "Don't build benchmarks")
set(SML_BUILD_EXAMPLES OFF CACHE BOOL "Don't build examples")
set(SML_BUILD_TESTS OFF CACHE BOOL "Don't build tests")
FetchContent_Declare(sml
                     GIT_REPOSITORY https://github.com/boost-ext/sml.git
                     GIT_TAG        23e9a24e434cac9922039cbb43ca54ab70bef72c # release-1.1.6
                    )
FetchContent_MakeAvailable(sml)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
    INTERFACE 
        $<BUILD_INTERFACE:${sml_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} INTERFACE c_std_99 cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp)

install(
  DIRECTORY ${sml_SOURCE_DIR}/include/
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(tam_state_machine::tam_state_machine)
ament_export_dependencies(rclcpp) # other dependencies so client does not need to call find_package on them


ament_package()