cmake_minimum_required(VERSION 3.14)
set(PROJECT_NAME vehicle_handler_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(param_management_cpp REQUIRED)
find_package(tum_types_cpp REQUIRED)
find_package(tum_helpers_cpp REQUIRED)

# Get boost pfr header library via fetch content
include(FetchContent) # once in the project to include the module

FetchContent_Declare(boost_pfr
                     GIT_REPOSITORY https://github.com/boostorg/pfr.git
                     GIT_TAG        294a4976bd04829dd204aaf9e9fd30338a5d3199 # release-2.2.0
                    )
FetchContent_MakeAvailable(boost_pfr)

add_library(${PROJECT_NAME} SHARED
  src/vehicle_handler.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS};${boost_pfr_SOURCE_DIR}/include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ament_index_cpp
  param_management_cpp
  tum_types_cpp
  tum_helpers_cpp)
target_link_libraries(${PROJECT_NAME} 
  PRIVATE ${YAML_CPP_LIBRARIES}
  PRIVATE ${Boost_LIBRARIES})

install(
  DIRECTORY config_overwrite
  DESTINATION share/${PROJECT_NAME}
)
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(
  ament_index_cpp
  param_management_cpp
  tum_types_cpp
  tum_helpers_cpp
) # other dependencies so client does not need to call find_package on them

ament_package()

# ==============================================================================
# BUILD EXAMPLES 
# ==============================================================================

# Create the executable
add_executable(module example/module.cpp)
# Link against the lib and private dependencies
target_link_libraries(module ${PROJECT_NAME}) 
# Install target
install(
  TARGETS module  
  DESTINATION example/${PROJECT_NAME}
)

