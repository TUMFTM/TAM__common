# Copyright 2024 Simon Sagmeister
cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================

set(PROJECT_NAME dynamic_node_composition)
set(EXECUTABLES StaticSingleThreadedExecutor StaticSingleThreadedExecutorWithParameterValidation)

set(LIB_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
)
set(EXECUTABLE_DEPENDENCIES
  param_management_ros2_integration_cpp
  vehicle_handler_cpp
)
# ============================================
# END OF CONFIGURATION

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

foreach(pkg_name ${LIB_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: BUILD LIBRARY TARGET TO IMPORT NODE CLASS WITHIN OTHER PKGS
# ====================================================================

add_library(${PROJECT_NAME} SHARED src/composition.cpp src/utils.cpp)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

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
ament_target_dependencies(${PROJECT_NAME} ${LIB_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(${LIB_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

# section: BUILD EXECUTABLES
# ====================================================================
foreach(pkg_name ${EXECUTABLE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

foreach(exec_name ${EXECUTABLES})
  add_executable(${exec_name} executables/${exec_name}.cpp)
  # add_executable(${exec_name} )

  target_link_libraries(${exec_name} ${PROJECT_NAME})
  ament_target_dependencies(${exec_name} ${EXECUTABLE_DEPENDENCIES})

  install(
    TARGETS ${exec_name}  
    # This seems to be standard install dir according to the ros docs. Imho bin would have been more intuitive but okay lets stick to the conventions.
    DESTINATION lib/${PROJECT_NAME} 

  )
endforeach()

# section: GENERAL
# ====================================================================
ament_package()