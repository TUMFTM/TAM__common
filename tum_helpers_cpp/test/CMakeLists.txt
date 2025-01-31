find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
add_executable(test_main test_main.cpp)
target_compile_features(test_main PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_main matplotlib_cpp)
target_link_libraries(test_main ${PROJECT_NAME})

add_executable(rotations_test rotations_test.cpp)
target_compile_features(rotations_test PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(rotations_test matplotlib_cpp)
target_link_libraries(rotations_test ${PROJECT_NAME})

install(
  DIRECTORY ../include/
  DESTINATION include
)
install(
  TARGETS test_main rotations_test
  DESTINATION lib/${PROJECT_NAME}
)