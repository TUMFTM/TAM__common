add_executable(test_main test_main.cpp)
target_compile_features(test_main PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_main ${PROJECT_NAME})


find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
add_executable(test_plot test_plot.cpp)
target_compile_features(test_plot PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(test_plot matplotlib_cpp)
target_link_libraries(test_plot ${PROJECT_NAME})

install(
  DIRECTORY tracks
  DESTINATION share/${PROJECT_NAME}/test_data
)
install(
  TARGETS test_plot test_main
  DESTINATION lib/${PROJECT_NAME}
)