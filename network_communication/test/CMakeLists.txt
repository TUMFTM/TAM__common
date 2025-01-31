find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

add_executable(udpsender "samples/udp_sender_receiver/udp_test_sender.cpp")
target_link_libraries(udpsender ${PROJECT_NAME})
ament_target_dependencies(udpsender rclcpp tum_msgs nav_msgs)

add_executable(udpreceiver "samples/udp_sender_receiver/udp_test_receiver.cpp")
target_link_libraries(udpreceiver ${PROJECT_NAME})
ament_target_dependencies(udpreceiver rclcpp tum_msgs nav_msgs)

add_executable(min_status_publisher "samples/tam_sender_receiver/minimal_status_publisher.cpp")
ament_target_dependencies(min_status_publisher rclcpp tum_msgs nav_msgs)

add_executable(tamsender "samples/tam_sender_receiver/tam_test_sender.cpp")
target_link_libraries(tamsender ${PROJECT_NAME})
ament_target_dependencies(tamsender rclcpp tum_msgs nav_msgs)

add_executable(tamreceiver "samples/tam_sender_receiver/tam_test_receiver.cpp")
target_link_libraries(tamreceiver ${PROJECT_NAME})
ament_target_dependencies(tamreceiver tum_msgs nav_msgs)

install(TARGETS
    udpsender
    udpreceiver
    min_status_publisher
    tamsender
    tamreceiver
    DESTINATION lib/${PROJECT_NAME}
)
