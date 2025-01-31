#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "network_communication/tam_receiver.h"
#include "network_communication/udp_receiver.h"
#include "tum_msgs/msg/tum_connection_status.hpp"
int main(int argc, char ** argv)
{
  int port{8080};
  rclcpp::init(argc, argv);
  auto receiver = std::make_shared<tam::network::Receiver>();
  receiver->add_processer<nav_msgs::msg::Odometry>(
    "/receive/telemetry", std::make_unique<tam::network::UdpReceiver>(port));
  rclcpp::spin(receiver);
  rclcpp::shutdown();
  return 0;
}