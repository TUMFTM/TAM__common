#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "network_communication/tam_sender.h"
#include "network_communication/udp_sender.h"
#include "tum_msgs/msg/tum_connection_status.hpp"
int main(int argc, char ** argv)
{
  int port{8080};
  rclcpp::init(argc, argv);
  auto sender = std::make_shared<tam::network::Sender>("/connection_status");
  sender->add_processer<nav_msgs::msg::Odometry>(
    std::make_unique<tam::network::UdpSender>(port), "/telemetry");
  rclcpp::spin(sender);
  rclcpp::shutdown();
  return 0;
}