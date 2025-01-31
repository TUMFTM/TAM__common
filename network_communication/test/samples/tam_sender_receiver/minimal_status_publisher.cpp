// Copyright 2025 Simon Hoffmann
#include <chrono>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tum_msgs/msg/tum_connection_status.hpp>
using namespace std::chrono_literals;
class MinimalStatusPublisher : public rclcpp::Node
{
public:
  MinimalStatusPublisher() : Node("minimal_status_publisher")
  {
    publisher_ =
      this->create_publisher<tum_msgs::msg::TUMConnectionStatus>("/connection_status", 10);
    prm_cmd_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/telemetry", 10);
    timer_ =
      this->create_wall_timer(500ms, std::bind(&MinimalStatusPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = tum_msgs::msg::TUMConnectionStatus();
    message.connection_status = tum_msgs::msg::TUMConnectionStatus::CONNECTED;
    message.ip_addresses = {"127.0.0.1"};
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.connection_status);
    publisher_->publish(message);

    auto prm_cmd = nav_msgs::msg::Odometry();
    prm_cmd_publisher_->publish(prm_cmd);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tum_msgs::msg::TUMConnectionStatus>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr prm_cmd_publisher_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}
