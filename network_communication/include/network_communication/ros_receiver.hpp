// Copyright 2024 Simon Hoffmann
#pragma once
#include <network_communication/helpers.h>
#include <network_communication/udp_receiver.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using std::placeholders::_1;
namespace tam::network
{
class RosReceiver
{
private:
  struct Receiver
  {
    ~Receiver()
    {
      if (thread->joinable()) {
        thread->join();
      }
    }
    rclcpp::PublisherBase::SharedPtr ros_publisher;
    std::unique_ptr<tam::network::BaseReceiver> net_receiver{nullptr};
    std::unique_ptr<std::thread> thread{nullptr};
    bool first{true};
    std::string topic;
  };

public:
  template <typename T>
  void add_receiver(
    rclcpp::Node * node, std::unique_ptr<tam::network::BaseReceiver> && protocol_strategy,
    const std::string & topic)
  {
    auto receiver = udp_receiver_list_.emplace_back(std::make_shared<Receiver>());
    receiver->net_receiver = std::move(protocol_strategy);
    receiver->ros_publisher = node->create_publisher<T>(topic, 1);
    receiver->thread =
      std::make_unique<std::thread>(&RosReceiver::net_msg_received<T>, this, receiver);
    receiver->topic = topic;
  }

private:
  std::vector<std::shared_ptr<Receiver>> udp_receiver_list_;
  template <typename T>
  void net_msg_received(std::shared_ptr<Receiver> receiver)
  {
    typename rclcpp::Publisher<T>::SharedPtr typed_ros2_pub;
    typed_ros2_pub =
      std::dynamic_pointer_cast<typename rclcpp::Publisher<T>>(receiver->ros_publisher);

    receiver->net_receiver->waiting_for_client_connect();
    while (rclcpp::ok()) {
      std::vector<uint8_t> data = receiver->net_receiver->receive();
      if (auto msg = tam::network::helpers::deserialize_ros_msg<T>(data)) {
        if (receiver->first) {
          std::cout << "First time receiving topic " << receiver->topic << " via Network \n";
          receiver->first = false;
        }
        typed_ros2_pub->publish(msg.value());
      }
    }
    receiver->net_receiver->disconnect();
  }
};
}  // namespace tam::network