// Copyright 2024 Simon Hoffmann
#pragma once
#include <network_communication/helpers.h>
#include <network_communication/udp_sender.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using std::placeholders::_1;
namespace tam::network
{
class RosSender
{
private:
  struct Sender
  {
    rclcpp::SubscriptionBase::SharedPtr ros_subscriber;
    std::unique_ptr<tam::network::BaseSender> net_sender{nullptr};
    bool first{true};
    std::string topic;
  };

public:
  template <typename T>
  void add_sender(
    rclcpp::Node * node, std::unique_ptr<tam::network::BaseSender> && protocol_strategy,
    const std::string & topic)
  {
    std::shared_ptr<Sender> sender = udp_sender_list_.emplace_back(std::make_shared<Sender>());
    sender->net_sender = std::move(protocol_strategy);
    std::function<void(const std::shared_ptr<T>)> fnc =
      std::bind(&RosSender::ros_msg_received<T>, this, _1, sender);
    sender->ros_subscriber = node->create_subscription<T>(topic, 10, fnc);
    sender->topic = topic;
  }

private:
  std::vector<std::shared_ptr<Sender>> udp_sender_list_;
  template <typename T>
  void ros_msg_received(const std::shared_ptr<T> msg, std::shared_ptr<Sender> sender)
  {
    if (sender->first) {
      std::cout << "First time sending topic " << sender->topic
                << " to :" << sender->net_sender->get_destination_ip() << "\n";
      sender->first = false;
    }
    std::vector<uint8_t> vec = tam::network::helpers::serialize_ros_msg<T>(*msg);
    sender->net_sender->send_data(vec);
  }
};
}  // namespace tam::network