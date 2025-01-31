// Copyright 2024 Simon Hoffmann
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tum_msgs/msg/tum_connection_status.hpp>
#include <network_communication/udp_receiver.h>
#include <network_communication/helpers.h>
#include <memory>
#include <optional>

namespace tam::network {

class Receiver : public rclcpp::Node{
private:
    struct RosMsgReceiver{
        rclcpp::PublisherBase::SharedPtr recvMsgPubs;
        std::unique_ptr<tam::network::BaseReceiver> Receiver{nullptr};
        bool printedInfo{false};
        std::unique_ptr<std::thread> thread{nullptr};
    };

public:
    explicit Receiver() : Node("test_receiver") {}
    ~Receiver() {
        for(auto& receiver: _rosMsgReceivers) {
            if(receiver->thread->joinable())
                receiver->thread->join();
        }
    }

    template <typename T>
    void add_processer(const std::string &topic, std::unique_ptr<BaseReceiver>&& protocol_strategy) {
        auto receiver = _rosMsgReceivers.emplace_back(std::make_shared<RosMsgReceiver>());
        receiver->Receiver = std::move(protocol_strategy);
        receiver->recvMsgPubs = this->create_publisher<T>(topic, 1);
        receiver->thread = std::make_unique<std::thread>(&Receiver::receive_msgs<T>, this, receiver);
    }

private:
    std::vector<std::shared_ptr<RosMsgReceiver>> _rosMsgReceivers;

    template <typename T>
    void receive_msgs(std::shared_ptr<RosMsgReceiver> receiver) {
        typename rclcpp::Publisher<T>::SharedPtr typed_ros2_pub;
        typed_ros2_pub = std::dynamic_pointer_cast<typename rclcpp::Publisher<T>>(
            receiver->recvMsgPubs);

        receiver->Receiver->waiting_for_client_connect();
        while (rclcpp::ok()) {
            std::vector<uint8_t> data = receiver->Receiver->receive();
            if(auto msg = tam::network::helpers::deserialize_ros_msg<T>(data)) {
                typed_ros2_pub->publish(msg.value());
                if (!receiver->printedInfo) {
                    receiver->printedInfo = true;
                    RCLCPP_INFO(this->get_logger(), "%s: receiving topic %s",
                        this->get_name(), receiver->recvMsgPubs->get_topic_name());
                }
            } 
        }
        receiver->Receiver->disconnect();
    }
};
};