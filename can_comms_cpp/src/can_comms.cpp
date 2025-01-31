#include "can_comms_cpp/can_comms.hpp"

using namespace std::chrono;
tam::core::CanComms::CanComms(
  rclcpp::Node * node,
  std::function<void(can_msgs::msg::Frame::SharedPtr)> can_callback,
  std::string interface,
  bool use_can,
  bool use_ros,
  bool use_bus_time)
: node(node), can_callback(can_callback), use_bus_time_{use_bus_time},
  use_can_{use_can}, use_ros_{use_ros}
{
  if(!use_can && !use_ros)
  {
    std::cerr << "Cannot disable both ROS and CAN at the same time!" << std::endl;
    throw std::runtime_error("no_comm_impossible");
  }

  can_sender = std::make_unique<drivers::socketcan::SocketCanSender>(interface);
  can_receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>(interface);

  if(use_ros)
  {
    backup_subscription_ = node->create_subscription<can_msgs::msg::Frame>(
      "/" + interface + "/compat/from_can_bus", 20, can_callback);

    backup_publisher_ = node->create_publisher<can_msgs::msg::Frame>(
      "/" + interface + "/compat/to_can_bus", 20);
  }

  if (use_can) {
    receiver_thread_ = std::make_unique<std::thread>(&CanComms::receive_can_direct, this);
  }
}
void tam::core::CanComms::send_can_direct(can_msgs::msg::Frame::SharedPtr can_msg)
{
  if (use_can_) {
    drivers::socketcan::FrameType type;
    if (can_msg->is_rtr) {
      type = drivers::socketcan::FrameType::REMOTE;
    } else if (can_msg->is_error) {
      type = drivers::socketcan::FrameType::ERROR;
    } else {
      type = drivers::socketcan::FrameType::DATA;
    }

    drivers::socketcan::CanId send_id =
      can_msg->is_extended
        ? drivers::socketcan::CanId(can_msg->id, 0, type, drivers::socketcan::ExtendedFrame)
        : drivers::socketcan::CanId(can_msg->id, 0, type, drivers::socketcan::StandardFrame);
    try {
      can_sender->send(can_msg->data.data(), can_msg->dlc, send_id, 10ms);
    } catch (const std::exception & ex) {
      std::cerr << "Error sending CAN Frame: " << ex.what() << std::endl;
      return;
    }
  }
  if(use_ros_)
  {
    this->backup_publisher_->publish(*can_msg);
  }
}
void tam::core::CanComms::receive_can_direct()
{
  drivers::socketcan::CanId receive_id{};
  can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
  frame_msg.header.frame_id = "can";

  while (1) {
    try {
      receive_id = can_receiver->receive(frame_msg.data.data(), 10ms);
    } catch (const std::exception & ex) {
      std::cerr << "Error receiveing CAN Frame: " << ex.what() << std::endl;
      continue;
    }

    if (this->use_bus_time_) {
      frame_msg.header.stamp =
        rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
    } else {
      frame_msg.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()
      ).count());
    }
    frame_msg.id = receive_id.identifier();
    frame_msg.is_rtr = (receive_id.frame_type() == drivers::socketcan::FrameType::REMOTE);
    frame_msg.is_extended = receive_id.is_extended();
    frame_msg.is_error = (receive_id.frame_type() == drivers::socketcan::FrameType::ERROR);
    frame_msg.dlc = receive_id.length();
    can_callback(std::make_shared<can_msgs::msg::Frame>(frame_msg));
  }
}
