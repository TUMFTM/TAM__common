#pragma once

#include <memory>
#include <string>
#include <thread>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
namespace tam::core
{
/**
 * Class that will handle connection to a CAN Bus
 *
 * It will also create a backup ROS pub/sub, which is useful for replaying rosbags
 *
 * This class will declare three ROS parameters:
 * 1. interface <std::string> "can0", this parameter determines the CAN interface to use
 * 2. nocan <bool> "false", this parameter determines wheter to use a CAN interface or only ROS
 * 3. use_bus_time <bool> "false", this parameter determines, which timestamp to use for the CAN
 * frame
 */
class CanComms
{
public:
  /**
   * Contructor for the CAN Communication method
   *
   * @param node Pointer to the parent node
   * @param can_callback Function that handles received CAN frames
   * @param interface Name of the CAN interface
   * @param use_can Use CAN, disable if no CAN is available
   * @param use_ros Use ROS, disable for performance
   * @param use_bus_time Use timestamp from Bus, otherwise from ROS node
   * 
   * CanComms need either CAN or ROS to be enabled to work the constructor will throw
   * an error if neither is enabled
   */
  CanComms(rclcpp::Node * node,
    std::function<void(can_msgs::msg::Frame::SharedPtr)> can_callback,
    std::string interface,
    bool use_can = true,
    bool use_ros = false,
    bool use_bus_time = false);

private:
  rclcpp::Node * node;
  std::function<void(can_msgs::msg::Frame::SharedPtr)> can_callback;

  std::unique_ptr<drivers::socketcan::SocketCanSender> can_sender;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> can_receiver;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr backup_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr backup_publisher_;

  void receive_can_direct();

  /**
   * Variable to determine which timestamp is set for the ROS CAN frames
   *
   * Using the CAN Bus time or the time, when the CAN Frame was received by the ROS subscription
   */
  const bool use_bus_time_ = false;

  /**
   * Receiver to handle incoming CAN frames from a real/virtual CAN Bus
   *
   * This means that this node will spawn a second thread and therefore introduce parallelism
   */
  std::unique_ptr<std::thread> receiver_thread_;

  /**
   * Variable to determine if a real CAN Bus should be used
   *
   * In some cases we would like to keep the same code, but only use ROS
   * because no CAN is physically/virtually present
   * 
   * One the applications would be the tam/dSpace sim environment, where the actual CAN
   * communication is modeled, but only via ROS and we do not have access to the hardware 
   * to add a virtual CAN Bus
   * 
   * It is not intended to run our software locally with a virtual CAN Bus, since the whole
   * point is to contantly test the real functionality if possible
   */
  const bool use_can_{false};

  /**
   * Variable to determine if ROS topic should be published
   * 
   * In some cases we would like to not publish to ROS as it would overwhelm the system
   * and a candump is created as a backup anyways
  */
  const bool use_ros_{false};

public:
  /**
   * Method for sending CAN frames
   *
   * @param can_msg Pointer to the ROS CAN frame
   *
   * This method will send the CAN frame to a real CAN interface if the "nocan"
   * parameter is not set, but it will always send it to the backup ROS publisher
   */
  void send_can_direct(const can_msgs::msg::Frame::SharedPtr can_msg);

  using SharedPtr = std::shared_ptr<tam::core::CanComms>;
  using UniquePtr = std::unique_ptr<tam::core::CanComms>;
};
}  // namespace tam::core
