# Network Communication Package

This package provides nodes for standardized network communication of ROS data.

The motivation for this package is the standardized setup of network interfaces and serialization for sending ROS topics via UDP streams.

Used for basestation communication, as ROS is not able to communicate remotely.

## Setup

No special setup needed.

## Usage

For a complete usage example look at the [samples](./test/samples/).

### Sender Usage

This will make a sender listen to the network status topic `/basestaiton/vehicle/connection_status` and determine with this topic where to send to data. The connection status message contains a source and multiple target addresses.

The messages on topic `/basestation/software_state` will be sent at most every 150 milliseconds via UDP. The UDP port is defined during creation once.

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto sender = std::make_shared<tam::network::Sender>(
        "/basestation/vehicle/connection_status"
    );
    sender->add_processer<diagnostic_msgs::msg::DiagnosticArray>(
        std::make_unique<tam::network::UdpSender>(tam::network::BasestationPorts::RX_DIAGNOSTICS),
        "/basestation/software_state", 150.0
    );
    rclcpp::spin(sender);
    rclcpp::shutdown();
    return 0;
}
```

### Receiver Usage

This example will make a node listen on a port for incoming UDP data and decode them as ROS messages. Notice here that the same port is used as in the sender exmaple. Received messages are published on the `/basestaiton/software_state` topic again on different device.

```cpp
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto receiver = std::make_shared<tam::network::Receiver>();
    receiver->add_processer<diagnostic_msgs::msg::DiagnosticArray>(
        "/basestation/software_state",
        std::make_unique<tam::network::UdpReceiver>(tam::network::BasestationPorts::RX_DIAGNOSTICS)
    );
    rclcpp::spin(receiver);
    rclcpp::shutdown();
    return 0;
}
```

