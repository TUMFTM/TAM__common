# CAN Communication Package

This package contains a class that facilitates communication via CAN.

The motivation for this package is the removal of a the CAN publisher and subscriber that was used beforehand, therefore reducing latency and reducing CPU load by saving this part of the communication chain.

Heavily based on the node implementation of [ROS2 socketcan by the Autoware Foundation](https://github.com/autowarefoundation/ros2_socketcan).

The library provides a class for CAN communication with three important things:

- Easy callback definition
- A member function called `send_can_direct`, which accepts a `can_msgs::msg::Frame::SharedPtr` and sends it to CAN
- Simple options for debugging and using ROS again

## Setup VCAN

Unfortunately this class will always try to open a CAN interface on startup and crash otherwise. To run the node anyways, create a virtual CAN interface like this:

- `sudo modprobe vcan`
- `sudo ip link add dev vcan0 type vcan`
- `sudo ip link set up vcan0`
- `sudo ip l property add dev vcan0 altname can0`
- `sudo ip l property add dev vcan0 altname can1`
- `sudo ip l property add dev vcan0 altname can2`

This will create a single virtual CAN bus, that can be addressed by an of the four names.

## Usage

```cpp
// Create comms in constructor of node
auto can_comms = std::make_shared<tam::core::CanComms>(
      this,
      std::bind(
        &can_msgs_handler_func,
        this,
        std::placeholders::_1
    ),
    "vcan0", // Interface name
    true, // Use CAN
    false, // Use ROS
    false // Use bus time
);

// Callback function
void can_msg_handler_func(can_msgs::msg::Frame::SharedPtr in_can_frame)
{
    if (!in_can_frame->is_rtr && !in_can_frame->is_error)
    {
        switch (in_can_frame->id)
        {
            // Handle message here
        }
    }
}
```

## Reference implementations

This was created for DBW interfaces, which is why it is used there mostly. Take a look at [`mod_vehicle_interface`](https://gitlab.lrz.de/iac/mod_vehicle_interface/-/tree/develop/meccanica_interface?ref_type=heads) for a reference implementation and optimal usage. Currently this is also used for the Kistler driver in [`iac_drivers`](https://gitlab.lrz.de/iac/iac_drivers).