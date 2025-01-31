# TUM Autonomous Motorsport: Common Library

This repository contains a collection of common functionality used in various modules of the software stack of TUM Autonomous Motorsport.

## Repository Content

This repository contains several packages that can be built. Below is a list of the available packages along with a brief description of their contents.

| Package Name        | Description                                                                 |
|---------------------|-----------------------------------------------------------------------------|
| [`can_comms_cpp`](./can_comms_cpp/README.md) | C++ library for CAN communication |
| [`limit_handler_cpp`](./limit_handler_cpp/README.md) | C++ library for handling acceleration limits |
| [`limit_handler_py`](./limit_handler_cpp/README.md) | Python binding for limit_handler_cpp |
| [`network_communication`](./network_communication/README.md) | UDP/ROS Bride for communication between Non-ROS and ROS Modules via given msg Definitions |
| [`ros2_watchdog_cpp`](./ros2_watchdog_cpp/README.md) | C++ library for monitoring ROS2 topics and nodes |
| [`ros2_watchdog_py`](./ros2_watchdog_cpp/README.md) | Python binding for ros2_watchdog_cpp |
| [`track_handler_cpp`](./track_handler_cpp/README.md) | C++ library for handling and processing race track files |
| [`track_handler_py`](./track_handler_cpp/README.md) | Python binding for track_handler_cpp |
| [`tum_helpers_cpp`](./tum_helpers_cpp/README.md) | Various helper and conversion functions for C++ applications |
| [`tum_helpers_py`](./tum_helpers_cpp/README.md) | Python binding of tum_helpers_cpp |
| [`tum_ros_helpers_cpp`](./tum_ros_helpers_cpp/README.md) | C++ library for providing helper functions for ROS2 applications |
| [`tum_type_conversions_ros_cpp`](./tum_type_conversions_ros_cpp/README.md) | Various message type conversions for C++ applications |
| [`tum_type_conversions_ros_py`](./tum_type_conversions_ros_cpp/README.md) | Python binding for tum_type_conversions_ros_cpp |
| [`tum_types_cpp`](./tum_types_cpp/README.md) | C++ library containing common struct definitions for data handling |
| [`tum_types_py`](./tum_types_cpp/README.md) | Python binding for tum_types_cpp |
| [`vehicle_handler_cpp`](./vehicle_handler_cpp/README.md) | C++ library for handling vehicle parameter and vehcile specific calculations |
| [`vehicle_handler_py`](./vehicle_handler_cpp/README.md) | Python binding for vehicle_handler_cpp |
| [`boost_sml`](./vendor_pkgs/boost_sml/README.md) | Vendor Package for the Open Source State Machine Library for C++ |
| [`matplotlib_cpp`](./vendor_pkgs/matplotlib_cpp/README.md) | Vendor Package for the Open Source Plotting library for C++ applications |

### Externally Included Repositories

| Package Name        | Description                                                                 |
|---------------------|-----------------------------------------------------------------------------|
| [`parameter_management`](https://github.com/TUMFTM/TAM__param_management/tree/main) | Parameter Management Library of TUM Autonmous Motorsport |
| [`time_series_logging`](https://github.com/TUMFTM/tsl) | The time series logging library of TUM Autonomous Motorsport |
| [`boost_sml`](https://github.com/boost-ext/sml) | State Machine Library for C++ |
| [`matplotlib_cpp`](https://github.com/lava/matplotlib-cpp) | Plotting functionality for C++ applications |


## Building the Project

### Installing the required dependencies

*Attention: The repository contains submodules so please clone with the `--recursive` option*

We are using Ubuntu 22.04 and ROS 2 Humble for building the project. 
We expect later releases of Ubuntu and ROS 2 to work as well, but we haven't tested it so far.

Additionally, the following packages have to be installed:

```bash
sudo apt install libboost-dev
sudo apt install ros-${ROS_DISTRO}-can-msgs
sudo apt install ros-${ROS_DISTRO}-ros2-socketcan 
sudo apt install ros-${ROS_DISTRO}-geographic-msgs # Required for autoware msgs
```

### Compiling the library.

After installing the required dependencies, all packages should compile using colcon.

```bash
colcon build
```

### Disclaimer

At this point, we cannot guarantee API stability since we are continuously developing and improving our software stack.
While we always try to avoid these, breaking changes could happen in future updates of the message definitions.


## References
### Curvilinear Cosy
Implementation of [segment.cpp](tum_helpers_cpp/src/segment.cpp), [segment.hpp](tum_helpers_cpp/include/tum_helpers_cpp/coordinate_system/segment.hpp), [curvilinear_cosy.cpp](tum_helpers_cpp/src/curvilinear_cosy.cpp), [curvilinear_cosy.hpp](tum_helpers_cpp/include/tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp) adapted from:
- https://ieeexplore.ieee.org/abstract/document/6856487
- https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker/-/tree/master/cpp/geometry
- see [License](tum_helpers_cpp/src/LICENSE)


### Contributors

The main contributors of the Project are:
- [Simon Sagmeister](https://github.com/simonsag96)
- [Simon Hoffmann](https://github.com/simonh92)
- [Phillip Pitschi](https://github.com/PhillPi) - Mainly contributing to the message conversion library
- [Maximilian Leitenstern](https://github.com/mleitenstern) - Mainly developing and implementing the vehicle handler
- [Dominic Ebner](https://github.com/Dekadee)
- [Daniel Esser](https://github.com/DaniEsser)
- [Marcel Weinmann](https://github.com/MarcelWeinmann)

We also want to thank all other members of TUM Autonomous Motorsport for actively developing this library.
