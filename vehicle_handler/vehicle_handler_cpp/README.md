<div align="center">
  <h1>Vehicle Handler</h1>
TUM Autonomous Motorsport package for modular handling of vehicle specific parameters and functions
</div>

<h2>Setup</h2>
<h3>Add vehicle_handler_cpp to your Package</h3>

- CMakeLists.txt

```CMAKE
find_package(vehicle_handler_cpp REQUIRED)
ament_target_dependencies(${YOUR_TARGET} PUBLIC vehicle_handler_cpp)
```

- package.xml

```xml
<depend>vehicle_handler_cpp</depend>
```
<h3>Create config overwrite in tam_launch</h3>

Default parameters for different vehicles are stored in [config_overwrite/](config_overwrite/) and should be provided in the shown format.
To overwrite the track configuration inside the container (e.g. from tam_launch), mount a local folder to the [config_overwrite/](config_overwrite/) folder inside the container:

```yaml
YOUR_MODULE:
  image: gitlab.lrz.de:5005/tam/<YOUR_MODULE>:<YOUR_TAG>
  network_mode: "host"
volumes:
    - "./<LOCAL_FOLDER>:/dev_ws/install/vehicle_handler_cpp/share/vehicle_handler_cpp/config_overwrite
```

For **tam_launch**:
```yaml
YOUR_MODULE:
  image: gitlab.lrz.de:5005/tam/<YOUR_MODULE>:<YOUR_TAG>
  network_mode: "host"
volumes:
    - "./config/vehicle_handler:/dev_ws/install/vehicle_handler_cpp/share/vehicle_handler_cpp/config_overwrite
```

<h2>Usage</h2>

<h3>Create the Vehicle</h3>

- include the library in your header file:

```C++
#include "vehicle_handler_cpp/vehicle_handler.hpp"
```

- create the vehicle by running:

```C++
std::unique_ptr<VehicleHandler> vehicle = tam::common::VehicleHandler::from_pkg_config();
```

- *NOTE*: this triggers the loading of all parameters from the config-files located in the folder [config_overwrite](config_overwrite/) for each vehicle. Therefore it is recommended to create the vehicle handler just once and store it e.g. as class variable.
- *RECOMMENDED USAGE*: set and create the vehicle handler as a class variable:

```C++
class your_module
{
public:
    your_module();
private:
    std::unique_ptr<VehicleHandler> vehicle_ = tam::common::VehicleHandler::from_pkg_config();
}
```

- see also the usage [example](example/module.cpp)
- this way, the parameters are only loaded when creating the class and you can access them from everywhere in your module/use the provided functionalities

<h3>Access Parameters</h3>

- for fast access to parameters for users and functions, the parameters are stored in structs, which are private members of the vehicle.
- Handling of the parameters is taken care of by a [param reference manager](https://gitlab.lrz.de/iac/iac_common/-/tree/develop/parameter_management/param_management_cpp?ref_type=heads)
- you can access parameters by calling the `vehicle()` or `tires()`-method on the vehicle handler instance (see further parameter option from autocompletion by typing `.`):
  - exemplary usage:

    ```C++
    const double brake_delay = vehicle_->vehicle().actuator.brake_delay;
    ```

- to transfer parameter values from the vehicle handler to your module param manager, you can use the method

    ```C++
    vehicle->load_params(tam::pmg::MgmtInterface * pmg)
    ```

  - it overwrites all existing parameters in your module that have the same string-name as in the vehicle handler
  - this method is intended to simplify the creation of modules with publication intent
  - to list all parameters registered in the vehicle handler, the function `list_parameters()` is provided
- *CAREFUL*: vehicle handler also provides a function to overwrite single parameters after its creation
  - to do so, call the following method (vehicle handler checks for param type compatibility):

    ```C++
    vehicle_->overwrite_param(const std::string & param_name, const <type> value)
    ```

  - the intent of this function is to allow the modification of parameters (for whatever reasons) but still be able to use the dynamic functions of the vehicle handler
  - Note that when using this function, the parameters of your vehicle handler are out of sync with the one of all other modules!

<h3>Dynamic Functions</h3>

- the vehicle handler provides methods using several dynamic functions from [tum_helpers_cpp](https://gitlab.lrz.de/iac/iac_common/-/tree/develop/tum_helpers_cpp/include/tum_helpers_cpp?ref_type=heads) and the parameters of the vehicle handler
- currently supported methods:

| Method    | Description | Return |
| -------- | ------- | ------ |
| `calc_aerodynamics(const tam::types::common::Vector3D<double> velocity)`  | Calculating drag-force and downforce and the resulting moments of their acting point on the COG | `tam::types::vehicle_params::AeroModelOutput` (two 3D-vectors with force on COG and moment) |
| `calc_dynamic_tire_radius(const tam::types::common::Vector3D<double> & velocity)` | Calculate dynamic tire radius | `tam::types::common::DataPerWheel<double>` |
| `calc_long_slip(const tam::types::common::Vector3D<double> & velocity, const tam::types::common::DataPerWheel<double> & wheelspeeds)` | Calculate slip-ratios | `tam::types::common::DataPerWheel<double>` |
| `calc_slip_angles(const tam::types::control::Odometry & odom, const double steering_angle)` | Calculate slip angles | `tam::types::common::DataPerWheel<double>` |

<h3>Adding a Parameter</h3>

- to add a new parameter, follow these steps:
  1. Add the parameter to the yaml config-file in [config_overwrite](config_overwrite/) in the respective category and for all vehicles
  2. Add the parameter to the corresponding struct in [tum_types_cpp](https://gitlab.lrz.de/iac/iac_common/-/tree/develop/tum_types_cpp?ref_type=heads) -> `tire.hpp` or `vehicle.hpp`
  3. *NOTE*: The category of the new parameter and its name need to be the same in the yaml file and the struct!
