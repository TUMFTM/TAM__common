# Dynamic Node Composition 

## Lib

This package provides library functions for dynamically loading ros2 nodes, registered as rclcpp components, from the respective shared libraries.
For documenation of the functions, see the docstrings in the central headerfile [composition.hpp](./include/dynamic_node_composition/composition.hpp).

For examples on how to use the provided library function, please refer to the provided [executables](./executables).

### CLI Argument Parsing

Additionally we provide a function that parses which componenents should be loaded directly from the command line arguments. The syntax for this is as follows:
```bash
ros2 run dynamic_node_composition <executable_name> --component <pkg_name_1> <component_name_1> --component <pkg_name_2> <component_name_2> [--ros-args ... --] 
```


## Executables

Additionally to the lib, the package provides a set of executables, that directly load the components into a specific executor. 
Afterwards the executors spin() function is called in order to execute the callbacks of the loaded nodes.

Which components should be loaded can be passed via the command line to the executables. Please refer to the section [CLI Argument Parsing](#-cli-argument-parsing).

In some cases, there are also certain helper functions for validation built in. For a complete list of which executables are available, please refer to [the executables folder](./executables)


# Limitations

- Currently the parameter backend for the param manager, is not initialized. This is because this might not be possible without having parameter name conflicts, dependent on which nodes will be composed. If this is required, please contact Simon Sagmeister to find a solution