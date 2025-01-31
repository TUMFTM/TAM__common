# tum_type_conversions_ros_cpp

This library provides helper function to convert ROS types to C++ types and vice versa.

The following types can be converted into one another:

| ROS type | C++ type |
| -------- | ----------- |
| `geometry_msgs::msg::Vector3` | `tam::types::common::Vector3D<double>` |
| `geometry_msgs::msg::Point` | `tam::types::common::Vector3D<double>` |
| `tum_msgs::msg::TUMFloat64PerWheel` | `tam::types::common::DataPerWheel` |
| `unsigned char` (for `diagnostic_messages::msg::DiagnsosticStatus`) | `tam::types::ErrorLvl` |
| `geometry_msgs::msg::AccelWithCovarianceStamped` | `tam::types::control::AccelerationwithCovariances` |
| `nav_msgs::msg::Odometry` | `tam::types::control::Odometry` |
| `tier4_planning_msgs::msg::Trajectory` | `tam::types::control::Trajectory` |
| `tum_msgs::msg::TUMControlConstraints` | `tam::types::control::ControlConstraints` |
| `tum_msgs::msg::TUMControlConstraintPoint` | `tam::types::control::ControlConstraintPoint` |
| `autoware_auto_perception_msgs::msg::TrackedObjects` | `tam::types::prediction::TrackedObjects` |
| `autoware_auto_perception_msgs::msg::PredictedObjects` | `tam::types::prediction::PredictedObjects` |
| `geometry_msgs::msg::Quaternion` | `tam::types::common::EulerYPR` |


The following C++ types can be converted into a ROS type:

| C++ type | ROS type |
| -------- | ----------- |
| `uint64_t` | `builtin_interfaces::msg::Time` |

The following ROS types can be converted into a C++ type:

| ROS type | C++ type |
| -------- | ----------- |
| `nav_msgs::msg::Odometry` | `tam::types::control::Odometry` |
| `sensor_msgs::msg::Imu` | `tam::types::control::Odometry` |