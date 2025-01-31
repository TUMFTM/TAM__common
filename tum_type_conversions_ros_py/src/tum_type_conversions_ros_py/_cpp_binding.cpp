// // Copyright 2023 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>

#include "tum_type_conversions_ros_cpp/orientation.hpp"
// #include <pybind11/stl.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Vector3.h>
// // include "pybind11_test/test_struct.hpp"

tam::types::common::EulerYPR _bound_quaternion_msg_to_euler_type(
  double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return tam::types::conversion::quaternion_msg_to_euler_type(q);
}
std::array<double, 4> _bound_euler_type_to_quaternion_msg(tam::types::common::EulerYPR euler_angles)
{
  auto q_msg = tam::types::conversion::euler_type_to_quaternion_msg(euler_angles);
  return {q_msg.x, q_msg.y, q_msg.z, q_msg.w};
}
namespace py = pybind11;
// REFACTOR Create a macro for binding by interface
PYBIND11_MODULE(_cpp_binding, m)
{
  m.def("_bound_quaternion_msg_to_euler_type", &_bound_quaternion_msg_to_euler_type)
    .def("_bound_euler_type_to_quaternion_msg", &_bound_euler_type_to_quaternion_msg);
}
