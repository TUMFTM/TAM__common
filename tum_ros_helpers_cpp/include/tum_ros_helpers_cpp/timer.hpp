// Copyright 2024 Simon Sagmeister
#include <rclcpp/rclcpp.hpp>
#include <utility>
namespace tam
{
/// @brief Create a timer that is either runs in simulation or wall time, depending of the value of
/// the parameter `use_sim_time`.
/// @note This is important since when using the standard `node->create_wall_timer()` function the
/// node (or more detailed - the timers of the node) will never be able to run in sim time
/// since they are wall timers that do not respect sim time. On the other hand,
/// always creating timers with the `rclcpp::create_timer()` function will
/// will result in a timer that is switchable between system clock and sim time. However, for
/// realtime deployment it is important to have a timer that runs with a steady clock (wall_timer).
/// Otherwise issues with time sync could interfere with the execution
/// of the timers. This is why we need this helper function.
/// @tparam DurationRepT
/// @tparam DurationT
/// @tparam CallbackT
/// @param node_ptr
/// @param period
/// @param callback
/// @param group
/// @return
template <typename DurationRepT, typename DurationT, typename CallbackT>
rclcpp::TimerBase::SharedPtr create_timer(
  rclcpp::Node * node_ptr, std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  // Check if node was started in sim time mode
  if (node_ptr->get_parameter("use_sim_time").as_bool()) {
    // Return a timer that is switchable between realtime and simulation time
    return rclcpp::create_timer(
      node_ptr, node_ptr->get_clock(), period, std::move(callback), group);
  } else {
    // Create and return wall timer (runs with a steady clock)
    return node_ptr->create_wall_timer(period, std::move(callback), group);
    return nullptr;
  }
}
}  // namespace tam
