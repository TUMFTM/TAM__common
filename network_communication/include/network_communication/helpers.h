#pragma once
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sysexits.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <string>
#include <vector>
namespace tam::network::helpers
{
template <typename T>
inline std::vector<uint8_t> serialize_ros_msg(T & msg)
{
  rclcpp::SerializedMessage serMsg;
  rclcpp::Serialization<T> serializer;
  try {
    serializer.serialize_message(&msg, &serMsg);
    uint8_t * data = serMsg.get_rcl_serialized_message().buffer;
    int size = serMsg.get_rcl_serialized_message().buffer_length;
    std::vector<uint8_t> serialized(data, data + size);
    return serialized;
  } catch (std::exception & ex) {
    // sometimes buffer is incomplete - ignore
    printf("could not serialize (failed on %s)\n", ex.what());
  }
  return std::vector<uint8_t>();
}
inline rclcpp::SerializedMessage get_serialized_msg_from_data(const std::vector<uint8_t> & data)
{
  // write data from buffer into serialized message
  rclcpp::SerializedMessage serMsg(data.size() + 1);
  auto & rcl_handle = serMsg.get_rcl_serialized_message();

  std::memcpy(rcl_handle.buffer, data.data(), data.size());
  rcl_handle.buffer[data.size()] = '\0';
  rcl_handle.buffer_length = static_cast<size_t>(data.size());

  return serMsg;
}
template <typename T>
inline std::optional<T> deserialize_ros_msg(const std::vector<uint8_t> & data)
{
  if (!rclcpp::ok()) return std::nullopt;

  // write data from buffer into serialized message
  rclcpp::SerializedMessage serMsg = get_serialized_msg_from_data(data);
  rclcpp::Serialization<T> serializer;
  try {
    T msg;
    serializer.deserialize_message(&serMsg, &msg);
    return msg;
  } catch (std::exception & ex) {
    // sometimes buffer is incomplete - ignore
    printf("could not deserialize (failed on %s)\n", ex.what());
  }
  return std::nullopt;
}
/// @brief Get a list the IPv4-Addresses of all your network adapters
/// From https://dev.to/fmtweisszwerg/cc-how-to-get-all-interface-addresses-on-the-local-device-3pki
/// @return List of IPv4 Addresses of all network adapters
static std::vector<std::string> get_current_ip_addresses()
{
  std::vector<std::string> out;
  struct ifaddrs * ptr_ifaddrs = nullptr;
  auto result = getifaddrs(&ptr_ifaddrs);
  if (result != 0) {
    std::cout << "`getifaddrs()` failed: " << strerror(errno) << std::endl;
    return out;
  }

  for (struct ifaddrs * ptr_entry = ptr_ifaddrs; ptr_entry != nullptr;
       ptr_entry = ptr_entry->ifa_next) {
    std::string ipaddress_human_readable_form;

    std::string interface_name = std::string(ptr_entry->ifa_name);
    sa_family_t address_family;
    if (ptr_entry->ifa_addr) {
      address_family = ptr_entry->ifa_addr->sa_family;
    } else {
      continue;
    }

    if (address_family == AF_INET) {
      // IPv4
      if (ptr_entry->ifa_addr != nullptr) {
        char buffer[INET_ADDRSTRLEN] = {
          0,
        };
        inet_ntop(
          address_family, &((struct sockaddr_in *)(ptr_entry->ifa_addr))->sin_addr, buffer,
          INET_ADDRSTRLEN);

        ipaddress_human_readable_form = std::string(buffer);
        out.push_back(ipaddress_human_readable_form);
      }

    } else {
      // AF_UNIX, AF_UNSPEC, AF_PACKET, IPV6 etc.
      // If ignored, delete this section.
    }
  }

  freeifaddrs(ptr_ifaddrs);
  return out;
}
static std::optional<int> get_own_ip_idx_in_list(const std::vector<std::string> & list)
{
  auto own_ips = get_current_ip_addresses();

  for (int i = 0; i < list.size(); i++) {
    if (std::find(own_ips.begin(), own_ips.end(), list.at(i)) != own_ips.end()) {
      // Element in vector.
      return i;
    }
  }
  return std::nullopt;
}
static std::string get_string_from_ip_list(const std::vector<std::string> & ip_list)
{
  std::string list = "[";
  for (size_t i = 0; i < ip_list.size(); i++)
  {
      list += ip_list.at(i);
      if(i < ip_list.size() - 1)
      {
          list += ",";
      }
  }
  return list + "]";
}
static std::string get_string_from_sockaddr_list(const std::vector<sockaddr_in> & ip_list)
{
  std::string list = "[";
  for (size_t i = 0; i < ip_list.size(); i++)
  {
      list += std::string(inet_ntoa(ip_list.at(i).sin_addr));
      if(i < ip_list.size() - 1)
      {
          list += ",";
      }
  }
  return list + "]";
}
};  // namespace tam::network::helpers