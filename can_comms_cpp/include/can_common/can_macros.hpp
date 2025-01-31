#pragma once

/**
 * These macros should decrease duplicate code and simplify handling CAN message encoding and decoding
 * 
 * The idea is to call the correct funcitons based on their name and assemble the calls by frame and signal
 * 
 * !!! There is a strict version for all the macros, which REQUIRES THE FRAME VARIABLE TO HAVE THE SAME NAME AS THE FRAME IN THE DBC !!!
 * 
 * E.g: CAN_STRICT_SET(eav24, ice_status_01, ice_actual_gear, last_gear_) becomes
 * 
 *    v                                       v
 * ice_status_01.ice_actual_gear = eav24_ice_status_01_ice_actual_gear_encode(last_gear_)
 * 
 * The non-strict version allows passing a struct name as well:
 * 
 * CAN_SET(eav24, ice_msgs, ice_status_01, ice_actual_gear, last_gear_) becomes:
 * 
 * ice_msgs.ice_actual_gear = eav24_ice_status_01_ice_actual_gear_encode(last_gear_)
*/

#define CAN_SET(dbc, struct, message, signal_name, value) \
  struct.signal_name = dbc##_##message##_##signal_name##_encode(value)

#define CAN_STRICT_SET(dbc, struct, signal_name, value) \
  struct.signal_name =  dbc##_##struct##_##signal_name##_encode(value)

#define CAN_GET(dbc, struct, message, signal_name) \
   dbc##_##message##_##signal_name##_decode(struct.signal_name)

#define CAN_STRICT_GET(dbc, struct, signal_name) \
   dbc##_##struct##_##signal_name##_decode(struct.signal_name)

#define CAN_PACK(dbc, frame, struct, message, size) \
   dbc##_##message##_pack(frame->data.data(), &struct, size)

#define CAN_STRICT_PACK(dbc, frame, struct, size) \
   dbc##_##struct##_pack(frame->data.data(), &struct, size)

#define CAN_UNPACK(dbc, struct, frame, message, size) \
   dbc##_##message##_unpack(&struct, frame->data.data(), size)

#define CAN_STRICT_UNPACK(dbc, struct, frame, size) \
   dbc##_##struct##_unpack(&struct, frame->data.data(), size)

#define CAN_PACK_AND_SEND(dbc, struct, const_name, comms) \
  { \
    auto frame = std::make_shared<can_msgs::msg::Frame>(); \
    if (CAN_STRICT_PACK(dbc, frame, struct, const_name##_LENGTH) == const_name##_LENGTH) \
    { \
      frame->id = const_name##_FRAME_ID; \
      frame->is_extended = const_name##_IS_EXTENDED; \
      frame->dlc = const_name##_LENGTH; \
      frame->header.stamp = this->now(); \
      comms->send_can_direct(frame); \
    } \
    else \
    { \
      RCLCPP_ERROR(this->get_logger(), "Failed to pack %s", #const_name); \
    } \
  } \
