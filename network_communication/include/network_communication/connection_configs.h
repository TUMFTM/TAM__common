// Copyright 2024 Simon Hoffmann
#pragma once
#include <string>
// NOTE These were resolved with dummy ports. This config will probably not even start.
// Ensure no port colissions in an actual config.
namespace tam::network
{
enum VehiclePorts {  // only what is sent via udp
  RX_CONNECTION_STATUS = 54321,
  RX_TUM_REQUEST = 54321,
  RX_FLAG_REQUEST = 54321,
  RX_JOYSTICK = 54321,
};
enum BasestationPorts {  // only what is sent via udp
  RX_PLANNING_DATA = 54321,
  RX_DIAGNOSTICS = 54321,
  RX_CONTROL_TELEMETRY = 54321,
  RX_STATE_ESTIMATION = 54321,
  RX_STATE_MACHINE_REPORT = 54321,
  RX_VEHICLE_STATE = 54321,
  RX_FLAG_REPORT = 54321,
  RX_LONG_SLIP_REPORT = 54321,
  RX_LAT_SLIP_REPORT = 54321,
  RX_TRACKED_OBJECTS = 54321,
  RX_GRIP_MAP_TELEMETRY = 54321,
  RX_GRIP_MAP_BUFFER = 54321,
  RX_LAP_TIMES = 54321,
  RX_SECTOR_TIMES = 54321
};
// PORTS 62xxx reserved for UDP-Bridge!!
// PORTS 504xx reserved for AMG-Bridge!!
};  // namespace tam::network
