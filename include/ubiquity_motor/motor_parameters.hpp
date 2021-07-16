/**
Copyright (c) 2016, Ubiquity Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef UBIQUITY_MOTOR__MOTOR_PARAMETERS_HPP_
#define UBIQUITY_MOTOR__MOTOR_PARAMETERS_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"

// These defines are for a high level topic for control of the motor node at a system level
#define ROS_TOPIC_SYSTEM_CONTROL "system_control"  // A topic for system level control commands
#define MOTOR_CONTROL_CMD "motor_control"          // A mnumonic for a motor control system command
#define MOTOR_CONTROL_ENABLE "enable"              // Parameter for MOTOR_CONTROL_CMD to enable control
#define MOTOR_CONTROL_DISABLE "disable"            // Parameter for MOTOR_CONTROL_CMD to enable control
#define MOTOR_SPEED_CONTROL_CMD "speed_control"    // A mnumonic for disable of speed to avoid colision

struct FirmwareParams
{
  int32_t pid_proportional;
  int32_t pid_integral;
  int32_t pid_derivative;
  int32_t pid_velocity;
  int32_t pid_denominator;
  int32_t pid_moving_buffer_size;
  int32_t controller_board_version;
  int32_t estop_detection;
  int32_t estop_pid_threshold;
  int32_t max_speed_fwd;
  int32_t max_speed_rev;
  int32_t max_pwm;
  int32_t deadman_timer;
  int32_t deadzone_enable;
  int32_t hw_options;
  int32_t option_switch;
  int32_t system_events;
  float battery_voltage_multiplier;
  float battery_voltage_offset;
  float battery_voltage_low_level;
  float battery_voltage_critical;

  FirmwareParams() {}    // NOTE(sam): Don't use this...

  explicit FirmwareParams(hardware_interface::HardwareInfo info)
  : pid_proportional(5000),       // pid settings from kinetic-devel branch
    pid_integral(7),
    pid_derivative(-110),
    pid_velocity(1500),
    pid_denominator(1000),
    pid_moving_buffer_size(10),
    controller_board_version(49),
    estop_detection(1),
    estop_pid_threshold(1500),
    max_speed_fwd(80),
    max_speed_rev(-80),
    max_pwm(250),
    deadman_timer(2400000),
    deadzone_enable(0),
    hw_options(0),
    option_switch(0),
    system_events(0),

    // ADC uses Vcc/2 for 2048 counts. We feed in battery with a 1/21 ratio
    // So for 5.00V mult=0.05127  When Vcc=5.16V (Pi4 mod) mult = 0.0529
    battery_voltage_multiplier(0.05127),         // See note above for this multiplier
    battery_voltage_offset(0.0),
    battery_voltage_low_level(23.2),
    battery_voltage_critical(22.5)
  {
    // TODO(sam): parse firmware params from hardware info
    (void)info;
    RCLCPP_WARN(
      rclcpp::get_logger("UbiquityMotorSystemHardware"),
      "Firmware parameter parsing is not yet implemented, using defaults");
  }
};

struct CommsParams
{
  std::string serial_port;
  int32_t baud_rate;

  CommsParams() {}    // NOTE(sam): Don't use this...

  explicit CommsParams(hardware_interface::HardwareInfo info)
  : serial_port("/dev/ttyS0"), baud_rate(9600)
  {
    auto params = info.hardware_parameters;

    if (params.find("serial_port") != params.end()) {
      serial_port = params["serial_port"];
    }

    if (params.find("baud_rate") != params.end()) {
      baud_rate = std::stoul(params["baud_rate"]);
    }
  }
};

struct SystemHardwareParams
{
  std::string wheel_type;
  std::string wheel_direction;

  int mcbControlEnabled;   // State to allow suspension of MCB serial control for diagnostic purposes
  int mcbSpeedEnabled;     // State to allow zero speed override for safety reasons

  SystemHardwareParams()
  : wheel_type("firmware_default"),
    wheel_direction("firmware_default"),
    mcbControlEnabled(1),
    mcbSpeedEnabled(1) {}

  explicit SystemHardwareParams(hardware_interface::HardwareInfo info)
  : wheel_type("firmware_default"),
    wheel_direction("firmware_default"),
    mcbControlEnabled(1),
    mcbSpeedEnabled(1)
  {
    // TODO(sam): parse firmware params from hardware info
    (void)info;
    RCLCPP_WARN(
      rclcpp::get_logger("UbiquityMotorSystemHardware"),
      "SystemHardware parameter parsing is not yet implemented, using defaults");
  }
};

#endif  // UBIQUITY_MOTOR__MOTOR_PARAMETERS_HPP_
