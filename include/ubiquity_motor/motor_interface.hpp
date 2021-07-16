/**
Copyright (c) 2016 Ubiquity Robotics, 2021 Dyno Robotics
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

#ifndef UBIQUITY_MOTOR__MOTOR_INTERFACE_HPP_
#define UBIQUITY_MOTOR__MOTOR_INTERFACE_HPP_

#include <memory>

#include "ubiquity_motor/motor_parameters.hpp"
#include "ubiquity_motor/motor_serial.hpp"

namespace ubiquity_motor
{
class MotorInterface
{
public:
  MotorInterface(
    std::shared_ptr<MotorSerial> motor_serial,
    SystemHardwareParams system_hardware_params,
    FirmwareParams firmware_params);

  double left_joint_hardware_position_;
  double right_joint_hardware_position_;

  std::chrono::milliseconds mcb_status_period_ms_;

  int firmware_version_;
  int firmware_date_;
  int firmware_options_;
  int hardware_version_;
  int estop_pid_threshold_;
  int max_speed_fwd_;
  int max_speed_rev_;
  int max_pwm_;
  int deadman_enable_;
  int system_events_;
  int wheel_type_;

  int32_t deadman_timer_;

  float battery_voltage_;

  double ticks_per_radian_;   // Odom ticks per radian for wheel encoders in use
  bool estop_motor_power_off_;   // Motor power inactive, most likely from ESTOP switch

  int32_t send_pid_count_;
  int num_firmware_params_;    // This is used for sendParams as modulo count

  void read_serial_inputs();

  void init_mcb_parameters();

  void request_firmware_version();
  void request_firmware_date();
  void request_system_events();

  bool get_estop_state();
  float get_battery_voltage();

  void set_hardware_version(int32_t hardware_version);
  void set_max_fwd_speed(int32_t max_speed_fwd);
  void set_estop_pid_threshold(int32_t estop_pid_threshold);
  void set_estop_detection(int32_t estop_detection);
  void set_wheel_type(int32_t new_wheel_type);
  void set_wheel_direction(int32_t wheel_direction);
  void set_option_switch_reg(int32_t option_switch_bits);
  void set_system_events(int32_t system_events);
  void set_max_rev_speed(int32_t max_speed_rev);
  void set_max_pwm(int32_t max_pwm);
  void set_deadman_timer(int32_t deadman_timer);
  void set_deadzone_enable(int32_t dadzone_enable);
  void set_debug_leds(bool led_1, bool led_2);

  void write_speeds_in_radians(double left_radians, double right_radians);

  void send_params();

  int16_t calculate_speed_from_radians(double radians) const;
  double calculate_radians_from_ticks(int16_t ticks) const;

private:
  std::shared_ptr<MotorSerial> motor_serial_;

  FirmwareParams firmware_params_;
  SystemHardwareParams system_hardware_params_;
};
}  // namespace ubiquity_motor

#endif  // UBIQUITY_MOTOR__MOTOR_INTERFACE_HPP_
