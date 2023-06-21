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

#ifndef UBIQUITY_MOTOR__MOTOR_SYSTEM_HPP_
#define UBIQUITY_MOTOR__MOTOR_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
// #include "ubiquity_motor/visibility_control.h" //TODO(sam): Add for windows support

#include "ubiquity_motor/motor_parameters.hpp"
#include "ubiquity_motor/motor_serial.hpp"
#include "ubiquity_motor/motor_interface.hpp"


namespace ubiquity_motor
{
class UbiquityMotorSystemHardware
  : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UbiquityMotorSystemHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::shared_ptr<MotorSerial> motor_serial_;
  std::shared_ptr<ubiquity_motor::MotorInterface> motor_interface_;

  std::chrono::milliseconds mcb_status_period_ms_;
  std::chrono::seconds sys_maintenance_period_sec_;
  std::chrono::milliseconds joint_update_period_ms_;

  std::chrono::_V2::system_clock::time_point last_joint_time_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  double hw_battery_voltage_;

  double left_last_wheel_pos_;
  double right_last_wheel_pos_;

  FirmwareParams firmware_params_;
  CommsParams comms_params_;
  SystemHardwareParams system_hardware_params_;

  uint right_wheel_joint_index_, left_wheel_joint_index_;

  void start_serial();
};

}  // namespace ubiquity_motor

#endif  // UBIQUITY_MOTOR__MOTOR_SYSTEM_HPP_
