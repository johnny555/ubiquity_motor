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

#include "ubiquity_motor/motor_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace ubiquity_motor
{
hardware_interface::return_type UbiquityMotorSystemHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  sys_maintenance_period_sec_ = 60s;   // A periodic MCB maintenance operation
  joint_update_period_ms_ = 250ms;   // A periodic time to update joint velocity

  firmware_params_ = FirmwareParams(info);
  comms_params_ = CommsParams(info);
  system_hardware_params_ = SystemHardwareParams(info);

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("UbiquityMotorSystemHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        (int)joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("UbiquityMotorSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("UbiquityMotorSystemHardware"),
        "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
        (int)joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("UbiquityMotorSystemHardware"),
        "Joint '%s' have '%s' as first state interface.'%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("UbiquityMotorSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  // Make sure that the system has a gpio named magni_IOs
  auto expected_gpio_name = "magni_IOs";
  auto it = std::find_if(info_.gpios.begin(), info_.gpios.begin(),
    [expected_gpio_name] (const hardware_interface::ComponentInfo & state_interface) {
      return state_interface.name == expected_gpio_name;
    }
  );
  if (it == info_.gpios.end())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("UbiquityMotorSystemHardware"),
      "The system does not have the expected gpio %s in the urdf", expected_gpio_name);
    return hardware_interface::return_type::ERROR;
  }

  for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
    if (gpio.name == "magni_IOs")
    {
      // Make sure that the magni_IOs gpio has a battery_voltage state interface
      auto expected_state_interface_name = "battery_voltage";
      auto it = std::find_if(gpio.state_interfaces.begin(), gpio.state_interfaces.end(),
        [expected_state_interface_name] (const hardware_interface::InterfaceInfo & state_interface) {
          return state_interface.name == expected_state_interface_name;
        }
      );
      if (it == gpio.state_interfaces.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("UbiquityMotorSystemHardware"),
          "Gpio '%s' does not have the expected state interface '%s' in the urdf", gpio.name.c_str(),
          expected_state_interface_name);
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  bool right_wheel_joint_assigned = false;
  bool left_wheel_joint_assigned = false;

  for (uint i = 0; i < info_.joints.size(); i++) {
    std::string left_wheel_joint_name = info_.hardware_parameters["left_wheel_joint_name"];
    if (info_.joints[i].name == left_wheel_joint_name) {
      left_wheel_joint_index_ = i;
      left_wheel_joint_assigned = true;
    }

    std::string right_wheel_joint_name = info_.hardware_parameters["right_wheel_joint_name"];
    if (info_.joints[i].name == right_wheel_joint_name) {
      right_wheel_joint_index_ = i;
      right_wheel_joint_assigned = true;
    }
  }

  if (!left_wheel_joint_assigned) {
    RCLCPP_FATAL(
      rclcpp::get_logger("UbiquityMotorSystemHardware"),
      "Left wheel joint '%s' was not found.",
      info_.hardware_parameters["left_wheel_joint_name"].c_str());
    return hardware_interface::return_type::ERROR;
  }

  if (!right_wheel_joint_assigned) {
    RCLCPP_FATAL(
      rclcpp::get_logger("UbiquityMotorSystemHardware"),
      "Right wheel joint '%s' was not found.",
      info_.hardware_parameters["left_wheel_joint_name"].c_str());
    return hardware_interface::return_type::ERROR;
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> UbiquityMotorSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "magni_IOs", "battery_voltage", &hw_battery_voltage_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UbiquityMotorSystemHardware::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type UbiquityMotorSystemHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("UbiquityMotorSystemHardware"), "Starting ...please wait...");

  // Keep trying to open serial
  int times = 0;
  while (rclcpp::ok() && motor_serial_.get() == nullptr) {
    try {
      start_serial();
    } catch (const serial::IOException & e) {
      if (times % 30 == 0) {
        RCLCPP_WARN(
          rclcpp::get_logger("UbiquityMotorSystemHardware"),
          "Error opening serial port %s with baud_rate %d, trying again",
          comms_params_.serial_port.c_str(), comms_params_.baud_rate);
      }
    }
    rclcpp::sleep_for(100ms);
    times++;

    if (times > 100) {
      // Fail setup if it takes more than ~10 seconds
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "UbiquityMotorSystemHardware"),
        "System Failed to start! Could not start serial communication.");
      // return hardware_interface::return_type::ERROR;
    }
  }

  motor_interface_.reset();
  motor_interface_ = std::make_shared<MotorInterface>(
    motor_serial_,
    system_hardware_params_,
    firmware_params_);

  motor_interface_->request_firmware_version();
  rclcpp::sleep_for(motor_interface_->mcb_status_period_ms_);

  // Make sure firmware is listening
  {
    // Start times counter at 1 to prevent false error print (0 % n = 0)
    int times = 1;
    while (rclcpp::ok() && motor_interface_->firmware_version_ == 0) {
      if (times % 30 == 0) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "UbiquityMotorSystemHardware"), "The Firmware is not reporting its version");
      }

      motor_interface_->request_firmware_version();
      motor_interface_->read_serial_inputs();
      rclcpp::sleep_for(100ms);
      times++;

      if (times > 100) {
        // Fail setup if it takes more than ~10 seconds
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "UbiquityMotorSystemHardware"), "Could not read frimware version!");
        // status_ = hardware_interface::status::STOPPED;
        return hardware_interface::return_type::ERROR;   // NOTE(sam): does not seem to prevent read/write from running
      }
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("UbiquityMotorSystemHardware"),
    "The Firmware reported its version to be: %d", motor_interface_->firmware_version_);

  if (motor_interface_->firmware_version_ >= MIN_FW_FIRMWARE_DATE) {
    // If supported by firmware also request date code for this version
    RCLCPP_INFO(rclcpp::get_logger("UbiquityMotorSystemHardware"), "Requesting Firmware daycode");
    motor_interface_->request_firmware_date();
  }

  RCLCPP_INFO(rclcpp::get_logger("UbiquityMotorSystemHardware"), "Initializing MCB");
  motor_interface_->init_mcb_parameters();
  RCLCPP_INFO(rclcpp::get_logger("UbiquityMotorSystemHardware"), "Initialization of MCB completed");

  if (motor_interface_->firmware_version_ >= MIN_FW_SYSTEM_EVENTS) {
    // Start out with zero for system events
    motor_interface_->set_system_events(0);      // Clear entire system events register
    motor_interface_->system_events_ = 0;
    rclcpp::sleep_for(motor_interface_->mcb_status_period_ms_);
  }

  // Send out the firmware parameters, most are the PID terms
  // We must be sure num_fw_params is set to the modulo used in sendParams()
  for (int i = 0; i < motor_interface_->num_firmware_params_; i++) {
    rclcpp::sleep_for(motor_interface_->mcb_status_period_ms_);
    motor_interface_->send_params();
  }

  // set some default values for the hardware joint interface
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("UbiquityMotorSystemHardware"), "System Successfully started!");

  // TODO(sam): Add ESTOP speed reset with delay

  return hardware_interface::return_type::OK;
}

void UbiquityMotorSystemHardware::start_serial()
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "UbiquityMotorSystemHardware"), "Delay before MCB serial port initialization");

  rclcpp::sleep_for(3s);

  RCLCPP_INFO(
    rclcpp::get_logger("UbiquityMotorSystemHardware"),
    "Initialize MCB serial port '%s' for %d baud",
    comms_params_.serial_port.c_str(), comms_params_.baud_rate);

  motor_serial_.reset();
  motor_serial_ = std::make_shared<MotorSerial>(
    comms_params_.serial_port,
    comms_params_.baud_rate);

  RCLCPP_INFO(rclcpp::get_logger("UbiquityMotorSystemHardware"), "MCB serial port initialized");
}

hardware_interface::return_type UbiquityMotorSystemHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("UbiquityMotorSystemHardware"), "Stopping ...please wait...");

  // Stop robot and close serial port
  motor_interface_->write_speeds_in_radians(0.0, 0.0);
  rclcpp::sleep_for(100ms);
  motor_serial_->closePort();

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("UbiquityMotorSystemHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UbiquityMotorSystemHardware::read()
{
  motor_interface_->read_serial_inputs();

  auto current_time = std::chrono::system_clock::now();
  std::chrono::duration<float> dt = current_time - last_joint_time_;
  RCLCPP_DEBUG(rclcpp::get_logger("UbiquityMotorSystemHardware"), "Read dt: %.5f", dt.count());

  // Get most recent hardware positions
  hw_positions_[left_wheel_joint_index_] = motor_interface_->left_joint_hardware_position_;
  hw_positions_[right_wheel_joint_index_] = motor_interface_->right_joint_hardware_position_;

  // Determine and set wheel velocities in rad/sec from hardware positions in rads
  if (dt > joint_update_period_ms_) {
    last_joint_time_ = std::chrono::system_clock::now();

    auto left_wheel_pos = hw_positions_[left_wheel_joint_index_];
    auto right_wheel_pos = hw_positions_[right_wheel_joint_index_];

    hw_velocities_[left_wheel_joint_index_] = (left_wheel_pos - left_last_wheel_pos_) / dt.count();
    hw_velocities_[right_wheel_joint_index_] = (right_wheel_pos - right_last_wheel_pos_) /
      dt.count();

    left_last_wheel_pos_ = left_wheel_pos;
    right_last_wheel_pos_ = right_wheel_pos;
  }

  // Get most recent battery voltage
  hw_battery_voltage_ = motor_interface_->battery_voltage_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ubiquity_motor::UbiquityMotorSystemHardware::write()
{
  // TODO(sam): handle motor control is disabled

  // TODO(sam): Handle motor control is disabled and re enable? or not? unclear usecase...

  // TODO(sam): Reset controller for time jump

  double left_radians = hw_commands_[left_wheel_joint_index_];
  double right_radians = hw_commands_[right_wheel_joint_index_];

  RCLCPP_DEBUG(
    rclcpp::get_logger("UbiquityMotorSystemHardware"),
    "Writing %.5f to left and %.5f to left wheel", left_radians, right_radians);

  motor_interface_->write_speeds_in_radians(left_radians, right_radians);

  // TODO(sam): Handle global disable (estop_state), estop_release_delay and mcb_speed_enabled

  return hardware_interface::return_type::OK;
}

}  // namespace ubiquity_motor

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ubiquity_motor::UbiquityMotorSystemHardware, hardware_interface::SystemInterface)
