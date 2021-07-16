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

#include <memory>
#include <vector>

#include <boost/math/special_functions/round.hpp>
#include "ubiquity_motor/motor_interface.hpp"

#define I2C_DEVICE "/dev/i2c-1"                     // This is specific to default Magni I2C port on host
static const uint8_t I2C_PCF8574_8BIT_ADDR = 0x40;  // I2C addresses are 7 bits but often shown as 8-bit

// #define SENSOR_DISTANCE 0.002478

// For experimental purposes users will see that the wheel encoders are three phases
// of very neaar 43 pulses per revolution or about 43*3 edges so we see very about 129 ticks per rev
// This leads to 129/(2*Pi)  or about 20.53 ticks per radian experimentally.
// Below we will go with the exact ratio from gearbox specs
// 60 ticks per revolution of the motor (pre gearbox)
// 17.2328767123 and  gear ratio of 4.29411764706:1
#define TICKS_PER_RADIAN_ENC_3_STATE (20.50251516) // used to read more misleading value of (41.0058030317/2)
#define QTICKS_PER_RADIAN (ticks_per_radian_ * 4)  // Quadrature ticks makes code more readable later

#define VELOCITY_READ_PER_SECOND 10.0  // read = ticks / (100 ms), so we have scale of 10 for ticks/second
#define LOWEST_FIRMWARE_VERSION 28

using namespace std::chrono_literals;

namespace ubiquity_motor
{
MotorInterface::MotorInterface(
  std::shared_ptr<MotorSerial> motor_serial,
  SystemHardwareParams system_hardware_params,
  FirmwareParams firmware_params)
{
  motor_serial_ = motor_serial;

  system_hardware_params_ = system_hardware_params;
  firmware_params_ = firmware_params;

  left_joint_hardware_position_ = 0.0;
  right_joint_hardware_position_ = 0.0;

  mcb_status_period_ms_ = 20ms;
  ticks_per_radian_ = TICKS_PER_RADIAN_ENC_3_STATE;

  send_pid_count_ = 0;
  num_firmware_params_ = 7;     // number of params sent if any change

  firmware_version_ = 0;     // Set to zero to make sure we are reading it off the hardware
}

void MotorInterface::request_firmware_version()
{
  MotorMessage fw_version_msg;
  fw_version_msg.setRegister(MotorMessage::REG_FIRMWARE_VERSION);
  fw_version_msg.setType(MotorMessage::TYPE_READ);
  fw_version_msg.setData(0);
  motor_serial_->transmitCommand(fw_version_msg);
}

//  Setup MCB parameters that are from host level options or settings
void ubiquity_motor::MotorInterface::init_mcb_parameters()
{
  auto logger = rclcpp::get_logger("init_mcb_parameters");
  // A full mcb initialization requires high level system overrides to be disabled

  system_hardware_params_.mcbControlEnabled = 1;
  system_hardware_params_.mcbSpeedEnabled = 1;

  // Force future calls to sendParams() to update current pid parametes on the MCB
  // robot->forcePidParamUpdates();

  // Determine the wheel type to be used by the robot base
  int32_t wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
  if (system_hardware_params_.wheel_type == "firmware_default") {
    // Here there is no specification so the firmware default will be used
    RCLCPP_INFO(logger, "Default wheel_type of 'standard' will be used.");
    wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
  } else {
    // Any other setting leads to host setting the wheel type
    if (system_hardware_params_.wheel_type == "standard") {
      wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
      RCLCPP_INFO(logger, "Host is specifying wheel_type of '%s'", "standard");
    } else if (system_hardware_params_.wheel_type == "thin") {
      wheel_type = MotorMessage::OPT_WHEEL_TYPE_THIN;
      RCLCPP_INFO(logger, "Host is specifying wheel_type of '%s'", "thin");
    } else {
      RCLCPP_WARN(
        logger, "Invalid wheel_type of '%s' specified! Using wheel type of standard",
        system_hardware_params_.wheel_type.c_str());
      system_hardware_params_.wheel_type = "standard";
      wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
    }
  }


  // Write out the wheel type setting to hardware layer
  set_wheel_type(wheel_type);
  wheel_type = wheel_type;

  rclcpp::sleep_for(mcb_status_period_ms_);

  int32_t wheel_direction = 0;
  if (system_hardware_params_.wheel_direction == "firmware_default") {
    // Here there is no specification so the firmware default will be used
    RCLCPP_INFO(logger, "Firmware default wheel_direction will be used.");
  } else {
    // Any other setting leads to host setting the wheel type
    if (system_hardware_params_.wheel_direction == "standard") {
      wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
      RCLCPP_INFO(logger, "Host is specifying wheel_direction of '%s'", "standard");
    } else if (system_hardware_params_.wheel_direction == "reverse") {
      wheel_direction = MotorMessage::OPT_WHEEL_DIR_REVERSE;
      RCLCPP_INFO(logger, "Host is specifying wheel_direction of '%s'", "reverse");
    } else {
      RCLCPP_WARN(
        logger, "Invalid wheel_direction of '%s' specified! Using wheel direction of standard",
        system_hardware_params_.wheel_direction.c_str());
      system_hardware_params_.wheel_direction = "standard";
      wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
    }
    // Write out the wheel direction setting
    set_wheel_direction(wheel_direction);
    rclcpp::sleep_for(mcb_status_period_ms_);
  }

  // Tell the controller board firmware what version the hardware is at this time.
  // TODO(?): Read from I2C.   At this time we only allow setting the version from ros parameters
  if (firmware_version_ >= MIN_FW_HW_VERSION_SET) {
    RCLCPP_INFO_ONCE(
      logger, "Firmware is version %d. Setting Controller board version to %d",
      firmware_version_, firmware_params_.controller_board_version);
    set_hardware_version(firmware_params_.controller_board_version);
    RCLCPP_DEBUG(
      logger, "Controller board version has been set to %d",
      firmware_params_.controller_board_version);
    rclcpp::sleep_for(mcb_status_period_ms_);
  }

  // TODO(sam): Don't use on hardware revision 4.9
  // Tell the MCB board what the I2C port on it is set to (mcb cannot read it's own switchs!)
  // We could re-read periodically but perhaps only every 5-10 sec but should do it from main loop
  // if (firmware_version_ >= MIN_FW_OPTION_SWITCH)
  // {
  //   firmware_params_.option_switch = get_option_switch();
  //   RCLCPP_INFO(logger, "Setting firmware option register to 0x%x.", firmware_params_.option_switch);
  //   set_option_switch_reg(firmware_params_.option_switch);
  //   rclcpp::sleep_for(mcb_status_period_sec_);
  // }

  if (firmware_version_ >= MIN_FW_SYSTEM_EVENTS) {
    // Start out with zero for system events
    set_system_events(0);   // Clear entire system events register
    system_events_ = 0;
    rclcpp::sleep_for(mcb_status_period_ms_);
  }

  // Setup other firmware parameters that could come from ROS parameters
  if (firmware_version_ >= MIN_FW_ESTOP_SUPPORT) {
    set_estop_pid_threshold(firmware_params_.estop_pid_threshold);
    rclcpp::sleep_for(mcb_status_period_ms_);

    set_estop_detection(firmware_params_.estop_detection);
    rclcpp::sleep_for(mcb_status_period_ms_);
  }

  if (firmware_version_ >= MIN_FW_MAX_SPEED_AND_PWM) {
    set_max_fwd_speed(firmware_params_.max_speed_fwd);
    rclcpp::sleep_for(mcb_status_period_ms_);

    set_max_rev_speed(firmware_params_.max_speed_rev);
    rclcpp::sleep_for(mcb_status_period_ms_);
  }
}

// read_serial_inputs() will receive serial and act on the response from motor controller
//
// The motor controller sends unsolicited messages periodically so we must read the
// messages to update status in near realtime
//
void MotorInterface::read_serial_inputs()
{
  auto logger = rclcpp::get_logger("MotorCommands: read_serial_inputs()");

  while (motor_serial_->commandAvailable()) {
    MotorMessage mm;
    mm = motor_serial_->receiveCommand();
    if (mm.getType() == MotorMessage::TYPE_RESPONSE) {
      switch (mm.getRegister()) {
        case MotorMessage::REG_SYSTEM_EVENTS:
          if ((mm.getData() & MotorMessage::SYS_EVENT_POWERON) != 0) {
            RCLCPP_WARN(logger, "Firmware System Event for PowerOn transition");
            system_events_ = mm.getData();
          }
          break;
        case MotorMessage::REG_FIRMWARE_VERSION:
          if (mm.getData() < LOWEST_FIRMWARE_VERSION) {
            RCLCPP_FATAL(
              logger, "Firmware version %d, expect %d or above",
              mm.getData(), LOWEST_FIRMWARE_VERSION);
            throw std::runtime_error("Firmware version too low");
          } else {
            RCLCPP_INFO_ONCE(logger, "Firmware version %d", mm.getData());
            firmware_version_ = mm.getData();
            // motor_diag_.firmware_version = firmware_version;
            // TODO(sam): Publish from Ubuiquity Motor Diagnostics Broadcaster?
          }
          break;

        case MotorMessage::REG_FIRMWARE_DATE:
          // Firmware date is only supported as of fw version MIN_FW_FIRMWARE_DATE
          RCLCPP_INFO_ONCE(logger, "Firmware date 0x%x (format 0xYYYYMMDD)", mm.getData());
          firmware_date_ = mm.getData();
          // motor_diag_.firmware_date = firmware_date;
          // TODO(sam): Publish from Ubuiquity Motor Diagnostics Broadcaster?
          break;

        case MotorMessage::REG_BOTH_ODOM:
          {
            /*
            * ODOM messages from the MCB tell us how far wheels have rotated
            *
            * It is here we keep track of wheel joint position
            * The odom counts from the MCB are the incremental number of ticks since last report
            *  WARNING: IF WE LOOSE A MESSAGE WE DRIFT FROM REAL POSITION
            */

            int32_t odom = mm.getData();
            // RCLCPP_ERROR(logger, "odom signed %d", odom);
            int16_t odomLeft = (odom >> 16) & 0xffff;
            int16_t odomRight = odom & 0xffff;

            // Debug code to be used for verification
            // g_odomLeft += odomLeft;
            // g_odomRight += odomRight;
            // g_odomEvent += 1;
            //if ((g_odomEvent % 50) == 1) { RCLCPP_ERROR(logger, "leftOdom %d rightOdom %d", g_odomLeft, g_odomRight); }

            // Add or subtract from position in radians using the incremental odom value
            left_joint_hardware_position_ += (odomLeft / ticks_per_radian_);
            right_joint_hardware_position_ += (odomRight / ticks_per_radian_);

            // motor_diag_.odom_update_status.tick(); // Let diag know we got odom
            // TODO(sam): Publish from Ubuiquity Motor Diagnostics Broadcaster?
            break;
          }
        case MotorMessage::REG_BOTH_ERROR:
          {
            // int32_t speed = mm.getData();
            // int16_t leftSpeed = (speed >> 16) & 0xffff;
            // int16_t rightSpeed = speed & 0xffff;

            // TODO(sam): Publish from Ubuiquity Motor Diagnostics Broadcaster?
            break;
          }

        case MotorMessage::REG_HW_OPTIONS:
          {
            int32_t data = mm.getData();

            // Enable or disable hardware options reported from firmware
            // TODO(sam): Publish data from Ubuiquity Motor Diagnostics Broadcaster?

            // Set radians per encoder tic based on encoder specifics
            if (data & MotorMessage::OPT_ENC_6_STATE) {
              RCLCPP_WARN_ONCE(logger, "Encoder Resolution: 'Enhanced'");
              firmware_params_.hw_options |= MotorMessage::OPT_ENC_6_STATE;
              ticks_per_radian_ = TICKS_PER_RADIAN_ENC_3_STATE * 2;
            } else {
              RCLCPP_WARN_ONCE(logger, "Encoder Resolution: 'Standard'");
              firmware_params_.hw_options &= ~MotorMessage::OPT_ENC_6_STATE;
              ticks_per_radian_ = TICKS_PER_RADIAN_ENC_3_STATE;
            }

            if (data & MotorMessage::OPT_WHEEL_TYPE_THIN) {
              RCLCPP_WARN_ONCE(logger, "Wheel type is: 'thin'");
              firmware_params_.hw_options |= MotorMessage::OPT_WHEEL_TYPE_THIN;
            } else {
              RCLCPP_WARN_ONCE(logger, "Wheel type is: 'standard'");
              firmware_params_.hw_options &= ~MotorMessage::OPT_WHEEL_TYPE_THIN;
            }

            if (data & MotorMessage::OPT_WHEEL_DIR_REVERSE) {
              RCLCPP_WARN_ONCE(logger, "Wheel direction is: 'reverse'");
              firmware_params_.hw_options |= MotorMessage::OPT_WHEEL_DIR_REVERSE;
            } else {
              RCLCPP_WARN_ONCE(logger, "Wheel direction is: 'standard'");
              firmware_params_.hw_options &= ~MotorMessage::OPT_WHEEL_DIR_REVERSE;
            }
            break;
          }

        case MotorMessage::REG_LIMIT_REACHED:
          {
            int32_t data = mm.getData();

            // TODO(sam): Publish from Ubuiquity Motor Diagnostics Broadcaster?

            if (data & MotorMessage::LIM_M1_PWM) {
              RCLCPP_WARN(logger, "left PWM limit reached");
              // motor_diag_.left_pwm_limit = true;
            }
            if (data & MotorMessage::LIM_M2_PWM) {
              RCLCPP_WARN(logger, "right PWM limit reached");
              // motor_diag_.right_pwm_limit = true;
            }
            if (data & MotorMessage::LIM_M1_INTEGRAL) {
              RCLCPP_DEBUG(logger, "left Integral limit reached");
              // motor_diag_.left_integral_limit = true;
            }
            if (data & MotorMessage::LIM_M2_INTEGRAL) {
              RCLCPP_DEBUG(logger, "right Integral limit reached");
              // motor_diag_.right_integral_limit = true;
            }
            if (data & MotorMessage::LIM_M1_MAX_SPD) {
              RCLCPP_WARN(logger, "left Maximum speed reached");
              // motor_diag_.left_max_speed_limit = true;
            }
            if (data & MotorMessage::LIM_M2_MAX_SPD) {
              RCLCPP_WARN(logger, "right Maximum speed reached");
              // motor_diag_.right_max_speed_limit = true;
            }
            if (data & MotorMessage::LIM_PARAM_LIMIT) {
              RCLCPP_WARN_ONCE(logger, "parameter limit in firmware");
              // motor_diag_.param_limit_in_firmware = true;
            }
            break;
          }
        case MotorMessage::REG_BATTERY_VOLTAGE:
          {
            // TODO(sam): Publish from Ubuiquity Motor Diagnostics Broadcaster

            // int32_t data = mm.getData();
            // sensor_msgs::msg::BatteryState bstate;
            // bstate.voltage = (float)data * firmware_params_.battery_voltage_multiplier +
            //                  firmware_params_.battery_voltage_offset;
            // bstate.current = std::numeric_limits<float>::quiet_NaN();
            // bstate.charge = std::numeric_limits<float>::quiet_NaN();
            // bstate.capacity = std::numeric_limits<float>::quiet_NaN();
            // bstate.design_capacity = std::numeric_limits<float>::quiet_NaN();
            // bstate.percentage = std::max(0.0, std::min(1.0, (bstate.voltage - 20.0) * 0.125));
            // bstate.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
            // bstate.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
            // bstate.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
            // battery_state_pub->publish(bstate);

            // motor_diag_.battery_voltage = bstate.voltage;
            // motor_diag_.battery_voltage_low_level = MotorCommands::firmware_params_.battery_voltage_low_level;
            // motor_diag_.battery_voltage_critical = MotorCommands::firmware_params_.battery_voltage_critical;
            break;
          }
        case MotorMessage::REG_MOT_PWR_ACTIVE:
          { // Starting with rev 5.0 board we can see power state
            int32_t data = mm.getData();

            if (data & MotorMessage::MOT_POW_ACTIVE) {
              if (estop_motor_power_off_ == true) {
                RCLCPP_WARN(
                  logger,
                  "Motor power has gone from inactive to active. Most likely from ESTOP switch");
              }
              estop_motor_power_off_ = false;
            } else {
              if (estop_motor_power_off_ == false) {
                RCLCPP_WARN(
                  logger,
                  "Motor power has gone inactive. Most likely from ESTOP switch active");
              }
              estop_motor_power_off_ = true;
            }
            // TODO(sam): Publish estop_motor_power_off_ from Ubuiquity Motor Diagnostics Broadcaster
            break;
          }

        case MotorMessage::REG_TINT_BOTH_WHLS:
          { // As of v41 show time between wheel enc edges
            int32_t data = mm.getData();
            uint16_t left_tick_spacing = (data >> 16) & 0xffff;
            uint16_t right_tick_spacing = data & 0xffff;
            uint16_t tickCap = 0;  // We can cap the max value if desired

            if ((tickCap > 0) && (left_tick_spacing > tickCap)) {
              left_tick_spacing = tickCap;
            }
            if ((tickCap > 0) && (right_tick_spacing > tickCap)) {
              right_tick_spacing = tickCap;
            }

            // Only publish the tic intervals when wheels are moving
            if (data > 1) {
              // Optionally show the intervals for debug
              // TODO(sam): Publish from Ubuiquity Motor Diagnostics Broadcaster?

              RCLCPP_DEBUG(
                logger, "Tic Ints M1 %d [0x%x]  M2 %d [0x%x]",
                left_tick_spacing, left_tick_spacing, right_tick_spacing, right_tick_spacing);
            }
            break;
          }
        default:
          break;
      }
    }
  }
}

// Firmware date register implemented as of MIN_FW_FIRMWARE_DATE
void MotorInterface::request_firmware_date()
{
  MotorMessage fw_date_msg;
  fw_date_msg.setRegister(MotorMessage::REG_FIRMWARE_DATE);
  fw_date_msg.setType(MotorMessage::TYPE_READ);
  fw_date_msg.setData(0);
  motor_serial_->transmitCommand(fw_date_msg);
}

// Request the MCB system event register
void MotorInterface::request_system_events()
{
  MotorMessage sys_event_msg;
  sys_event_msg.setRegister(MotorMessage::REG_SYSTEM_EVENTS);
  sys_event_msg.setType(MotorMessage::TYPE_READ);
  sys_event_msg.setData(0);
  motor_serial_->transmitCommand(sys_event_msg);
}

// Due to greatly limited pins on the firmware processor the host figures out the hardware rev and sends it to fw
// The hardware version is 0x0000MMmm  where MM is major rev like 4 and mm is minor rev like 9 for first units.
// The 1st firmware version this is set for is 32, before it was always 1
void MotorInterface::set_hardware_version(int32_t hardware_version)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "MotorCommands"), "setting hardware_version to %x", (int)hardware_version);
  MotorMessage mm;
  mm.setRegister(MotorMessage::REG_HARDWARE_VERSION);
  mm.setType(MotorMessage::TYPE_WRITE);
  mm.setData(hardware_version);
  motor_serial_->transmitCommand(mm);
}

// Setup the controller board threshold to put into force estop protection on boards prior to rev 5.0 with hardware support
void MotorInterface::set_estop_pid_threshold(int32_t estop_pid_threshold)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "MotorCommands"), "setting Estop PID threshold to %d", (int)estop_pid_threshold);
  MotorMessage mm;
  mm.setRegister(MotorMessage::REG_PID_MAX_ERROR);
  mm.setType(MotorMessage::TYPE_WRITE);
  mm.setData(estop_pid_threshold);
  motor_serial_->transmitCommand(mm);
}

// Setup the controller board to have estop button state detection feature enabled or not
void MotorInterface::set_estop_detection(int32_t estop_detection)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "MotorCommands"), "setting estop button detection to %x", (int)estop_detection);
  MotorMessage mm;
  mm.setRegister(MotorMessage::REG_ESTOP_ENABLE);
  mm.setType(MotorMessage::TYPE_WRITE);
  mm.setData(estop_detection);
  motor_serial_->transmitCommand(mm);
}

// Returns true if estop switch is active OR if motor power is off somehow off
bool MotorInterface::get_estop_state()
{
  return estop_motor_power_off_;
}

// Setup the controller board maximum settable motor forward speed
void MotorInterface::set_max_fwd_speed(int32_t max_speed_fwd)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "MotorCommands"), "setting max motor forward speed to %d", (int)max_speed_fwd);
  MotorMessage mm;
  mm.setRegister(MotorMessage::REG_MAX_SPEED_FWD);
  mm.setType(MotorMessage::TYPE_WRITE);
  mm.setData(max_speed_fwd);
  motor_serial_->transmitCommand(mm);
}

// Setup the Wheel Type. Overrides mode in use on hardware
// This used to only be standard but THIN_WHEELS were added in Jun 2020
void MotorInterface::set_wheel_type(int32_t new_wheel_type)
{
  MotorMessage ho;
  switch (new_wheel_type) {
    case MotorMessage::OPT_WHEEL_TYPE_STANDARD:
    case MotorMessage::OPT_WHEEL_TYPE_THIN:
      RCLCPP_INFO_ONCE(
        rclcpp::get_logger(
          "MotorCommands"), "setting MCB wheel type %d", (int)new_wheel_type);
      wheel_type_ = new_wheel_type;
      ho.setRegister(MotorMessage::REG_WHEEL_TYPE);
      ho.setType(MotorMessage::TYPE_WRITE);
      ho.setData(wheel_type_);
      motor_serial_->transmitCommand(ho);
      break;
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "MotorCommands"), "Illegal MCB wheel type 0x%x will not be set!", (int)new_wheel_type);
  }
}

// Setup the Wheel direction. Overrides mode in use on hardware
// This allows for customer to install wheels on cutom robots as they like
void MotorInterface::set_wheel_direction(int32_t wheel_direction)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "MotorCommands"), "setting MCB wheel direction to %d", (int)wheel_direction);
  MotorMessage ho;
  ho.setRegister(MotorMessage::REG_WHEEL_DIR);
  ho.setType(MotorMessage::TYPE_WRITE);
  ho.setData(wheel_direction);
  motor_serial_->transmitCommand(ho);
}

// Setup the controller board option switch register which comes from the I2C 8-bit IO chip on MCB
void MotorInterface::set_option_switch_reg(int32_t option_switch_bits)
{
  RCLCPP_INFO(
    rclcpp::get_logger("MotorCommands"), "setting MCB option switch register to 0x%x",
    (int)option_switch_bits);
  MotorMessage os;
  os.setRegister(MotorMessage::REG_OPTION_SWITCH);
  os.setType(MotorMessage::TYPE_WRITE);
  os.setData(option_switch_bits);
  motor_serial_->transmitCommand(os);
}

// Setup the controller board system event register or clear bits in the register
void MotorInterface::set_system_events(int32_t system_events)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "MotorCommands"), "setting MCB system event register to %d", (int)system_events);
  MotorMessage se;
  se.setRegister(MotorMessage::REG_SYSTEM_EVENTS);
  se.setType(MotorMessage::TYPE_WRITE);
  se.setData(system_events);
  motor_serial_->transmitCommand(se);
}

// Setup the controller board maximum settable motor reverse speed
void MotorInterface::set_max_rev_speed(int32_t max_speed_rev)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "MotorCommands"), "setting max motor reverse speed to %d", (int)max_speed_rev);
  MotorMessage mm;
  mm.setRegister(MotorMessage::REG_MAX_SPEED_REV);
  mm.setType(MotorMessage::TYPE_WRITE);
  mm.setData(max_speed_rev);
  motor_serial_->transmitCommand(mm);
}

// Setup the controller board maximum PWM level allowed for a motor
void MotorInterface::set_max_pwm(int32_t max_pwm)
{
  RCLCPP_INFO(rclcpp::get_logger("MotorCommands"), "setting max motor PWM to %x", (int)max_pwm);
  MotorMessage mm;
  mm.setRegister(MotorMessage::REG_MAX_PWM);
  mm.setType(MotorMessage::TYPE_WRITE);
  mm.setData(max_pwm);
  motor_serial_->transmitCommand(mm);
}

void MotorInterface::set_deadman_timer(int32_t deadman_timer)
{
  RCLCPP_ERROR(rclcpp::get_logger("MotorCommands"), "setting deadman to %d", (int)deadman_timer);
  MotorMessage mm;
  mm.setRegister(MotorMessage::REG_DEADMAN);
  mm.setType(MotorMessage::TYPE_WRITE);
  mm.setData(deadman_timer);
  motor_serial_->transmitCommand(mm);
}

void MotorInterface::set_deadzone_enable(int32_t deadzone_enable)
{
  RCLCPP_ERROR(
    rclcpp::get_logger(
      "MotorCommands"), "setting deadzone enable to %d", (int)deadzone_enable);
  MotorMessage mm;
  mm.setRegister(MotorMessage::REG_DEADZONE);
  mm.setType(MotorMessage::TYPE_WRITE);
  mm.setData(deadman_timer_);
  motor_serial_->transmitCommand(mm);
}

void MotorInterface::send_params()
{
  std::vector<MotorMessage> commands;

  // RCLCPP_ERROR(rclcpp::get_logger("MotorCommands"), "sending PID %d %d %d %d",
  //(int)p_value, (int)i_value, (int)d_value, (int)denominator_value);

  // Only send one register at a time to avoid overwhelming serial comms
  // SUPPORT NOTE!  Adjust modulo for total parameters in the cycle
  //                and be sure no duplicate modulos are used!
  int cycle = (send_pid_count_++) % num_firmware_params_;   // MUST BE THE TOTAL NUMBER IN THIS HANDLING

  if (cycle == 0) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "MotorCommands"), "Setting PidParam P to %d", firmware_params_.pid_proportional);
    // motor_diag_.fw_pid_proportional = firmware_params_.pid_proportional;
    MotorMessage p;
    p.setRegister(MotorMessage::REG_PARAM_P);
    p.setType(MotorMessage::TYPE_WRITE);
    p.setData(firmware_params_.pid_proportional);
    commands.push_back(p);
  }

  if (cycle == 1) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "MotorCommands"), "Setting PidParam I to %d", firmware_params_.pid_integral);
    // motor_diag_.fw_pid_integral = firmware_params_.pid_integral;
    MotorMessage i;
    i.setRegister(MotorMessage::REG_PARAM_I);
    i.setType(MotorMessage::TYPE_WRITE);
    i.setData(firmware_params_.pid_integral);
    commands.push_back(i);
  }

  if (cycle == 2) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "MotorCommands"), "Setting PidParam D to %d", firmware_params_.pid_derivative);
    // motor_diag_.fw_pid_derivative = firmware_params_.pid_derivative;
    MotorMessage d;
    d.setRegister(MotorMessage::REG_PARAM_D);
    d.setType(MotorMessage::TYPE_WRITE);
    d.setData(firmware_params_.pid_derivative);
    commands.push_back(d);
  }

  if (cycle == 3 && (firmware_version_ >= MIN_FW_PID_V_TERM)) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "MotorCommands"), "Setting PidParam V to %d", firmware_params_.pid_velocity);
    // motor_diag_.fw_pid_velocity = firmware_params_.pid_velocity;
    MotorMessage v;
    v.setRegister(MotorMessage::REG_PARAM_V);
    v.setType(MotorMessage::TYPE_WRITE);
    v.setData(firmware_params_.pid_velocity);
    commands.push_back(v);
  }

  if (cycle == 4) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "MotorCommands"), "Setting PidParam Denominator to %d", firmware_params_.pid_denominator);
    // motor_diag_.fw_pid_denominator = firmware_params_.pid_denominator;
    MotorMessage denominator;
    denominator.setRegister(MotorMessage::REG_PARAM_C);
    denominator.setType(MotorMessage::TYPE_WRITE);
    denominator.setData(firmware_params_.pid_denominator);
    commands.push_back(denominator);
  }

  if (cycle == 5) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "MotorCommands"), "Setting PidParam D window to %d",
      firmware_params_.pid_moving_buffer_size);
    // motor_diag_.fw_pid_moving_buffer_size = firmware_params_.pid_moving_buffer_size;
    MotorMessage winsize;
    winsize.setRegister(MotorMessage::REG_MOVING_BUF_SIZE);
    winsize.setType(MotorMessage::TYPE_WRITE);
    winsize.setData(firmware_params_.pid_moving_buffer_size);
    commands.push_back(winsize);
  }

  if (cycle == 6) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "MotorCommands"), "Setting PidParam max_pwm to %d", firmware_params_.max_pwm);
    // motor_diag_.fw_max_pwm = firmware_params_.max_pwm;
    MotorMessage maxpwm;
    maxpwm.setRegister(MotorMessage::REG_MAX_PWM);
    maxpwm.setType(MotorMessage::TYPE_WRITE);
    maxpwm.setData(firmware_params_.max_pwm);
    commands.push_back(maxpwm);
  }

  // SUPPORT NOTE!  Adjust max modulo for total parameters in the cycle, be sure no duplicates used!

  if (commands.size() != 0) {
    motor_serial_->transmitCommands(commands);
  }
}

// Get current battery voltage
float MotorInterface::get_battery_voltage()
{
  return battery_voltage_;
}

void MotorInterface::set_debug_leds(bool led_1, bool led_2)
{
  std::vector<MotorMessage> commands;

  MotorMessage led1;
  led1.setRegister(MotorMessage::REG_LED_1);
  led1.setType(MotorMessage::TYPE_WRITE);
  if (led_1) {
    led1.setData(0x00000001);
  } else {
    led1.setData(0x00000000);
  }
  commands.push_back(led1);

  MotorMessage led2;
  led2.setRegister(MotorMessage::REG_LED_2);
  led2.setType(MotorMessage::TYPE_WRITE);
  if (led_2) {
    led2.setData(0x00000001);
  } else {
    led2.setData(0x00000000);
  }
  commands.push_back(led2);

  motor_serial_->transmitCommands(commands);
}

double MotorInterface::calculate_radians_from_ticks(int16_t ticks) const
{
  return ticks * VELOCITY_READ_PER_SECOND / QTICKS_PER_RADIAN;
}

// write_speeds_in_radians()  Take in radians per sec for wheels and send in message to controller
//
// A direct write speeds that allows caller setting speeds in radians
// This interface allows maintaining of system speed in state but override to zero
// which is of value for such a case as ESTOP implementation
//
void MotorInterface::write_speeds_in_radians(double left_radians, double right_radians)
{
  MotorMessage both;
  both.setRegister(MotorMessage::REG_BOTH_SPEED_SET);
  both.setType(MotorMessage::TYPE_WRITE);

  int16_t left_speed = calculate_speed_from_radians(left_radians);
  int16_t right_speed = calculate_speed_from_radians(right_radians);

  // The masking with 0x0000ffff is necessary for handling -ve numbers
  int32_t data = (left_speed << 16) | (right_speed & 0x0000ffff);
  both.setData(data);

  // std_msgs::msg::Int32 smsg;
  // smsg.data = left_speed;

  motor_serial_->transmitCommand(both);

  // RCLCPP_ERROR(rclcpp::get_logger("MotorCommands"), "velocity_command %f rad/s %f rad/s",
  // joints_[WheelJointLocation::Left].velocity_command, joints_[WheelJointLocation::Right].velocity_command);
  // joints_[LEFT_WHEEL_JOINT].velocity_command, joints_[RIGHT_WHEEL_JOINT].velocity_command);
  // RCLCPP_ERROR(rclcpp::get_logger("MotorCommands"), "SPEEDS %d %d", left.getData(), right.getData());
}

// calculate the binary speed value sent to motor controller board
// using an input expressed in radians.
// The firmware uses the same speed value no matter what type of encoder is used
int16_t MotorInterface::calculate_speed_from_radians(double radians) const
{
  int16_t speed;
  double encoderFactor = 1.0;

  // The firmware accepts same units for speed value
  // and will deal with it properly depending on encoder handling in use
  if (firmware_params_.hw_options & MotorMessage::OPT_ENC_6_STATE) {
    encoderFactor = 0.5;
  }

  speed = boost::math::iround(
    encoderFactor * (radians * QTICKS_PER_RADIAN /
    VELOCITY_READ_PER_SECOND));
  return speed;
}

}  // namespace ubiquity_motor
