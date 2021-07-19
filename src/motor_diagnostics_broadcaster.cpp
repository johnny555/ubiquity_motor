/**
Copyright (c) 2021 Dyno Robotics
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

/*
 * Author: Samuel Lindgren
 */

#include "ubiquity_motor/motor_diagnostics_broadcaster.hpp"

#include <memory>
#include <string>

namespace ubiquity_motor {

MotorDiagnosticsBroadcaster::MotorDiagnosticsBroadcaster()
    : controller_interface::ControllerInterface() {}

controller_interface::return_type MotorDiagnosticsBroadcaster::init(
    const std::string& controller_name) {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        return ret;
    }

    // TODO(sam): Add diagnostics_updater

    try {
        node_->declare_parameter<float>("battery_voltage_low_level", 23.2);
        node_->declare_parameter<float>("battery_voltage_critical", 22.5);
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

CallbackReturn MotorDiagnosticsBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    try {
        // register battery state publisher
        battery_state_publisher_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(
            "~/battery", rclcpp::SystemDefaultsQoS());
        realtime_publisher_ = std::make_unique<StatePublisher>(battery_state_publisher_);
    } catch (const std::exception& e) {
        fprintf(
            stderr,
            "Exception thrown during publisher creation at configure stage with message : %s \n",
            e.what());
        return CallbackReturn::ERROR;
    }

    realtime_publisher_->lock();
    realtime_publisher_->msg_.current = std::numeric_limits<float>::quiet_NaN();
    realtime_publisher_->msg_.charge = std::numeric_limits<float>::quiet_NaN();
    realtime_publisher_->msg_.capacity = std::numeric_limits<float>::quiet_NaN();
    realtime_publisher_->msg_.design_capacity = std::numeric_limits<float>::quiet_NaN();
    realtime_publisher_->msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    realtime_publisher_->msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    realtime_publisher_->msg_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    realtime_publisher_->unlock();

    RCLCPP_DEBUG(node_->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotorDiagnosticsBroadcaster::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
MotorDiagnosticsBroadcaster::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = {"magni_IOs/battery_voltage"};
    return state_interfaces_config;
}

CallbackReturn MotorDiagnosticsBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    for (auto i = 0u; i < state_interfaces_.size(); i++) {
        auto full_name = state_interfaces_[i].get_full_name();
        RCLCPP_INFO(node_->get_logger(), "Found state intefrace with name: %s", full_name.c_str());
        if (full_name == "magni_IOs/battery_voltage") {
            battery_voltage_interface_index_ = i;
        }
    }

    // NOTE(sam): should I copy referances to new vector and only access from that vector?
    // They seem to do that in the semantic interface:
    // https://github.com/ros-controls/ros2_control/blob/master/controller_interface/include/semantic_components/semantic_component_interface.hpp
    return CallbackReturn::SUCCESS;
}

CallbackReturn MotorDiagnosticsBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    // NOTE(sam): should release the interfaces here if "loaned" in on_activate like in the semantic
    // interface Maybe "loaning" is overkill when they are read-only anyways? TBD...

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type MotorDiagnosticsBroadcaster::update() {
    auto battery_voltage = state_interfaces_[battery_voltage_interface_index_].get_value();

    RCLCPP_DEBUG(node_->get_logger(), "battery_voltage: %0.5f", battery_voltage);

    if (realtime_publisher_ && realtime_publisher_->trylock()) {
        realtime_publisher_->msg_.header.stamp = node_->now();

        realtime_publisher_->msg_.voltage = battery_voltage;
        realtime_publisher_->msg_.percentage =
            std::max(0.0, std::min(1.0, (battery_voltage - 20.0) * 0.125));

        // TODO(sam): Update diagnostics
        // motor_diag_.battery_voltage = realtime_publisher_->msg_.voltage;
        // motor_diag_.battery_voltage_low_level =
        // MotorHardware::fw_params.battery_voltage_low_level; motor_diag_.battery_voltage_critical
        // = MotorHardware::fw_params.battery_voltage_critical;

        realtime_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

}  // namespace ubiquity_motor

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ubiquity_motor::MotorDiagnosticsBroadcaster,
                       controller_interface::ControllerInterface)