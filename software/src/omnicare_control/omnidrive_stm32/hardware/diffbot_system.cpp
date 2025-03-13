// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "omnidrive_stm32/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omnidrive_stm32
{
  hardware_interface::CallbackReturn OmniDriveSTM32::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    //
    // Parâmetros gerais do robô
    //
    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.back_wheel_name = info_.hardware_parameters["back_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    //
    // Setup das rodas
    //

    wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
    wheel_b_.setup(cfg_.back_wheel_name, cfg_.enc_counts_per_rev);

    //
    // Não sei o q faz
    //

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("OmniDriveSTM32"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("OmniDriveSTM32"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("OmniDriveSTM32"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("OmniDriveSTM32"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("OmniDriveSTM32"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  //
  // As interfaces entre o MCU -> ros2_control (Estado - Posição e Velocidade)
  //
  std::vector<hardware_interface::StateInterface> OmniDriveSTM32::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    //
    // Left wheel
    //
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

    //
    // Right wheel
    //
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

    //
    // Back wheel
    //
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_b_.name, hardware_interface::HW_IF_POSITION, &wheel_b_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_b_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_.vel));

    return state_interfaces;
  }

  //
  // As interfaces entre o MCU <- ros2_control (Comando - Mandando Velocidade para o MCU)
  //

  std::vector<hardware_interface::CommandInterface> OmniDriveSTM32::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    //
    // Left wheel
    //
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

    //
    // Right wheel
    //
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

    //
    // Back wheel
    //
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_b_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn OmniDriveSTM32::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveSTM32"), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveSTM32"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn OmniDriveSTM32::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveSTM32"), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveSTM32"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn OmniDriveSTM32::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveSTM32"), "Activating ...please wait...");
    if (!comms_.connected())
      comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveSTM32"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn OmniDriveSTM32::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveSTM32"), "Deactivating ...please wait...");
    comms_.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("OmniDriveSTM32"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }
  // Read from serial (USB COM)
  hardware_interface::return_type OmniDriveSTM32::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc,wheel_b_.enc);

    //
    // Calculando as velocidades ((final - inicial) / tempo (m/s))
    //
    double delta_seconds = period.seconds();

    //
    // Left wheel
    //
    double pos_prev = wheel_l_.pos;
    wheel_l_.pos = wheel_l_.calc_enc_angle();
    wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

    //
    // Right wheel
    //
    pos_prev = wheel_r_.pos;
    wheel_r_.pos = wheel_r_.calc_enc_angle();
    wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

    //
    // Back wheel
    //
    pos_prev = wheel_b_.pos;
    wheel_b_.pos = wheel_b_.calc_enc_angle();
    wheel_b_.vel = (wheel_b_.pos - pos_prev) / delta_seconds;

    return hardware_interface::return_type::OK;
  }

  //
  // Write on MCU
  //

  hardware_interface::return_type omnidrive_stm32 ::OmniDriveSTM32::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }
    float radius = 0.140;

    // // 
    // // V = Omega * R (m/s) --> V = wheel_l_.cmd (rad/s) * radius (m)
    // // 
    // float motor_l_meters_per_second = wheel_l_.cmd * radius;
    // float motor_r_meters_per_second = wheel_r_.cmd * radius;
    // float motor_b_meters_per_second = wheel_b_.cmd * radius;

    // Counters per loop
    int motor_l_counts_per_second = wheel_l_.cmd / wheel_l_.rads_per_count;
    int motor_r_counts_per_second = wheel_r_.cmd / wheel_r_.rads_per_count;
    int motor_b_counts_per_second = wheel_b_.cmd / wheel_b_.rads_per_count;
    
    // motor_l_counts_per_second = 6000;
    // motor_r_counts_per_second = 0;
    // motor_b_counts_per_second = 6000;


    comms_.set_motor_values(motor_l_counts_per_second,motor_r_counts_per_second,motor_b_counts_per_second);
    return hardware_interface::return_type::OK;
  }

} // namespace omnidrive_stm32

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    omnidrive_stm32::OmniDriveSTM32, hardware_interface::SystemInterface)
