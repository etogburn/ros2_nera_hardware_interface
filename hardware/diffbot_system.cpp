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

#include "ros2_nera_hardware_interface/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_nera_hardware_interface
{
hardware_interface::CallbackReturn NeraHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.imu_sensor_name= info_.hardware_parameters["imu_sensor_name"];
  cfg_.loop_rate = std::stoi(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.max_gyro_radps = std::stod(info_.hardware_parameters["max_gyro_radps"]);
  cfg_.max_accel_mps = std::stod(info_.hardware_parameters["max_accel_mps"]);
  // cfg_.gyro_offset[X_IDX] = std::stod(info_.hardware_parameters["gyro_offset_x"]);
  // cfg_.gyro_offset[Y_IDX] = std::stod(info_.hardware_parameters["gyro_offset_y"]);
  // cfg_.gyro_offset[Z_IDX] = std::stod(info_.hardware_parameters["gyro_offset_z"]);
  
  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  imu_.setup(cfg_.imu_sensor_name, cfg_.max_gyro_radps, cfg_.max_accel_mps, cfg_.loop_rate);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NeraHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NeraHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NeraHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NeraHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NeraHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NeraHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  for(int i = 0; i < 10; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      imu_.name, imu_.imuStateInterface[i], &imu_.imuStateVals[i]));
  }

  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, imu_.accelName[Y_IDX], &imu_.accelmps[Y_IDX]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, imu_.accelName[Z_IDX], &imu_.accelmps[Z_IDX]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, imu_.gyroName[X_IDX], &imu_.gyroRadps[X_IDX]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, imu_.gyroName[Y_IDX], &imu_.gyroRadps[Y_IDX]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, imu_.gyroName[Z_IDX], &imu_.gyroRadps[Z_IDX]));

  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "orientation.x", &imu_.mag[0]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "orientation.y", &imu_.mag[1]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "orientation.z", &imu_.mag[2]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   imu_.name, "orientation.w", &imu_.mag[3]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NeraHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn NeraHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NeraHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  const int del = 1000000/cfg_.loop_rate;

  do {
      //std::this_thread::sleep_for(std::chrono::milliseconds(del));
      comms_.get_accel_values(imu_.rawAccel[0], imu_.rawAccel[1], imu_.rawAccel[2]);
      comms_.get_gyro_values(imu_.rawGyro[0], imu_.rawGyro[1], imu_.rawGyro[2]);
      usleep(del);
  } while(!imu_.calibrate());

  std::cout << imu_.gyroOffset[0] << ", " << imu_.gyroOffset[1] << ", " << imu_.gyroOffset[2] <<std::endl;

  RCLCPP_INFO(rclcpp::get_logger("NeraHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NeraHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NeraHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }

  RCLCPP_INFO(rclcpp::get_logger("NeraHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn NeraHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NeraHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  comms_.send_wake_msg();

  RCLCPP_INFO(rclcpp::get_logger("NeraHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NeraHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("NeraHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("NeraHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type NeraHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.get_encoder_values(wheel_l_.enc, wheel_r_.enc);
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_r_.pos = wheel_r_.calc_enc_angle();

  comms_.get_speed_values(wheel_l_.speed, wheel_r_.speed);
  wheel_l_.vel = wheel_l_.calc_speed();
  wheel_r_.vel = wheel_r_.calc_speed();

  comms_.get_accel_values(imu_.rawAccel[0], imu_.rawAccel[1], imu_.rawAccel[2]);
  comms_.get_gyro_values(imu_.rawGyro[0], imu_.rawGyro[1], imu_.rawGyro[2]);

  imu_.calc();

  //std::cout << imu_.rawAccel[0] << " " << imu_.rawAccel[1] << " " << imu_.rawAccel[2] << std::endl;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_nera_hardware_interface ::NeraHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int16_t motor_l_rpm = wheel_l_.get_wheel_rpm();
  int16_t motor_r_rpm = wheel_r_.get_wheel_rpm();
  comms_.set_motor_values(motor_l_rpm, motor_r_rpm);
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_nera_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_nera_hardware_interface::NeraHardware, hardware_interface::SystemInterface)
