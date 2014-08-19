/*
 * Copyright 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <iostream>
#include <string>
#include "AtlasControlTypes.h"
#include "AtlasVectorTypes.h"
#include "AtlasInterfaceTypes.h"
#include "AtlasInterface.h"

AtlasInterface::AtlasInterface()
{
}

AtlasInterface::~AtlasInterface()
{
}

AtlasErrorCode AtlasInterface::get_instance(AtlasInterface *& iface)
{
}

AtlasErrorCode AtlasInterface::delete_instance()
{
}

AtlasErrorCode AtlasInterface::open_net_connection_to_robot(
  std::string robot_ip_address, int send_port, int recv_port)
{
}

AtlasErrorCode AtlasInterface::close_net_connection_to_robot()
{
}

bool AtlasInterface::net_connection_open()
{
}

AtlasErrorCode AtlasInterface::get_robot_ip_address(std::string& robot_ip_address)
{
}

AtlasErrorCode AtlasInterface::start(
  AtlasHydraulicPressureSetting desired_pressure, int64_t* packet_seq_id)
{
}

AtlasErrorCode AtlasInterface::stop(int64_t* packet_seq_id)
{
}

bool AtlasInterface::command_in_progress()
{
}

AtlasErrorCode AtlasInterface::set_desired_behavior(
  AtlasRobotBehavior behavior, int64_t* packet_seq_id)
{
}

AtlasErrorCode AtlasInterface::get_desired_behavior(
  AtlasRobotBehavior& desired_behavior)
{
}

AtlasErrorCode AtlasInterface::send_control_data_to_robot(
  AtlasControlDataToRobot& data_to_robot, int64_t* packet_seq_id)
{
}

AtlasErrorCode AtlasInterface::send_ext_data_to_robot(
  AtlasExtendedDataToRobot& ext_data_to_robot, int64_t* packet_seq_id)
{
}

AtlasErrorCode AtlasInterface::new_control_data_from_robot_available(
  double timeout, bool* new_data_available)
{
}

AtlasErrorCode AtlasInterface::get_control_data_from_robot(
  AtlasControlDataFromRobot* data_from_robot)
{
}

AtlasErrorCode AtlasInterface::set_desired_pump_rpm(uint16_t desired_pump_rpm)
{
}

AtlasErrorCode AtlasInterface::download_robot_log_file(
  std::string dest_directory, float duration, std::string filename)
{
}

AtlasErrorCode AtlasInterface::calibrate(int64_t* packet_seq_id)
{
}

AtlasErrorCode AtlasInterface::force_imu_realignment(int64_t* packet_seq_id)
{
}

std::string AtlasInterface::get_error_code_text(AtlasErrorCode ec) const
{
}

std::string AtlasInterface::get_run_state_as_string(
  AtlasRobotRunState run_state) const
{
}

std::string AtlasInterface::get_status_flag_as_string(
  AtlasRobotStatus robot_status_flag) const
{
}

std::string AtlasInterface::link_name_from_link_id(AtlasLinkId link_id) const
{
}

AtlasLinkId AtlasInterface::link_id_from_link_name(std::string link_name) const
{
}

std::string AtlasInterface::joint_name_from_joint_id(AtlasJointId joint_id) const
{
}

AtlasJointId AtlasInterface::joint_id_from_joint_name(
  std::string joint_name) const
{
}

std::string AtlasInterface::behavior_name_from_behavior(
  AtlasRobotBehavior behavior) const
{
}

AtlasRobotBehavior AtlasInterface::behavior_from_behavior_name(
  std::string behavior_name) const
{
}

AtlasErrorCode AtlasInterface::clear_faults(int64_t* packet_seq_id)
{
}
