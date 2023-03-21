/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad
 */

#ifndef KUKA_RSI_HARDWARE_INTERFACE_KUKA_HARDWARE_INTERFACE_
#define KUKA_RSI_HARDWARE_INTERFACE_KUKA_HARDWARE_INTERFACE_

// STL
#include <vector>
#include <string>
#include <map>
#include <bitset>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>


// ros_control
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Timers
#include <chrono>

// UDP server
#include <kuka_rsi_hw_interface/udp_server.h>

// RSI
#include <kuka_rsi_hw_interface/rsi_state.h>
#include <kuka_rsi_hw_interface/rsi_command.h>

//SRV
#include <kuka_rsi_hw_interface/write_output_bool.h>
#include <kuka_rsi_hw_interface/write_output_bool_array.h>
#include <kuka_rsi_hw_interface/write_output_bool_array_2outs.h>
#include <kuka_rsi_hw_interface/write_output_bool_array_all_outs.h>

//MSG
#include <kuka_rsi_hw_interface/uint16_t_array.h>
#include <kuka_rsi_hw_interface/input_data.h>


namespace kuka_rsi_hw_interface
{

static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;

class KukaHardwareInterface : public hardware_interface::RobotHW
{

private:

  // ROS node handle
  ros::NodeHandle nh_;

  int n_dof_;
  std::string test_type_IN_;
  std::string test_type_OUT_;
  std::vector<std::string> servo_param_names_;
  std::map<std::string, double> servo_params_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;

  std::vector<bool> digital_output_bit_;
  std::vector<uint16_t> digital_output_;
  std::vector<bool> digital_input_bit_;
  std::vector<uint16_t> digital_input_;

  int32_t deltaTargetPos_PUU_;
  double deltaTargetPos_m_;
  int32_t deltaTargetVel_RPM_;
  double deltaTargetVel_mps_;
  int16_t deltaTargetTor_;
  float deltaTargetTor_Nm_;
  uint8_t deltaOpMode_;

  int32_t deltaActualPos_PUU_;
  double deltaActualPos_m_;
  int32_t deltaActualVel_RPM_;
  double deltaActualVel_mps_;
  int16_t deltaActualTor_;
  float deltaActualTor_Nm_;
  uint8_t deltaOpModeDisp_;


  // Constants for unit conversion
  double const_PUU2m_;      // conversion factor [PUU -> m]
  double const_rpm2mps_;    // conversion factor [RPM/10 -> m/s]
  float const_torque2Nm_;   // conversion factor [rated_torque[Nm]/1000 -> Nm]


  // RSI
  RSIState rsi_state_;
  RSICommand rsi_command_;
  std::vector<double> rsi_initial_joint_positions_;
  std::vector<double> rsi_joint_position_corrections_;
  unsigned long long ipoc_;

  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > rt_rsi_recv_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > rt_rsi_send_;

  std::unique_ptr<UDPServer> server_;
  std::string local_host_;
  int local_port_;
  std::string remote_host_;
  std::string remote_port_;
  std::string in_buffer_;
  std::string out_buffer_;

  // Timing
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  // Internal method to read params and to compute conversion constants
  bool read_params();
  bool compute_conversion_constants();

public:

  KukaHardwareInterface();
  ~KukaHardwareInterface();

  void start();
  void configure();
  bool read(const ros::Time time, const ros::Duration period);
  bool write(const ros::Time time, const ros::Duration period);

  bool write_digital_output(kuka_rsi_hw_interface::write_output_bool::Request &req, kuka_rsi_hw_interface::write_output_bool::Response &res);
  bool write_digital_output_array(kuka_rsi_hw_interface::write_output_bool_array::Request &req, kuka_rsi_hw_interface::write_output_bool_array::Response &res);
  bool write_digital_output_array_2outs(kuka_rsi_hw_interface::write_output_bool_array_2outs::Request &req, kuka_rsi_hw_interface::write_output_bool_array_2outs::Response &res);
  bool write_digital_output_array_all_outs(kuka_rsi_hw_interface::write_output_bool_array_all_outs::Request &req, kuka_rsi_hw_interface::write_output_bool_array_all_outs::Response &res);

  // Getter methods
  std::vector<uint16_t> digital_input() const;
  uint32_t deltaActualPos_PUU() const;
  double deltaActualPos_m() const;
  int32_t deltaActualVel_RPM() const;
  double deltaActualVel_mps() const;
  int16_t deltaActualTor() const;
  float deltaActualTor_Nm() const;
};

} // namespace kuka_rsi_hw_interface

#endif
