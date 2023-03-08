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
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>

#include <stdexcept>

#define _USE_MATH_DEFINES
#include <cmath>

//#ifndef M_PI
//#define M_PI (3.14159265358979323846)
//#endif

#define DEFAULT_N_DOF 6
#define N_DIGOUT 16

namespace kuka_rsi_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface() :
    joint_position_(DEFAULT_N_DOF, 0.0), joint_velocity_(DEFAULT_N_DOF, 0.0), joint_effort_(DEFAULT_N_DOF, 0.0),
    joint_position_command_(DEFAULT_N_DOF, 0.0), joint_velocity_command_(DEFAULT_N_DOF, 0.0), joint_effort_command_(DEFAULT_N_DOF, 0.0),
    joint_names_(DEFAULT_N_DOF), rsi_initial_joint_positions_(DEFAULT_N_DOF, 0.0), rsi_joint_position_corrections_(DEFAULT_N_DOF, 0.0),
    ipoc_(0), n_dof_(DEFAULT_N_DOF), digital_output_bit_(1, false), digital_output_(3, 0), digital_input_bit_(2, false),
    digital_input_(3, 0), deltaTargetPos_PUU_(0), deltaTargetPos_mm_(0.0), deltaActualPos_PUU_(0), deltaActualPos_mm_(0.0)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  // Read params from parameter server (yaml file)
  read_params();

  // Compute electronic and mechanical gear ratio
  double tau_pinionrack = M_PI * servo_params_.at("pinion_diameter");
  double tau_mechanical = tau_pinionrack * 1.0 / servo_params_.at("tau_gearbox");
  double electronic_resolution = servo_params_.at("encoder_pulse_number") * servo_params_.at("encoder_resolution_multiplier");
  double electronic_gear_ratio = servo_params_.at("electronic_gear_ratio_feed_constant") / servo_params_.at("electronic_gear_ratio_numerator");
  double n_PUU_per_rev = electronic_resolution * electronic_gear_ratio;

  const_PUU2mm_ = tau_mechanical * 1.0 / n_PUU_per_rev;

  //Create ros_control interface std::string command_type
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i],
                                             &joint_effort_[i]));

    // Create joint position control interface
    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                        &joint_position_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded kuka_rsi_hardware_interface");

  // Publish digital input to ROS topic for debugging
  if (not test_type_IN_.compare("one_bit"))
  {
    digital_input_pub_ = nh_.advertise<std_msgs::Bool>("DIGITAL_INPUT", 100);
  }
  else if (not test_type_IN_.compare("array_uint16"))
  {
    digital_input_pub_ = nh_.advertise<std_msgs::UInt16>("DIGITAL_INPUT", 100);
  }
  else if (not test_type_IN_.compare("array_uint16_2ins"))
  {
    digital_input_pub_ = nh_.advertise<kuka_rsi_hw_interface::uint16_t_array>("DIGITAL_INPUT", 100);
  }
  else // (not test_type_IN_.compare("none"))
  {

  }

}

KukaHardwareInterface::~KukaHardwareInterface()
{

}

bool KukaHardwareInterface::read_params()
{
  // Read params of the DELTA Servo Drive
  servo_param_names_ = {"tau_gearbox","pinion_diameter","encoder_pulse_number","encoder_resolution_multiplier",
                        "max_motor_speed","electronic_gear_ratio_feed_constant","electronic_gear_ratio_numerator"};

  double value(0);
  std::string current_param;
  for (int i = 0; i < servo_param_names_.size(); ++i)
  {
    current_param = servo_param_names_[i];

    if (!nh_.getParam("delta/" + current_param, value))
    {
      ROS_ERROR_STREAM("Cannot find required parameter '" << current_param << "' "
                       "on the parameter server.");
    }
    servo_params_.insert({current_param, value});

    ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Servo - " << current_param << ": " << value);
  }

  // Read robot joint names
  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
              "on the parameter server.");
  }

  // Read robot number of DoF
  if (!nh_.getParam("rsi/n_dof", n_dof_))
  {
    ROS_ERROR("Cannot find required parameter 'n_dof' "
              "on the parameter server.");
  }
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "DOF: " << n_dof_);

  // Read test type (inputs)
  if (!nh_.getParam("rsi/test_type_IN", test_type_IN_))
  {
    ROS_ERROR("Cannot find required parameter 'rsi/test_type_IN' "
              "on the parameter server.");
  }
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Test type IN: " << test_type_IN_);

  // Read test type (outputs)
  if (!nh_.getParam("rsi/test_type_OUT", test_type_OUT_))
  {
    ROS_ERROR("Cannot find required parameter 'rsi/test_type_OUT' "
              "on the parameter server.");
  }
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Test type OUT: " << test_type_OUT_);

  return true;
}

bool KukaHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
  in_buffer_.resize(1024);
  if (server_->recv(in_buffer_) == 0)
  {
    return false;
  }
//ROS_WARN("here1");
  if (rt_rsi_recv_->trylock()){
    rt_rsi_recv_->msg_.data = in_buffer_;
    rt_rsi_recv_->unlockAndPublish();
  }
//ROS_WARN("here2");

  rsi_state_ = RSIState(in_buffer_, test_type_IN_, servo_params_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
  }

  ipoc_ = rsi_state_.ipoc;

  // Read and publish the value of digital input
  std_msgs::Empty msg;
  if (not test_type_IN_.compare("one_bit"))
  {
    // std_msgs::Bool msg;
    // msg.data = rsi_state_.digital_input_bit_1;
    digital_input_bit_[0] = rsi_state_.digital_input_bit_1;
    digital_input_bit_[1] = rsi_state_.digital_input_bit_2;
    std::cout << "( " << digital_input_bit_[0] << ", " << digital_input_bit_[1] <<std::endl;
  }
  else if (not test_type_IN_.compare("array_uint16"))
  {
    // std_msgs::UInt16 msg;
    // msg.data = rsi_state_.digital_input_beckhoff;
    digital_input_[0] = rsi_state_.digital_input_beckhoff;
    // std::cout << digital_input_[0] << std::endl;
  }
  else if (not test_type_IN_.compare("array_uint16_2ins"))
  {
    digital_input_[0] = rsi_state_.digital_input_beckhoff;
    std::cout << "Beckhoff: " << digital_input_[0] << std::endl;
    digital_input_[1] = rsi_state_.digital_input_odot;
    std::cout << "Odot: " << digital_input_[1] << std::endl;
  }
  else if (not test_type_IN_.compare("array_uint16_all_ins"))
  {
    digital_input_[0] = rsi_state_.digital_input_beckhoff;
    std::cout << "Beckhoff: " << digital_input_[0] << std::endl;

    digital_input_[1] = rsi_state_.digital_input_odot;
    std::cout << "Odot: " << digital_input_[1] << std::endl;

    deltaActualPos_PUU_ = rsi_state_.digital_input_deltaActualPos;
    deltaActualPos_mm_ = const_PUU2mm_ * deltaActualPos_PUU_;
    std::cout << "Delta Actual Pos: " << deltaActualPos_PUU_ << " [PUU] | " << deltaActualPos_mm_ << " [mm]" << std::endl;
  }
  else // (not test_type_IN_.compare("none"))
  {

  }

  // digital_input_pub_.publish(msg);

  return true;
}

bool KukaHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
  out_buffer_.resize(1024);

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }

  out_buffer_ = RSICommand(rsi_joint_position_corrections_, digital_output_bit_, digital_output_, deltaTargetPos_PUU_, ipoc_, test_type_OUT_).xml_doc;
  server_->send(out_buffer_);

//  ROS_WARN_ONCE("Out buffer = %s",out_buffer_.c_str());

  if(rt_rsi_send_->trylock()) {
    rt_rsi_send_->msg_.data = out_buffer_;
    rt_rsi_send_->unlockAndPublish();
  }
  return true;
}


bool KukaHardwareInterface::write_digital_output(kuka_rsi_hw_interface::write_output_bool::Request &req, kuka_rsi_hw_interface::write_output_bool::Response &res)
{
  digital_output_bit_.clear();
  digital_output_bit_.push_back(req.out1);
  std::cout << "Command sent: " << out_buffer_.c_str() << std::endl;
  return true;
}


bool KukaHardwareInterface::write_digital_output_array(kuka_rsi_hw_interface::write_output_bool_array::Request &req, kuka_rsi_hw_interface::write_output_bool_array::Response &res)
{
  digital_output_[0] = req.out;
  std::cout << "Command sent: " << out_buffer_.c_str() << std::endl;
  return true;
}


bool KukaHardwareInterface::write_digital_output_array_2outs(kuka_rsi_hw_interface::write_output_bool_array_2outs::Request &req, kuka_rsi_hw_interface::write_output_bool_array_2outs::Response &res)
{
  digital_output_[0] = req.out1;
  digital_output_[1] = req.out2;
  std::cout << "Command sent: " << out_buffer_.c_str() << std::endl;
  return true;
}


bool KukaHardwareInterface::write_digital_output_array_all_outs(kuka_rsi_hw_interface::write_output_bool_array_all_outs::Request &req, kuka_rsi_hw_interface::write_output_bool_array_all_outs::Response &res)
{
  digital_output_[0] = req.out1;
  digital_output_[1] = req.out2;
  digital_output_[2] = req.out3;
  deltaTargetPos_mm_ = req.out4;
  deltaTargetPos_PUU_ = 1.0 / const_PUU2mm_ * deltaTargetPos_mm_;
  std::cout << "Command sent: " << out_buffer_.c_str() << std::endl;
  return true;
}


void KukaHardwareInterface::start()
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!");

  int bytes = server_->recv(in_buffer_);
  ROS_WARN_STREAM("in_buffer_n bytes received: " << bytes);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100)
  {
    bytes = server_->recv(in_buffer_);
  }

  ROS_WARN_STREAM("State buffer: " << in_buffer_);

  ROS_WARN_ONCE("Reading state in start function...");
  rsi_state_ = RSIState(in_buffer_, test_type_IN_, servo_params_);
  ROS_WARN_ONCE("State read in start function.");

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    joint_position_command_[i] = joint_position_[i];
    rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
  }

  ipoc_ = rsi_state_.ipoc;

  ROS_WARN_ONCE("Sending command in start function...");
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, digital_output_bit_, digital_output_, deltaTargetPos_PUU_, ipoc_, test_type_OUT_).xml_doc;
  ROS_WARN_ONCE("Command sent in start function.");

  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");

}

void KukaHardwareInterface::configure()
{
  const std::string param_addr = "rsi/listen_address";
  const std::string param_port = "rsi/listen_port";

  if (nh_.getParam(param_addr, local_host_) && nh_.getParam(param_port, local_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_hardware_interface",
                          "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")");
  }
  else
  {
    std::string msg = "Failed to get RSI listen address or listen port from"
    " parameter server (looking for '" + param_addr + "' and '" + param_port + "')";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }

  rt_rsi_recv_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc_recv", 3));
  rt_rsi_send_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc_send", 3));
}

} // namespace kuka_rsi_hardware_interface
