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

#define DEFAULT_N_DOF 6
#define N_DIGOUT 16

namespace kuka_rsi_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface() :
    joint_position_(DEFAULT_N_DOF, 0.0), joint_velocity_(DEFAULT_N_DOF, 0.0), joint_effort_(DEFAULT_N_DOF, 0.0),
    joint_position_command_(DEFAULT_N_DOF, 0.0), joint_velocity_command_(DEFAULT_N_DOF, 0.0), joint_effort_command_(DEFAULT_N_DOF, 0.0),
    joint_names_(DEFAULT_N_DOF), rsi_initial_joint_positions_(DEFAULT_N_DOF, 0.0), rsi_joint_position_corrections_(DEFAULT_N_DOF, 0.0),
    ipoc_(0), n_dof_(DEFAULT_N_DOF), digital_output_bit_(1, false), digital_output_(0), digital_input_(0)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  nh_.param("rsi/n_dof", n_dof_, DEFAULT_N_DOF);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "DOF: " << n_dof_);

  if (!nh_.getParam("rsi/test_type_IN_", test_type_IN_))
  {
    ROS_ERROR("Cannot find required parameter 'rsi/test_type_IN_' "
      "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
      "'rsi/test_type_IN_' on the parameter server.");
  }
  if (!nh_.getParam("rsi/test_type_OUT_", test_type_OUT_))
  {
    ROS_ERROR("Cannot find required parameter 'rsi/test_type_OUT_' "
      "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
      "'rsi/test_type_OUT_' on the parameter server.");
  }

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Test type IN: " << test_type_IN_);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Test type OUT: " << test_type_OUT_);

  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
      "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
      "'controller_joint_names' on the parameter server.");
  }

  //Create ros_control interfacesstd::string command_type
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
  digital_input_pub_ = nh_.advertise<std_msgs::UInt16>("DIGITAL_INPUT", 100);

}

KukaHardwareInterface::~KukaHardwareInterface()
{

}

bool KukaHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
  in_buffer_.resize(1024);
  if (server_->recv(in_buffer_) == 0)
  {
    return false;
  }

  if (rt_rsi_recv_->trylock()){
    rt_rsi_recv_->msg_.data = in_buffer_;
    rt_rsi_recv_->unlockAndPublish();
  }

  rsi_state_ = RSIState(in_buffer_, test_type_IN_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
  }

  ipoc_ = rsi_state_.ipoc;
  digital_input_ = rsi_state_.digital_input;

  // Read and publish the value of digital input
  std_msgs::UInt16 msg;
  msg.data = digital_input_;
  digital_input_pub_.publish(msg);

  return true;
}

bool KukaHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
  out_buffer_.resize(1024);

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }

  out_buffer_ = RSICommand(rsi_joint_position_corrections_, digital_output_bit_, digital_output_, ipoc_, test_type_OUT_).xml_doc;
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
  digital_output_ = req.out;
  std::cout << "Command sent: " << out_buffer_.c_str() << std::endl;
  return true;
}


void KukaHardwareInterface::start()
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!");

  int bytes = server_->recv(in_buffer_);
  bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100)
  {
    bytes = server_->recv(in_buffer_);
  }

  ROS_WARN_ONCE("State buffer: ");
  std::cout << in_buffer_ << std::endl;

  ROS_WARN_ONCE("Reading state in start function...");
  rsi_state_ = RSIState(in_buffer_, test_type_IN_);
  ROS_WARN_ONCE("State read in start function.");

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    joint_position_command_[i] = joint_position_[i];
    rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
  }

  ipoc_ = rsi_state_.ipoc;
  digital_input_ = rsi_state_.digital_input;

  ROS_WARN_ONCE("Sending command in start function...");
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, digital_output_bit_, digital_output_, ipoc_, test_type_OUT_).xml_doc;
  ROS_WARN_ONCE("Command sent in start function.");

  server_->send(out_buffer_);
  // Set receive timeout to 1 second9
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
