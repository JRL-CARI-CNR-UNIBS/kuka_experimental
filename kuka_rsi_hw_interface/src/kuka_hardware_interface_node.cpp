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


int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("hardware_interface", "Starting hardware interface...");

  ros::init(argc, argv, "kuka_rsi_hardware_interface");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;

  kuka_rsi_hw_interface::KukaHardwareInterface kuka_rsi_hw_interface;
  kuka_rsi_hw_interface.configure();

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  // Services to write digital output
  ros::ServiceServer server = nh.advertiseService(ros::names::append(ros::this_node::getName(),"/write_digital_output"), &kuka_rsi_hw_interface::KukaHardwareInterface::write_digital_output, &kuka_rsi_hw_interface);
  ros::ServiceServer server_array = nh.advertiseService(ros::names::append(ros::this_node::getName(),"/write_digital_output_array"), &kuka_rsi_hw_interface::KukaHardwareInterface::write_digital_output_array, &kuka_rsi_hw_interface);
  ros::ServiceServer server_array_2outs = nh.advertiseService(ros::names::append(ros::this_node::getName(),"/write_digital_output_array_2outs"), &kuka_rsi_hw_interface::KukaHardwareInterface::write_digital_output_array_2outs, &kuka_rsi_hw_interface);


  controller_manager::ControllerManager controller_manager(&kuka_rsi_hw_interface, nh);

  kuka_rsi_hw_interface.start();

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  // Run as fast as possible
  int cycles = 0;
  while (ros::ok())
  //while (!g_quit)
  {
    ROS_INFO_ONCE("Reading");
    // Receive current state from robot
    if (!kuka_rsi_hw_interface.read(timestamp, period))
    {
      ROS_FATAL_NAMED("kuka_hardware_interface", "Failed to read state from robot. Shutting down! Cycles = %d", cycles);
      ros::shutdown();
    }
    ROS_INFO_ONCE("Read!");

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    // Update the controllers
    controller_manager.update(timestamp, period);

    // Send new setpoint to robot
    ROS_INFO_ONCE("Writing");
    kuka_rsi_hw_interface.write(timestamp, period);
    ROS_INFO_ONCE("Written!");

    cycles++;
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

  return 0;

}
