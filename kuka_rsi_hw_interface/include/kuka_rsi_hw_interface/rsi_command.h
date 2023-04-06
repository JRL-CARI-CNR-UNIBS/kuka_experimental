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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#ifndef KUKA_RSI_HW_INTERFACE_RSI_COMMAND_
#define KUKA_RSI_HW_INTERFACE_RSI_COMMAND_

#include <tinyxml.h>
#include <vector>
#include <string>
#include <ros/ros.h>

#define MASK_BYTE 0xFF00

namespace kuka_rsi_hw_interface
{

class RSICommand
{
public:
  RSICommand();
  RSICommand(std::vector<double> joint_position_correction, std::vector<bool> digital_output_bit, std::vector<uint16_t> digital_output, int32_t deltaTargetPos_PUU, int32_t deltaTargetVel_RPM, int16_t deltaTargetTor, int8_t deltaOpMode, uint32_t delta_AccelTime, uint32_t delta_DecelTime, uint32_t delta_ProfileVel, unsigned long long ipoc, std::string command_type);
  std::string xml_doc;
};

RSICommand::RSICommand()
{
  // Intentionally empty
}

RSICommand::RSICommand(std::vector<double> joint_position_correction, std::vector<bool> digital_output_bit, std::vector<uint16_t> digital_output, int32_t deltaTargetPos_PUU, int32_t deltaTargetVel_RPM, int16_t deltaTargetTor, int8_t deltaOpMode, uint32_t deltaAccelTime, uint32_t deltaDecelTime, uint32_t deltaProfileVel, unsigned long long ipoc, std::string command_type)
{
  TiXmlDocument doc;
  TiXmlElement* root = new TiXmlElement("Sen");
  root->SetAttribute("Type", "ImFree");
  TiXmlElement* el = new TiXmlElement("AK");
  // Add string attribute
  el->SetAttribute("A1", std::to_string(joint_position_correction[0]));
  el->SetAttribute("A2", std::to_string(joint_position_correction[1]));
  el->SetAttribute("A3", std::to_string(joint_position_correction[2]));
  el->SetAttribute("A4", std::to_string(joint_position_correction[3]));
  el->SetAttribute("A5", std::to_string(joint_position_correction[4]));
  el->SetAttribute("A6", std::to_string(joint_position_correction[5]));
  root->LinkEndChild(el);

  // Digital Output
  if (not command_type.compare("one_bit"))
  {
    TiXmlElement* out = new TiXmlElement("Out");
    std::string bool_string = std::to_string(digital_output_bit[0]);
    ROS_WARN_ONCE("Output bit string: %s", bool_string.c_str());

    out->SetAttribute("01", bool_string);
    root->LinkEndChild(out);
  }
//  else if (not command_type.compare("array_byte"))
//  {
//    uint16_t msb = (digital_output & MASK_BYTE) >> 8;
//    uint16_t lsb = digital_output & (~MASK_BYTE);

//    TiXmlElement* out = new TiXmlElement("Beckhoff_OUT");
//    out->SetAttribute("LSB", std::to_string(lsb));
//    out->SetAttribute("MSB", std::to_string(msb));
//    root->LinkEndChild(out);
//  }
  else if (not command_type.compare("array_uint16"))
  {
    TiXmlElement* out_beckhoff = new TiXmlElement("Beckhoff_OUT");
    out_beckhoff->LinkEndChild(new TiXmlText(std::to_string(digital_output[0])));
//    out->SetAttribute("Out", std::to_string(digital_output[0]));
    root->LinkEndChild(out_beckhoff);
  }
  else if (not command_type.compare("array_uint16_2outs"))
  {
    TiXmlElement* out_beckhoff = new TiXmlElement("Beckhoff_OUT");
    out_beckhoff->LinkEndChild(new TiXmlText(std::to_string(digital_output[0])));

//    out->SetAttribute("Out", std::to_string(digital_output[0]));
    root->LinkEndChild(out_beckhoff);

    TiXmlElement* out_odot = new TiXmlElement("Odot_OUT");
    out_odot->LinkEndChild(new TiXmlText(std::to_string(digital_output[1])));

//    out->SetAttribute("Out", std::to_string(digital_output[1]));
    root->LinkEndChild(out_odot);
  }
  else if (not command_type.compare("array_uint16_all_outs"))
  {
    TiXmlElement* out_beckhoff = new TiXmlElement("Beckhoff_OUT");
    out_beckhoff->LinkEndChild(new TiXmlText(std::to_string(digital_output[0])));
    root->LinkEndChild(out_beckhoff);

    TiXmlElement* out_odot = new TiXmlElement("Odot_OUT");
    out_odot->LinkEndChild(new TiXmlText(std::to_string(digital_output[1])));
    root->LinkEndChild(out_odot);

    TiXmlElement* delta_control = new TiXmlElement("Delta_Control");
    delta_control->LinkEndChild(new TiXmlText(std::to_string(digital_output[2])));
    root->LinkEndChild(delta_control);

    TiXmlElement* delta_TargetPos = new TiXmlElement("Delta_TargetPos");
    delta_TargetPos->LinkEndChild(new TiXmlText(std::to_string(deltaTargetPos_PUU)));
    root->LinkEndChild(delta_TargetPos);

    TiXmlElement* delta_TargetVel = new TiXmlElement("Delta_TargetVel");
    delta_TargetVel->LinkEndChild(new TiXmlText(std::to_string(deltaTargetVel_RPM)));
    root->LinkEndChild(delta_TargetVel);

    TiXmlElement* delta_TargetTor = new TiXmlElement("Delta_TargetTor");
    delta_TargetTor->LinkEndChild(new TiXmlText(std::to_string(deltaTargetTor)));
    root->LinkEndChild(delta_TargetTor);

    TiXmlElement* delta_OpMode = new TiXmlElement("Delta_OpMode");
    delta_OpMode->LinkEndChild(new TiXmlText(std::to_string(deltaOpMode)));
    root->LinkEndChild(delta_OpMode);

    TiXmlElement* delta_AccelTime = new TiXmlElement("Delta_AccelTime");
    delta_AccelTime->LinkEndChild(new TiXmlText(std::to_string(deltaAccelTime)));
    root->LinkEndChild(delta_AccelTime);

    TiXmlElement* delta_DecelTime = new TiXmlElement("Delta_DecelTime");
    delta_DecelTime->LinkEndChild(new TiXmlText(std::to_string(deltaDecelTime)));
    root->LinkEndChild(delta_DecelTime);

    TiXmlElement* delta_ProfileVel = new TiXmlElement("Delta_ProfileVel");
    delta_ProfileVel->LinkEndChild(new TiXmlText(std::to_string(deltaProfileVel)));
    root->LinkEndChild(delta_ProfileVel);

  }
  else // (not command_type.compare("none"))
  {
    ROS_WARN_ONCE("No command.");
  }

  el = new TiXmlElement("IPOC");
  el->LinkEndChild(new TiXmlText(std::to_string(ipoc)));
  root->LinkEndChild(el);

  doc.LinkEndChild(root);

  TiXmlPrinter printer;
  printer.SetStreamPrinting();
  doc.Accept(&printer);
  xml_doc = printer.Str();

  ROS_WARN_ONCE("Command to be sent (previous): %s", xml_doc.c_str());

}

} // namespace kuka_rsi_hw_interface

#endif
