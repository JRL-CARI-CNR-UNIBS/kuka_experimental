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

#ifndef KUKA_RSI_HW_INTERFACE_RSI_STATE_
#define KUKA_RSI_HW_INTERFACE_RSI_STATE_

#include <string>
#include <tinyxml.h>
#include <vector>
#include <bitset>

#define DEFAULT_N_DOF 6

namespace kuka_rsi_hw_interface
{

class RSIState
{

private:
  std::string xml_doc_;

public:
  RSIState() :
    positions(DEFAULT_N_DOF, 0.0),
    initial_positions(DEFAULT_N_DOF, 0.0),
    cart_position(DEFAULT_N_DOF, 0.0),
    initial_cart_position(DEFAULT_N_DOF, 0.0),
    digital_input_bit(false),
    digital_input(0)
  {
    xml_doc_.resize(1024);
  }

  RSIState(std::string xml_doc, std::string state_type);
  // AIPOS
  std::vector<double> positions;
  // ASPos
  std::vector<double> initial_positions;
  // RIst
  std::vector<double> cart_position;
  // RSol
  std::vector<double> initial_cart_position;
  // IPOC
  unsigned long long ipoc;
  // Digital input
  bool digital_input_bit;
  unsigned long digital_input;

};

RSIState::RSIState(std::string xml_doc, std::string state_type) :
  xml_doc_(xml_doc),
  positions(DEFAULT_N_DOF, 0.0),
  initial_positions(DEFAULT_N_DOF, 0.0),
  cart_position(DEFAULT_N_DOF, 0.0),
  initial_cart_position(DEFAULT_N_DOF, 0.0),
  digital_input_bit(false),
  digital_input(0)
{
  ROS_WARN_ONCE("String passed to RSIState object: %s", xml_doc_.c_str());

  // Parse message from robot
  TiXmlDocument bufferdoc;
  bufferdoc.Parse(xml_doc_.c_str());

  // Declare a printer
  TiXmlPrinter printer;
  // attach it to the document you want to convert in to a std::string
  bufferdoc.Accept(&printer);
  // Create a std::string and copy your document data in to the string
  std::string parsed_string = printer.CStr();

  ROS_WARN_ONCE("String parsed inside RSIState object by TiXmlDocument: %s", parsed_string.c_str());

  // Get the Rob node
  TiXmlElement* rob = bufferdoc.FirstChildElement("Rob");

  // Extract axis specific actual position
  TiXmlElement* AIPos_el = rob->FirstChildElement("AIPos");
  AIPos_el->Attribute("A1", &positions[0]);
  AIPos_el->Attribute("A2", &positions[1]);
  AIPos_el->Attribute("A3", &positions[2]);
  AIPos_el->Attribute("A4", &positions[3]);
  AIPos_el->Attribute("A5", &positions[4]);
  AIPos_el->Attribute("A6", &positions[5]);

  // Extract axis specific setpoint position
  TiXmlElement* ASPos_el = rob->FirstChildElement("ASPos");
  ASPos_el->Attribute("A1", &initial_positions[0]);
  ASPos_el->Attribute("A2", &initial_positions[1]);
  ASPos_el->Attribute("A3", &initial_positions[2]);
  ASPos_el->Attribute("A4", &initial_positions[3]);
  ASPos_el->Attribute("A5", &initial_positions[4]);
  ASPos_el->Attribute("A6", &initial_positions[5]);

  // Extract cartesian actual position
  TiXmlElement* RIst_el = rob->FirstChildElement("RIst");
  RIst_el->Attribute("X", &cart_position[0]);
  RIst_el->Attribute("Y", &cart_position[1]);
  RIst_el->Attribute("Z", &cart_position[2]);
  RIst_el->Attribute("A", &cart_position[3]);
  RIst_el->Attribute("B", &cart_position[4]);
  RIst_el->Attribute("C", &cart_position[5]);

  // Extract cartesian actual position
  TiXmlElement* RSol_el = rob->FirstChildElement("RSol");
  RSol_el->Attribute("X", &initial_cart_position[0]);
  RSol_el->Attribute("Y", &initial_cart_position[1]);
  RSol_el->Attribute("Z", &initial_cart_position[2]);
  RSol_el->Attribute("A", &initial_cart_position[3]);
  RSol_el->Attribute("B", &initial_cart_position[4]);
  RSol_el->Attribute("C", &initial_cart_position[5]);

  // Extract digital input values
  TiXmlElement* digin_el;
  if (not state_type.compare("one_bit"))
  {
    ROS_WARN_ONCE("Saving one bit: ");
    digin_el = rob->FirstChildElement("In");
    digin_el->QueryBoolAttribute("01", &digital_input_bit);
    std::string bool_string = std::to_string(digital_input_bit);
    ROS_WARN_ONCE("Input bit string: %s", bool_string.c_str());
  }
  else if (not state_type.compare("array_byte"))
  {
    ROS_WARN_ONCE("Saving one byte: ");
    digin_el = rob->FirstChildElement("Beckhoff_IN");
    digital_input = std::stoul(digin_el->FirstChild()->Value());
    ROS_WARN_ONCE("Digital input buffer: %lu", digital_input);
  }
  else // (not state_type.compare("none"))
  {
    ROS_WARN_ONCE("No saving.");
  }

  // Get the IPOC timestamp
  TiXmlElement* ipoc_el = rob->FirstChildElement("IPOC");
  ipoc = std::stoull(ipoc_el->FirstChild()->Value());

  ROS_WARN_ONCE("Reading complete.");

}

} // namespace kuka_rsi_hw_interface

#endif
