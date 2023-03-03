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
    digital_input_bit_1(false),
    digital_input_bit_2(false),
    digital_input_beckhoff(0),
    digital_input_odot(0),
    digital_input_deltaStatus(0),
    digital_input_deltaActualPos(0)
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
  bool digital_input_bit_1;
  bool digital_input_bit_2;

  uint16_t digital_input_beckhoff;
  uint16_t digital_input_odot;

  uint16_t digital_input_deltaStatus;
  uint32_t digital_input_deltaActualPos;

};

RSIState::RSIState(std::string xml_doc, std::string state_type) :
  xml_doc_(xml_doc),
  positions(DEFAULT_N_DOF, 0.0),
  initial_positions(DEFAULT_N_DOF, 0.0),
  cart_position(DEFAULT_N_DOF, 0.0),
  initial_cart_position(DEFAULT_N_DOF, 0.0),
  digital_input_bit_1(false),
  digital_input_bit_2(false),
  digital_input_beckhoff(0),
  digital_input_odot(0),
  digital_input_deltaStatus(0),
  digital_input_deltaActualPos(0)
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
  if (not state_type.compare("one_bit"))
  {
    TiXmlElement* digin_el_1;
    digin_el_1 = rob->FirstChildElement("Beckhoff_IN");
    std::string bool_string_1 = digin_el_1->FirstChild()->Value();
    //digital_input_bit_1 = (((uint16_t) std::stoul(bool_string_1)) % 2 == 1); // check if the first bit (LSB) is 1
    std::istringstream(bool_string_1) >> digital_input_bit_1;
    ROS_WARN("Input bit string: %s", bool_string_1.c_str());

    TiXmlElement* digin_el_2;
    digin_el_2 = rob->FirstChildElement("Delta_IN");
    std::string bool_string_2 = digin_el_2->FirstChild()->Value();
    //digital_input_bit_1 = (((uint16_t) std::stoul(bool_string_2)) % 2 == 1); // check if the first bit (LSB) is 1
    std::istringstream(bool_string_2) >> digital_input_bit_2;
    ROS_WARN("Input bit string: %s", bool_string_2.c_str());
  }
  else if (not state_type.compare("array_uint16"))
  {
    TiXmlElement* digin_el;
    digin_el = rob->FirstChildElement("Beckhoff_IN");
    digital_input_beckhoff = (uint16_t) std::stoull(digin_el->FirstChild()->Value());
    ROS_WARN_ONCE("Beckhoff - digital input buffer: %u", digital_input_beckhoff);
  }
  else if (not state_type.compare("array_uint16_2ins"))
  {
//    TiXmlElement* digin_el_beck;
//    digin_el_beck = rob->FirstChildElement("Beckhoff_IN");
//    digital_input_beckhoff = (uint16_t) std::stoull(digin_el_beck->FirstChild()->Value());
//    ROS_WARN("Beckhoff - digital input buffer: %us", digital_input_beckhoff);

//    TiXmlElement* digin_el_odot;
//    digin_el_odot = rob->FirstChildElement("Odot_IN");
//    digital_input_odot = (uint16_t) std::stoull(digin_el_odot->FirstChild()->Value());
//    ROS_WARN("Odot - digital input buffer: %us", digital_input_odot);

    TiXmlElement* digin_el;
    digin_el = rob->FirstChildElement("Beckhoff_IN");
    std::string bool_string_1 = digin_el->FirstChild()->Value();
    //digital_input_bit_1 = (((uint16_t) std::stoul(bool_string_1)) % 2 == 1); // check if the first bit (LSB) is 1
    std::istringstream(bool_string_1) >> digital_input_beckhoff;
    ROS_WARN("Beckhoff - digital input buffer: %s", bool_string_1.c_str());

    digin_el = rob->FirstChildElement("Odot_IN");
    std::string bool_string_2 = digin_el->FirstChild()->Value();
    //digital_input_bit_1 = (((uint16_t) std::stoul(bool_string_2)) % 2 == 1); // check if the first bit (LSB) is 1
    std::istringstream(bool_string_2) >> digital_input_odot;
    ROS_WARN("Odot - digital input buffer: %s", bool_string_2.c_str());
  }
  else if (not state_type.compare("array_uint16_all_ins"))
  {
    TiXmlElement* digin_el;
    digin_el = rob->FirstChildElement("Beckhoff_IN");
    std::string bool_string_1 = digin_el->FirstChild()->Value();
    std::istringstream(bool_string_1) >> digital_input_beckhoff;
    ROS_WARN("Beckhoff - digital input buffer: %s", bool_string_1.c_str());

    digin_el = rob->FirstChildElement("Odot_IN");
    std::string bool_string_2 = digin_el->FirstChild()->Value();
    std::istringstream(bool_string_2) >> digital_input_odot;
    ROS_WARN("Odot - digital input buffer: %s", bool_string_2.c_str());

    digin_el = rob->FirstChildElement("Delta_Status");
    std::string bool_string_3 = digin_el->FirstChild()->Value();
    std::istringstream(bool_string_3) >> digital_input_deltaStatus;
    ROS_WARN("Delta Status - digital input buffer: %s", bool_string_3.c_str());

    digin_el = rob->FirstChildElement("Delta_ActualPos");
    std::string bool_string_4 = digin_el->FirstChild()->Value();
    std::istringstream(bool_string_4) >> digital_input_deltaActualPos;
    ROS_WARN("Delta Actual Pos - digital input buffer: %s", bool_string_4.c_str());
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
