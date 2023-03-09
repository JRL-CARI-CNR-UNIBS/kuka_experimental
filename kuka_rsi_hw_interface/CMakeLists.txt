cmake_minimum_required(VERSION 3.1.0)
project(kuka_rsi_hw_interface)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  hardware_interface
  joint_limits_interface
  realtime_tools
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)

add_service_files(
  FILES
  write_output_bool.srv
  write_output_bool_array.srv
  write_output_bool_array_2outs.srv
  write_output_bool_array_all_outs.srv
)

add_message_files(
  FILES
  input_data.msg
  uint16_t_array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    std_msgs
  DEPENDS
    TinyXML
)

set(INCLUDE_DIR include/kuka_rsi_hw_interface)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${INCLUDE_DIR}

)

add_library(kuka_hardware_interface
  src/kuka_hardware_interface.cpp
  ${INCLUDE_DIR}/kuka_hardware_interface.h
  ${INCLUDE_DIR}/rsi_command.h
  ${INCLUDE_DIR}/rsi_state.h
  ${INCLUDE_DIR}/udp_server.h
)

target_link_libraries(kuka_hardware_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

add_executable(kuka_hardware_interface_node
  src/kuka_hardware_interface_node.cpp
)

target_link_libraries(kuka_hardware_interface_node
  kuka_hardware_interface
)

install(
  TARGETS kuka_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS kuka_hardware_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
