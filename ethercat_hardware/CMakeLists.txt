cmake_minimum_required(VERSION 3.8)
project(ethercat_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -O3)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(pr2_msgs REQUIRED)
find_package(pr2_hardware_interface REQUIRED)
find_package(eml REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS regex system thread filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML REQUIRED IMPORTED_TARGET tinyxml)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ActuatorInfo.msg
  msg/BoardInfo.msg
  msg/MotorTemperature.msg
  msg/MotorTrace.msg
  msg/MotorTraceSample.msg
  msg/RawFTData.msg
  msg/RawFTDataSample.msg
  srv/SoftProcessorFirmwareRead.srv
  srv/SoftProcessorFirmwareWrite.srv
  srv/SoftProcessorReset.srv
  DEPENDENCIES builtin_interfaces std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# Device drivers (WG05, WG06, WG021, WG014, EK1122) are pluginlib plugins living in this library.
add_library(${PROJECT_NAME}_lib SHARED
  src/ethercat_hardware.cpp src/ethercat_com.cpp
  src/ethercat_device.cpp src/wg0x.cpp src/wg05.cpp src/wg06.cpp src/wg021.cpp
  src/ek1122.cpp src/wg014.cpp src/motor_model.cpp
  src/ethernet_interface_info.cpp src/motor_heating_model.cpp
  src/wg_soft_processor.cpp src/wg_util.cpp src/wg_mailbox.cpp src/wg_eeprom.cpp
)
# Keep the ROS 1 library name (libethercat_hardware.so); the rosidl target already uses
# the plain project name.
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  ${cpp_typesupport_target}
  rclcpp::rclcpp
  realtime_tools::realtime_tools
  pluginlib::pluginlib
  diagnostic_updater::diagnostic_updater
  ${std_msgs_TARGETS} ${geometry_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${pr2_msgs_TARGETS}
  pr2_hardware_interface::pr2_hardware_interface
  eml::eml
  # Public headers use boost::mutex/boost::thread inline; consumers get these targets via
  # cmake/ethercat_hardware-extras.cmake.
  Boost::thread Boost::system)
# Implementation-only dependencies, kept out of the exported link interface.
target_link_libraries(${PROJECT_NAME}_lib PRIVATE
  Boost::regex Boost::filesystem
  PkgConfig::TINYXML)
# Fail at link time instead of at plugin load time if a dependency is missing.
target_link_options(${PROJECT_NAME}_lib PRIVATE -Wl,--no-undefined)

pluginlib_export_plugin_description_file(${PROJECT_NAME} ethercat_device_plugin.xml)

# TODO(ros2 port): motorconf (writes motor configuration to board EEPROMs) is not ported yet.

# Read-only bring-up check: initializes the chain and reads encoders with motors halted.
add_executable(read_only_check src/tools/read_only_check.cpp)
target_link_libraries(read_only_check ${PROJECT_NAME}_lib)
# The binary gets file capabilities (setcap) for raw sockets and realtime priority. Such binaries
# run in secure mode, where the loader ignores LD_LIBRARY_PATH, so bake absolute library paths in
# (ROS 1 did the same with pr2_enable_rpath). DT_RPATH (not RUNPATH) so it also covers
# transitive dependencies.
set_target_properties(read_only_check PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE)
target_link_options(read_only_check PRIVATE -Wl,--disable-new-dtags)
install(TARGETS read_only_check DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_lib EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES actuators.conf actuators_alpha.conf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(wg0x_test test/wg0x_test.cpp)
  target_link_libraries(wg0x_test ${PROJECT_NAME}_lib)
  ament_add_gtest(motor_heating_model_test test/motor_heating_model_test.cpp)
  target_link_libraries(motor_heating_model_test ${PROJECT_NAME}_lib)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime rclcpp realtime_tools pluginlib diagnostic_updater
  diagnostic_msgs geometry_msgs std_msgs pr2_msgs pr2_hardware_interface eml)
ament_package(CONFIG_EXTRAS cmake/ethercat_hardware-extras.cmake)
