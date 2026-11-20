cmake_minimum_required(VERSION 3.16)
project(pr2_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ethercat_hardware REQUIRED)
find_package(pr2_mechanism_model REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML REQUIRED IMPORTED_TARGET tinyxml)

# The hardware component plugin
add_library(${PROJECT_NAME} SHARED src/pr2_ethercat_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ethercat_hardware::ethercat_hardware_lib
  pr2_mechanism_model::pr2_mechanism_model
  PkgConfig::TINYXML)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
pluginlib_export_plugin_description_file(hardware_interface pr2_ros2_control_plugins.xml)

# Realtime controller manager node. Needs a raw socket, mlockall and SCHED_FIFO: the launch file
# starts it through pr2-rt (tools/), which grants those as ambient capabilities (decision D11).
add_executable(pr2_control_node src/pr2_control_node.cpp)
target_link_libraries(pr2_control_node
  controller_manager::controller_manager
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  realtime_tools::realtime_tools
  rclcpp::rclcpp)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS pr2_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs ethercat_hardware pr2_mechanism_model)
ament_package()
