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
find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
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

# Shared by the hardware component and pr2_control_node: one instance in the process (D26).
add_library(pr2_motor_release_gate SHARED src/motor_release_gate.cpp)
target_include_directories(pr2_motor_release_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# The hardware component plugin
add_library(${PROJECT_NAME} SHARED src/pr2_ethercat_system.cpp src/effort_chain.cpp)
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
  pr2_motor_release_gate
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
  ${controller_manager_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  pr2_motor_release_gate
  rclcpp::rclcpp)

install(TARGETS ${PROJECT_NAME} pr2_motor_release_gate EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS pr2_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(iai_pr2_description REQUIRED)
  find_program(XACRO xacro REQUIRED)
  # The robot's description, transmissions included (the <ros2_control> block is not needed).
  set(TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/pr2_test.urdf)
  add_custom_command(OUTPUT ${TEST_URDF}
    COMMAND ${XACRO} ${iai_pr2_description_DIR}/../robots/pr2_with_ft2_cableguide.xacro -o ${TEST_URDF})
  add_custom_target(pr2_test_urdf ALL DEPENDS ${TEST_URDF})
  ament_add_gtest(test_effort_chain test/test_effort_chain.cpp)
  add_dependencies(test_effort_chain pr2_test_urdf)
  target_compile_definitions(test_effort_chain PRIVATE TEST_URDF="${TEST_URDF}")
  target_link_libraries(test_effort_chain ${PROJECT_NAME} PkgConfig::TINYXML)
  ament_add_gtest(test_motor_release_gate test/test_motor_release_gate.cpp)
  target_link_libraries(test_motor_release_gate pr2_motor_release_gate)
  ament_add_gtest(test_velocity_window test/test_velocity_window.cpp)
  target_include_directories(test_velocity_window PRIVATE include)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs ethercat_hardware pr2_mechanism_model)
ament_package()
