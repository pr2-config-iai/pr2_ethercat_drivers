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
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
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
  ethercat_hardware::ethercat_hardware_lib
  pr2_mechanism_model::pr2_mechanism_model
  PkgConfig::TINYXML)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
pluginlib_export_plugin_description_file(hardware_interface pr2_ros2_control_plugins.xml)

# Realtime controller manager node. It gets file capabilities (see README), which put it in
# secure mode where LD_LIBRARY_PATH is ignored, so it needs an absolute DT_RPATH (inherited by the
# plugins it loads). Linking the plugin library puts all its dependencies on the link path.
add_executable(pr2_control_node src/pr2_control_node.cpp)
target_link_libraries(pr2_control_node
  controller_manager::controller_manager
  realtime_tools::realtime_tools
  rclcpp::rclcpp
  ${PROJECT_NAME})
# Plugins loaded at runtime (controllers, the URDF parser plugins, rmw) have dependencies that are
# not on the link path (e.g. /opt/ros/jazzy/opt/sdformat_vendor/lib), so also add every
# LD_LIBRARY_PATH entry of the build environment that lies under a ROS 2 (ament) prefix. Entries
# from other environments (e.g. ROS 1 workspaces sourced by .bashrc) are left out.
set(_pr2_rpath "${CMAKE_INSTALL_PREFIX}/lib")
string(REPLACE ":" ";" _ld_dirs "$ENV{LD_LIBRARY_PATH}")
string(REPLACE ":" ";" _ament_prefixes "$ENV{AMENT_PREFIX_PATH}")
foreach(_dir IN LISTS _ld_dirs)
  foreach(_prefix IN LISTS _ament_prefixes)
    if(_prefix AND _dir MATCHES "^${_prefix}(/|$)")
      list(APPEND _pr2_rpath "${_dir}")
      break()
    endif()
  endforeach()
endforeach()
list(REMOVE_DUPLICATES _pr2_rpath)
target_link_options(pr2_control_node PRIVATE -Wl,--disable-new-dtags -Wl,--no-as-needed)
set_target_properties(pr2_control_node PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
  INSTALL_RPATH "${_pr2_rpath}")

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS pr2_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs ethercat_hardware pr2_mechanism_model)
ament_package()
