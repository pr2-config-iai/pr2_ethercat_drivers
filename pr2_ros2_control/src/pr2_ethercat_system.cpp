#include "pr2_ros2_control/pr2_ethercat_system.hpp"

#include <tinyxml.h>

#include <pthread.h>
#include <sched.h>
#include <algorithm>
#include <chrono>
#include <set>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ethercat_hardware/ethercat_hardware.h"
#include "pr2_mechanism_model/robot.h"
#include "pr2_mechanism_model/transmission.h"

namespace pr2_ros2_control
{

using CallbackReturn = Pr2EthercatSystem::CallbackReturn;
using hardware_interface::return_type;

namespace
{
bool parseBool(const std::string &value)
{
  std::string v = value;
  std::transform(v.begin(), v.end(), v.begin(), ::tolower);
  return v == "true" || v == "1" || v == "yes";
}
}  // namespace

Pr2EthercatSystem::Pr2EthercatSystem() = default;

Pr2EthercatSystem::~Pr2EthercatSystem()
{
  shutdownHardware();
}

CallbackReturn Pr2EthercatSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto &hp = info_.hardware_parameters;
  if (hp.count("interface")) interface_ = hp.at("interface");
  if (hp.count("allow_unprogrammed")) allow_unprogrammed_ = parseBool(hp.at("allow_unprogrammed"));
  if (hp.count("robot_description_topic")) robot_description_topic_ = hp.at("robot_description_topic");
  if (hp.count("robot_description_timeout")) robot_description_timeout_ = std::stod(hp.at("robot_description_timeout"));

  // Milestone 1 is read-only: refuse configurations that ask for commands.
  for (const auto &joint : info_.joints)
  {
    if (!joint.command_interfaces.empty())
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' declares command interfaces, but this version is read-only.",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto &si : joint.state_interfaces)
    {
      if (si.name != "position" && si.name != "velocity" && si.name != "effort")
      {
        RCLCPP_ERROR(get_logger(), "Joint '%s': unsupported state interface '%s' (use position, velocity, effort).",
                     joint.name.c_str(), si.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
  }

  // The PR2 transmissions are described in the URDF outside the <ros2_control> tag, which the
  // hardware component does not get to see, so read the full description from its topic.
  if (!fetchRobotDescription(urdf_))
  {
    RCLCPP_ERROR(get_logger(), "No robot description received on %s within %.1f s.",
                 robot_description_topic_.c_str(), robot_description_timeout_);
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Read-only PR2 EtherCAT system on %s, %zu joints.", interface_.c_str(), info_.joints.size());
  return CallbackReturn::SUCCESS;
}

bool Pr2EthercatSystem::fetchRobotDescription(std::string &urdf) const
{
  // A private node + executor: the controller manager's executor is busy calling us.
  auto node = std::make_shared<rclcpp::Node>(
    "pr2_ethercat_description_reader",
    rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));
  std::string received;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    robot_description_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [&received](const std_msgs::msg::String &msg) { received = msg.data; });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(robot_description_timeout_));
  while (received.empty() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some(std::chrono::milliseconds(100));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  urdf = received;
  return !urdf.empty();
}

CallbackReturn Pr2EthercatSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (ethercat_initialized_once_)
  {
    // eml keeps its master as process-wide singletons; it cannot be brought up twice.
    RCLCPP_ERROR(get_logger(), "The EtherCAT chain can only be initialized once per process; restart the node.");
    return CallbackReturn::ERROR;
  }

  if (lock_.acquire(interface_) != ethercat_hardware::InterfaceLock::Result::ACQUIRED)
  {
    RCLCPP_ERROR(get_logger(), "%s", lock_.error().c_str());
    return CallbackReturn::ERROR;
  }

  ec_ = std::make_unique<EthercatHardware>(get_node());
  ethercat_initialized_once_ = true;
  std::vector<char> iface(interface_.begin(), interface_.end());
  iface.push_back('\0');
  RCLCPP_INFO(get_logger(), "Initializing EtherCAT devices on %s ...", interface_.c_str());
  if (!ec_->tryInit(iface.data(), allow_unprogrammed_))
  {
    RCLCPP_ERROR(get_logger(), "EtherCAT initialization failed (see messages above).");
    shutdownHardware();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "EtherCAT up: %zu actuators.", ec_->hw_->actuators_.size());

  // Build the PR2 mechanism model (transmissions) on top of the actuators.
  TiXmlDocument doc;
  doc.Parse(urdf_.c_str());
  TiXmlElement *root = doc.RootElement();
  robot_ = std::make_unique<pr2_mechanism_model::Robot>(ec_->hw_);
  if (!root || !robot_->initXml(root))
  {
    RCLCPP_ERROR(get_logger(), "Failed to build the PR2 mechanism model from the robot description.");
    shutdownHardware();
    return CallbackReturn::ERROR;
  }
  state_ = std::make_unique<pr2_mechanism_model::RobotState>(robot_.get());
  // Gripper transmissions, recognised by their URDF type: the transmission classes are plugins in
  // their own library, which this plugin deliberately does not link (see pr2_mechanism_model).
  gripper_transmissions_.clear();
  for (TiXmlElement *t = root->FirstChildElement("transmission"); t; t = t->NextSiblingElement("transmission"))
  {
    const char *type = t->Attribute("type");
    if (!type && t->FirstChildElement("type"))
    {
      type = t->FirstChildElement("type")->GetText();
    }
    const std::string suffix = "PR2GripperTransmission";
    const std::string type_str = type ? type : "";
    if (t->Attribute("name") && type_str.size() >= suffix.size() &&
        type_str.compare(type_str.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      gripper_transmissions_.insert(t->Attribute("name"));
    }
  }
  RCLCPP_INFO(get_logger(), "Mechanism model: %zu transmissions, %zu joints.", robot_->transmissions_.size(),
              state_->joint_states_.size());

  // ROS 1's calibration controllers set each calibrated joint's reference position from the URDF
  // <calibration> edge when they start (also when the joint is "already calibrated"): the rising
  // edge if there is one, else the falling edge. The transmissions add it to the joint position.
  for (auto &js : state_->joint_states_)
  {
    const auto &cal = js.joint_ ? js.joint_->calibration : nullptr;
    if (cal && (cal->rising || cal->falling))
    {
      js.reference_position_ = cal->rising ? *cal->rising : *cal->falling;
    }
  }

  // Connect the declared joints to the model and cache the interface handles.
  handles_.clear();
  std::vector<std::string> unknown;
  for (const auto &joint : info_.joints)
  {
    JointHandles h;
    h.joint = state_->getJointState(joint.name);
    if (!h.joint)
    {
      unknown.push_back(joint.name);
      continue;
    }
    for (const auto &si : joint.state_interfaces)
    {
      const auto &handle = get_state_interface_handle(joint.name + "/" + si.name);
      if (si.name == "position") h.position = handle;
      else if (si.name == "velocity") h.velocity = handle;
      else if (si.name == "effort") h.effort = handle;
    }
    handles_.push_back(h);
  }
  if (!unknown.empty())
  {
    std::ostringstream ss;
    for (const auto &n : unknown) ss << " " << n;
    RCLCPP_ERROR(get_logger(), "These joints are not driven by any PR2 transmission:%s", ss.str().c_str());
    shutdownHardware();
    return CallbackReturn::ERROR;
  }
  std::set<std::string> declared;
  for (const auto &joint : info_.joints) declared.insert(joint.name);
  std::ostringstream not_exported;
  for (const auto &kv : state_->joint_states_map_)
  {
    if (!declared.count(kv.first)) not_exported << " " << kv.first;
  }
  if (!not_exported.str().empty())
  {
    RCLCPP_WARN(get_logger(), "Joints with a transmission that are not declared in <ros2_control> (not exported):%s",
                not_exported.str().c_str());
  }

  // One halted exchange so the state interfaces hold real values before anything reads them.
  // Like ROS 1 pr2_ethercat's first cycle after init: one motor reset. Two cycles later it clears
  // the boards' latched safety faults and the error flags (e.g. the timestamp jump every board
  // reports for its first packet). halt stays set on every cycle and the commands keep enable off,
  // so no board leaves MODE_OFF.
  RCLCPP_INFO(get_logger(), "Startup motor reset (as in ROS 1): clears latched board faults, motors stay halted.");
  reset_on_next_read_ = true;
  if (read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0)) != return_type::OK)
  {
    shutdownHardware();
    return CallbackReturn::ERROR;
  }

  // Same reset as above, on request (dashboard "Reset motors"), e.g. after the breakers were
  // enabled. Motors stay halted: read() passes halt on every cycle.
  using Empty = std_srvs::srv::Empty;
  reset_motors_service_ = get_node()->create_service<Empty>(
    "~/reset_motors",
    [this](const std::shared_ptr<Empty::Request>, std::shared_ptr<Empty::Response>) {
      RCLCPP_INFO(get_logger(), "Motor reset requested: clears latched board faults, motors stay halted.");
      reset_requested_ = true;
    });
  halt_motors_service_ = get_node()->create_service<Empty>(
    "~/halt_motors",
    [this](const std::shared_ptr<Empty::Request>, std::shared_ptr<Empty::Response>) {
      RCLCPP_INFO(get_logger(), "Halt requested: motors are always halted in this version.");
    });
  return CallbackReturn::SUCCESS;
}

void Pr2EthercatSystem::reportCalibration()
{
  // ROS 1's calibration controllers treat a joint as calibrated when its actuator already has a
  // non-zero zero offset (stored on the board). Apply the same rule to every transmission ROS 1
  // calibrates: those with a URDF calibration edge, and the grippers.
  std::vector<std::string> missing;
  size_t calibrated = 0;
  for (size_t i = 0; i < robot_->transmissions_.size(); ++i)
  {
    const auto &t = robot_->transmissions_[i];
    // Grippers have no <calibration> edge: ROS 1 calibrates them against the closed end stop.
    bool needs_calibration = gripper_transmissions_.count(t->name_) > 0;
    for (auto *js : state_->transmissions_out_[i])
    {
      if (js->joint_ && js->joint_->calibration && (js->joint_->calibration->rising || js->joint_->calibration->falling))
      {
        needs_calibration = true;
      }
    }
    if (!needs_calibration)
    {
      continue;
    }
    bool ok = true;
    for (auto *a : state_->transmissions_in_[i])
    {
      ok = ok && a->state_.zero_offset_ != 0.0;
    }
    for (auto *js : state_->transmissions_out_[i])
    {
      js->calibrated_ = ok;
    }
    if (ok)
    {
      ++calibrated;
    }
    else
    {
      missing.push_back(t->joint_names_.empty() ? t->name_ : t->joint_names_[0]);
    }
  }
  RCLCPP_INFO(get_logger(), "Calibration: %zu transmissions use the offsets stored on the motor boards.", calibrated);
  if (!missing.empty())
  {
    std::ostringstream ss;
    for (const auto &n : missing) ss << " " << n;
    RCLCPP_WARN(get_logger(), "Not calibrated (no stored offset; run the ROS 1 calibration):%s", ss.str().c_str());
  }
}

CallbackReturn Pr2EthercatSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!ec_)
  {
    return CallbackReturn::ERROR;
  }
  reportCalibration();
  fault_reported_ = false;
  startDiagnosticsThread();
  RCLCPP_INFO(get_logger(), "Active (read-only: motors stay halted).");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Pr2EthercatSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  stopDiagnosticsThread();
  return CallbackReturn::SUCCESS;
}

void Pr2EthercatSystem::startDiagnosticsThread()
{
  stopDiagnosticsThread();
  ec_->resumeCollectDiagnostics();
  diagnostics_stop_ = false;
  diagnostics_thread_ = std::thread([this]() {
    // Lifecycle transitions may run in the realtime control thread; never inherit its priority.
    sched_param param{};
    param.sched_priority = 0;
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    pthread_setname_np(pthread_self(), "pr2_ec_diag");
    std::unique_lock<std::mutex> lock(diagnostics_mutex_);
    while (!diagnostics_stop_)
    {
      lock.unlock();
      ec_->collectDiagnostics();
      lock.lock();
      diagnostics_cv_.wait_for(lock, std::chrono::seconds(1), [this]() { return diagnostics_stop_; });
    }
  });
}

void Pr2EthercatSystem::stopDiagnosticsThread()
{
  if (!diagnostics_thread_.joinable())
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    diagnostics_stop_ = true;
  }
  diagnostics_cv_.notify_all();
  // collectDiagnostics() waits for read() to send its frames, and ros2_control does not call read()
  // during lifecycle transitions: interrupt it rather than wait.
  ec_->interruptCollectDiagnostics();
  diagnostics_thread_.join();
}

CallbackReturn Pr2EthercatSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  shutdownHardware();
  return CallbackReturn::SUCCESS;
}

CallbackReturn Pr2EthercatSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  shutdownHardware();
  return CallbackReturn::SUCCESS;
}

CallbackReturn Pr2EthercatSystem::on_error(const rclcpp_lifecycle::State &)
{
  shutdownHardware();
  return CallbackReturn::SUCCESS;
}

void Pr2EthercatSystem::shutdownHardware()
{
  stopDiagnosticsThread();
  reset_motors_service_.reset();
  halt_motors_service_.reset();
  handles_.clear();
  state_.reset();
  robot_.reset();
  if (ec_)
  {
    RCLCPP_INFO(get_logger(), "Shutting down EtherCAT (devices back to PREOP).");
  }
  ec_.reset();  // ~EthercatHardware puts the devices back into PREOP and closes the socket
  lock_.release();
}

return_type Pr2EthercatSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!ec_ || !state_)
  {
    return return_type::OK;
  }

  // One EtherCAT exchange: sends the commands from the last write() (halted) and receives status.
  const bool reset = reset_on_next_read_ || reset_requested_.exchange(false);
  reset_on_next_read_ = false;
  ec_->update(reset, true /*halt*/);
  state_->propagateActuatorPositionToJointPosition();

  for (const auto &h : handles_)
  {
    if (h.position) std::ignore = set_state(h.position, h.joint->position_, false);
    if (h.velocity) std::ignore = set_state(h.velocity, h.joint->velocity_, false);
    if (h.effort) std::ignore = set_state(h.effort, h.joint->measured_effort_, false);
  }

  for (const auto &kv : ec_->hw_->actuators_)
  {
    if (kv.second->state_.is_enabled_)
    {
      if (!fault_reported_)
      {
        RCLCPP_FATAL(get_logger(), "Actuator %s reports enabled in read-only mode.", kv.first.c_str());
        fault_reported_ = true;
      }
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type Pr2EthercatSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!ec_)
  {
    return return_type::OK;
  }
  // Read-only: never command anything (update() is also called with halt=true).
  for (const auto &kv : ec_->hw_->actuators_)
  {
    kv.second->command_.enable_ = false;
    kv.second->command_.effort_ = 0.0;
  }
  return return_type::OK;
}

}  // namespace pr2_ros2_control

PLUGINLIB_EXPORT_CLASS(pr2_ros2_control::Pr2EthercatSystem, hardware_interface::SystemInterface)
