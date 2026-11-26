#include "pr2_ros2_control/pr2_ethercat_system.hpp"

#include <tinyxml.h>

#include <pthread.h>
#include <sched.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ethercat_hardware/ethercat_hardware.h"
#include "pr2_ros2_control/effort_chain.hpp"
#include "pr2_ros2_control/motor_release_gate.hpp"
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

// Joint names separated by spaces and/or commas.
std::set<std::string> parseNames(const std::string &value)
{
  std::string v = value;
  std::replace(v.begin(), v.end(), ',', ' ');
  std::istringstream ss(v);
  std::set<std::string> names;
  for (std::string n; ss >> n;) names.insert(n);
  return names;
}

double steadySeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
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
  if (hp.count("commanding_enabled")) commanding_enabled_ = parseBool(hp.at("commanding_enabled"));
  if (hp.count("commanded_joints")) commanded_joints_ = parseNames(hp.at("commanded_joints"));
  if (hp.count("max_effort_fraction")) max_effort_fraction_ = std::stod(hp.at("max_effort_fraction"));
  if (hp.count("min_acceptable_rt_loop_frequency"))
    min_acceptable_rt_loop_frequency_ = std::stod(hp.at("min_acceptable_rt_loop_frequency"));
  if (hp.count("velocity_window")) velocity_window_ = std::stoi(hp.at("velocity_window"));
  if (velocity_window_ < 1 || velocity_window_ > 1000)
  {
    RCLCPP_ERROR(get_logger(), "velocity_window must be within [1, 1000] samples, got %d.", velocity_window_);
    return CallbackReturn::ERROR;
  }
  if (!(max_effort_fraction_ >= 0.0 && max_effort_fraction_ <= 1.0))
  {
    RCLCPP_ERROR(get_logger(), "max_effort_fraction must be within [0, 1], got %f.", max_effort_fraction_);
    return CallbackReturn::ERROR;
  }

  std::set<std::string> with_effort_command;
  for (const auto &joint : info_.joints)
  {
    for (const auto &ci : joint.command_interfaces)
    {
      if (ci.name != "effort")
      {
        RCLCPP_ERROR(get_logger(), "Joint '%s': unsupported command interface '%s' (only effort).",
                     joint.name.c_str(), ci.name.c_str());
        return CallbackReturn::ERROR;
      }
      with_effort_command.insert(joint.name);
    }
    for (const auto &si : joint.state_interfaces)
    {
      if (si.name != "position" && si.name != "velocity" && si.name != "effort" && si.name != "calibrated")
      {
        RCLCPP_ERROR(get_logger(), "Joint '%s': unsupported state interface '%s' (use position, velocity, effort, calibrated).",
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
  for (const auto &name : commanded_joints_)
  {
    if (!with_effort_command.count(name))
    {
      RCLCPP_ERROR(get_logger(), "commanded_joints: '%s' has no effort command interface in <ros2_control>.",
                   name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  if (commanding_enabled_)
  {
    std::ostringstream ss;
    for (const auto &n : commanded_joints_) ss << " " << n;
    RCLCPP_WARN(get_logger(), "PR2 EtherCAT system on %s, %zu joints. COMMANDING ENABLED for:%s "
                "(efforts limited to %.0f%% of the URDF limit; motors halted until the first motor reset).",
                interface_.c_str(), info_.joints.size(), commanded_joints_.empty() ? " (none)" : ss.str().c_str(),
                100.0 * max_effort_fraction_);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Read-only PR2 EtherCAT system on %s, %zu joints.", interface_.c_str(), info_.joints.size());
  }
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
  windowed_actuators_.clear();
  velocity_windows_.clear();
  if (velocity_window_ > 1)
  {
    for (auto &kv : ec_->hw_->actuators_)
    {
      windowed_actuators_.push_back(kv.second);
      velocity_windows_.emplace_back(static_cast<size_t>(velocity_window_));
    }
  }
  RCLCPP_INFO(get_logger(), "Actuator velocity over %d encoder samples%s.", velocity_window_,
              velocity_window_ == 1 ? " (as ROS 1)" : "");
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
  effort_command_index_.clear();
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
      else if (si.name == "calibrated") h.calibrated = handle;
    }
    if (!joint.command_interfaces.empty())
    {
      h.effort_command = get_command_interface_handle(joint.name + "/effort");
    }
    h.commanded = commanding_enabled_ && commanded_joints_.count(joint.name) > 0;
    if (h.effort_command)
    {
      effort_command_index_[joint.name + "/effort"] = handles_.size();
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
  if (!configureCommanding())
  {
    shutdownHardware();
    return CallbackReturn::ERROR;
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
  // Here and not on activation: the state interfaces (including <joint>/calibrated) are available
  // to controllers from now on, and read() runs while inactive too.
  reportCalibration();

  // As ROS 1 pr2_ethercat: ~/reset_motors clears latched board faults and releases a halt (the
  // dashboard, or pr2_run_stop_auto_restart after the e-stop is released); ~/halt_motors halts
  // until the next reset. Read-only, the motors stay halted after a reset too.
  using Empty = std_srvs::srv::Empty;
  reset_motors_service_ = get_node()->create_service<Empty>(
    "~/reset_motors",
    [this](const std::shared_ptr<Empty::Request>, std::shared_ptr<Empty::Response>) {
      if (!commanding_enabled_)
      {
        RCLCPP_INFO(get_logger(), "Motor reset requested: clears latched board faults, motors stay halted.");
        reset_requested_ = true;
        return;
      }
      if (motors_halted_ && MotorReleaseGate::instance().requestRelease())
      {
        RCLCPP_INFO(get_logger(), "Motor reset requested: restarting the controllers, then releasing the motors.");
        return;  // read() resets once the controllers are restarted
      }
      if (motors_halted_)
      {
        RCLCPP_WARN(get_logger(), "Motor reset requested, but no controller manager restarts the controllers "
                                  "(not running in pr2_control_node): they keep their state from before the halt.");
      }
      RCLCPP_INFO(get_logger(), "Motor reset requested: clears latched board faults and the halt.");
      reset_requested_ = true;
    });
  halt_motors_service_ = get_node()->create_service<Empty>(
    "~/halt_motors",
    [this](const std::shared_ptr<Empty::Request>, std::shared_ptr<Empty::Response>) {
      RCLCPP_INFO(get_logger(), "Halt requested: motors halted until the next motor reset.");
      halt_requested_ = true;
    });
  return CallbackReturn::SUCCESS;
}

bool Pr2EthercatSystem::configureCommanding()
{
  commanded_actuators_.clear();
  joint_state_commanded_.assign(state_->joint_states_.size(), 0);
  if (!commanding_enabled_)
  {
    return true;
  }
  for (size_t j = 0; j < state_->joint_states_.size(); ++j)
  {
    const auto &js = state_->joint_states_[j];
    joint_state_commanded_[j] = js.joint_ && commanded_joints_.count(js.joint_->name) ? 1 : 0;
  }
  // Every transmission that drives a commanded joint: its actuators may be enabled. A transmission
  // can drive more joints than the listed one (the wrist drives roll and flex with two motors);
  // those joints get zero effort in write().
  std::set<std::string> found;
  for (size_t i = 0; i < robot_->transmissions_.size(); ++i)
  {
    bool commanded = false;
    for (const auto *js : state_->transmissions_out_[i])
    {
      if (js->joint_ && commanded_joints_.count(js->joint_->name))
      {
        commanded = true;
        found.insert(js->joint_->name);
      }
    }
    if (!commanded)
    {
      continue;
    }
    std::ostringstream ss;
    for (const auto *a : state_->transmissions_in_[i])
    {
      commanded_actuators_.insert(a);
      ss << " " << a->name_;
    }
    RCLCPP_WARN(get_logger(), "Transmission %s may be enabled (actuators:%s).",
                robot_->transmissions_[i]->name_.c_str(), ss.str().c_str());
  }
  for (const auto &name : commanded_joints_)
  {
    if (!found.count(name))
    {
      RCLCPP_ERROR(get_logger(), "commanded_joints: '%s' is not driven by any transmission.", name.c_str());
      return false;
    }
  }
  return true;
}

void Pr2EthercatSystem::reportCalibration()
{
  // ROS 1's calibration controllers treat a joint as calibrated when its actuator already has a
  // non-zero zero offset (stored on the board). Apply the same rule to every transmission ROS 1
  // calibrates: those with a URDF calibration edge, and the grippers.
  std::vector<std::string> missing;
  std::set<const pr2_mechanism_model::JointState *> needs;
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
    needs.insert(state_->transmissions_out_[i].begin(), state_->transmissions_out_[i].end());
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
  for (auto &h : handles_)
  {
    h.needs_calibration = needs.count(h.joint) > 0;
    if (h.calibrated)
    {
      std::ignore = set_state(h.calibrated, (!h.needs_calibration || h.joint->calibrated_) ? 1.0 : 0.0, false);
    }
    if (h.commanded && h.needs_calibration && !h.joint->calibrated_)
    {
      RCLCPP_ERROR(get_logger(), "Commanded joint %s is not calibrated: it gets no effort.", h.joint->joint_->name.c_str());
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
  fault_reported_ = false;
  monitor_started_ = false;  // lifecycle transitions pause read(); don't count that as a slow loop
  startDiagnosticsThread();
  RCLCPP_INFO(get_logger(), commanding_enabled_ ? "Active (commanding enabled; halted until a motor reset)."
                                                : "Active (read-only: motors stay halted).");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Pr2EthercatSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  MotorReleaseGate::instance().cancel();
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

  // One EtherCAT exchange: sends the commands from the last write() and receives status.
  // EthercatHardware keeps a halt until the next reset (as ROS 1), so halt only has to be passed
  // once; read-only it is passed on every cycle so that a reset does not release the motors.
  bool gate_reset = false;
  switch (MotorReleaseGate::instance().takeResult())
  {
    case MotorReleaseGate::State::RESTARTED:
      gate_reset = true;
      gate_failure_reported_ = false;
      break;
    case MotorReleaseGate::State::FAILED:
      if (!gate_failure_reported_)
      {
        RCLCPP_ERROR(get_logger(), "Restarting the controllers failed: the motors stay halted. Reset again to retry.");
        gate_failure_reported_ = true;
      }
      break;
    default:
      break;
  }
  const bool requested_reset = reset_requested_.exchange(false) || gate_reset;
  const bool reset = reset_on_next_read_ || requested_reset;
  reset_on_next_read_ = false;
  if (requested_reset)
  {
    released_by_reset_ = true;
  }
  bool halt = !commanding_enabled_ || !released_by_reset_ || halt_requested_.exchange(false);
  if (loopTooSlow())
  {
    halt = true;
  }
  ec_->update(reset, halt);
  // Position plus zero offset is the raw encoder position, which a calibration does not shift.
  for (size_t i = 0; i < windowed_actuators_.size(); ++i)
  {
    auto &s = windowed_actuators_[i]->state_;
    s.velocity_ = velocity_windows_[i].update(s.position_ + s.zero_offset_, s.sample_timestamp_.seconds());
  }
  state_->propagateActuatorPositionToJointPosition();

  bool halted = commanded_actuators_.empty();
  for (const auto *a : commanded_actuators_)
  {
    halted = halted || a->state_.halted_;
  }
  motors_halted_ = halted;

  for (const auto &h : handles_)
  {
    if (h.position) std::ignore = set_state(h.position, h.joint->position_, false);
    if (h.velocity) std::ignore = set_state(h.velocity, h.joint->velocity_, false);
    if (h.effort) std::ignore = set_state(h.effort, h.joint->measured_effort_, false);
    if (h.calibrated)
    {
      std::ignore = set_state(h.calibrated, (!h.needs_calibration || h.joint->calibrated_) ? 1.0 : 0.0, false);
    }
  }

  // Only the actuators of commanded transmissions may ever be enabled.
  for (const auto &kv : ec_->hw_->actuators_)
  {
    if (kv.second->state_.is_enabled_ && !commanded_actuators_.count(kv.second))
    {
      if (!fault_reported_)
      {
        RCLCPP_FATAL(get_logger(), "Actuator %s reports enabled, but it is not allowed to be.", kv.first.c_str());
        fault_reported_ = true;
      }
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type Pr2EthercatSystem::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                           const std::vector<std::string> &)
{
  for (const auto &name : start_interfaces)
  {
    const auto it = effort_command_index_.find(name);
    if (it == effort_command_index_.end())
    {
      continue;  // not ours
    }
    if (!handles_[it->second].commanded)
    {
      RCLCPP_ERROR(get_logger(), "Refusing to give %s to a controller: %s.", name.c_str(),
                   commanding_enabled_ ? "the joint is not in commanded_joints" : "commanding is not enabled");
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type Pr2EthercatSystem::perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                           const std::vector<std::string> &stop_interfaces)
{
  for (const auto &name : stop_interfaces)
  {
    const auto it = effort_command_index_.find(name);
    if (it != effort_command_index_.end())
    {
      handles_[it->second].claimed = false;
    }
  }
  for (const auto &name : start_interfaces)
  {
    const auto it = effort_command_index_.find(name);
    if (it != effort_command_index_.end())
    {
      handles_[it->second].claimed = handles_[it->second].commanded;
    }
  }
  return return_type::OK;
}

bool Pr2EthercatSystem::loopTooSlow()
{
  // ROS 1 pr2_ethercat: "Realtime loop should be running at least 750Hz. Calculate realtime loop
  // frequency every 200 msec. Halt motors if average frequency over last 600 msec is less than
  // 750Hz."
  constexpr double monitor_period = 0.6 / 3;
  const double now = steadySeconds();
  if (!monitor_started_)
  {
    monitor_started_ = true;
    monitor_last_time_ = now;
    monitor_cycles_ = 0;
    for (double &h : monitor_history_) h = 1000.0;
    return loop_too_slow_ = false;
  }
  ++monitor_cycles_;
  if (now - monitor_last_time_ > monitor_period)
  {
    monitor_index_ = (monitor_index_ + 1) % 3;
    monitor_history_[monitor_index_] = double(monitor_cycles_) / monitor_period;
    const double average = (monitor_history_[0] + monitor_history_[1] + monitor_history_[2]) / 3.0;
    const bool too_slow = average < min_acceptable_rt_loop_frequency_;
    if (too_slow && !loop_too_slow_ && commanding_enabled_)
    {
      RCLCPP_ERROR(get_logger(), "Halting motors: realtime loop only ran at %.1f Hz (minimum %.1f Hz).", average,
                   min_acceptable_rt_loop_frequency_);
    }
    loop_too_slow_ = too_slow;
    monitor_cycles_ = 0;
    monitor_last_time_ = now;
  }
  return loop_too_slow_;
}

return_type Pr2EthercatSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!ec_)
  {
    return return_type::OK;
  }
  if (commanding_enabled_ && state_)
  {
    // ROS 1 pr2_controller_manager::update: zero all efforts, apply the controllers' commands,
    // enforce the joint limits, propagate through the transmissions (which enables the actuators).
    state_->zeroCommands();
    for (const auto &h : handles_)
    {
      if (!h.commanded || !h.claimed || !h.effort_command || (h.needs_calibration && !h.joint->calibrated_))
      {
        continue;
      }
      double effort = 0.0;
      if (get_command(h.effort_command, effort, false) && std::isfinite(effort))
      {
        h.joint->commanded_effort_ = effort;
      }
    }
    finishEffortCommands(*state_, joint_state_commanded_, max_effort_fraction_, commanded_actuators_,
                         ec_->hw_->actuators_);
    return return_type::OK;
  }
  // Read-only: every actuator disabled, zero effort.
  for (const auto &kv : ec_->hw_->actuators_)
  {
    kv.second->command_.enable_ = false;
    kv.second->command_.effort_ = 0.0;
  }
  return return_type::OK;
}

}  // namespace pr2_ros2_control

PLUGINLIB_EXPORT_CLASS(pr2_ros2_control::Pr2EthercatSystem, hardware_interface::SystemInterface)
