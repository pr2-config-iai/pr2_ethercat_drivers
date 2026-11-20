^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ethercat_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (unreleased)
------------------
* ``motors_halted`` is published as ``~/motors_halted`` (``/pr2_ethercat/motors_halted`` under
  ros2_control), the ROS 1 name that ``pr2_dashboard_aggregator`` subscribes to. A plain relative
  name resolves to the node's namespace in ROS 2, which gave ``/motors_halted``.
* The device drivers (WG0X, WG014, WG021, EK1122) moved from ``libethercat_hardware`` into their own
  plugin library ``libethercat_hardware_devices``, which nothing links. Otherwise class_loader
  assigns their factories to the loader of whichever plugin pulls in the core library (e.g. the
  ros2_control hardware component), and the device loader finds "no factory" for them.
* Port to ROS 2 (Jazzy). The realtime code paths are unchanged; only the ROS interface moved.
* ``EthercatHardware`` takes an ``rclcpp::Node::SharedPtr``; devices get it via
  ``EthercatDevice::setNode()`` before ``construct()``.
* ``EthercatHardware::tryInit()``: like ``init()`` but returns false instead of exiting the
  process, for use inside ros2_control. ``changeState()`` returns bool. ``init()`` keeps the ROS 1
  behaviour (exit) for the command-line tools. eml's singletons still allow only one
  initialization per process.
* ``ethercat_hardware::InterfaceLock``: the ROS 1 compatible ``/var/tmp/run/EtherCAT_<if>.pid``
  lock, shared by ``read_only_check`` and ``pr2_ros2_control``.
* Fix: several members (``al_``, ``em_``, ``buffers_``, ``oob_com_``, ...) were uninitialized, so
  destroying an ``EthercatHardware`` after a failed init freed garbage pointers.
* Fix: the device plugin loader was destroyed before the devices it created.
* Parameters (on the node, e.g. ``realtime_loop``): ROS 1 hierarchical names become dotted names.

  * ``realtime_socket_timeout``, ``max_pd_retries``: unchanged.
  * ``~<actuator>/enable_pressure_sensor`` -> ``<actuator>.enable_pressure_sensor`` (same for
    ``enable_ft_sensor``, ``disable_motor_model_checking``).
  * ``~motor_heating_model/<x>`` -> ``motor_heating_model.<x>``.
  * ``~<actuator>/ft_params`` -> ``<actuator>.ft_params.offsets`` (6), ``.gains`` (6) and
    ``.calibration_coeff``, now the 6x6 matrix flattened row-major to 36 doubles (ROS 2
    parameters cannot hold nested lists).
  * ``non_ethercat_devices`` struct -> list of names plus ``non_ethercat_devices.<name>.type``.

* Latched topics (``motors_halted``, ``motor_trace/*``, ``motor_temperature/*``) use
  transient_local QoS.
* Soft processor services: only the first WG06 registers them, as in ROS 1 (where later
  ``advertiseService`` calls failed); ROS 2 would otherwise create competing servers.
* New ``read_only_check`` tool: initializes the chain and reads encoders with motors halted.
  Takes the same ``/var/tmp/run/EtherCAT_<if>.pid`` lock as ROS 1 ``pr2_ethercat`` (the host's
  copy inside distrobox). Run it through ``pr2-rt`` (from ``pr2_ros2_control/tools``), which grants
  the raw-socket, mlockall and SCHED_FIFO capabilities:
  ``ros2 run --prefix pr2-rt ethercat_hardware read_only_check -i ecat0``.
* Fixes found while porting: ``num_ethercat_devices_`` was shadowed by a local in ``init()``;
  device destructors dereferenced a null slave handler when ``construct()`` never ran;
  tinyxml was never linked (only worked through catkin's transitive libraries).
* Not ported yet: ``motorconf``.

1.9.0 (2021-03-03)
------------------

1.8.19 (2019-07-26)
-------------------
* Make sure to include the correct boost libraries.
  This follows the principle of "include what you use", and
  also should in theory fix the problems on the build farm.
  (`#76 <https://github.com/PR2/pr2_ethercat_drivers/issues/76>`_)
  Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Contributors: Chris Lalancette

1.8.18 (2019-03-12)
-------------------
* fix order of Changelog contents
* Merge pull request `#75 <https://github.com/pr2/pr2_ethercat_drivers/issues/75>`_ from k-okada/add_travis
  update travis.yml and fix for kinetic/melodic
* fix gcc 8 error narrowing char
  explicitly mark the char array as unsigned.
* add build/run depend to tinyxml
* fixed compile error
* Merge pull request `#73 <https://github.com/pr2/pr2_ethercat_drivers/issues/73>`_ from TAMS-Group/pr-kinetic-add-ft-frame
  add frame_id to f/t wrench messages
* add frame_id to f/t wrench messages
  This adds the original frame name to the messages
  from the force torque sensors.
  Fixes `#66 <https://github.com/pr2/pr2_ethercat_drivers/issues/66>`_
* Contributors: David Feil-Seifer, Kei Okada, v4hn

1.8.17 (2018-02-14)
-------------------
* Merge pull request `#69 <https://github.com/PR2/pr2_ethercat_drivers/issues/69>`_ from goretkin/patch-1
  removed lingering hard-coded constant
* Merge pull request `#70 <https://github.com/PR2/pr2_ethercat_drivers/issues/70>`_ from k-okada/orp
  change maintainer to ROS orphaned package maintainer
* change maintainer to ROS orphaned package maintainer
* removed lingering constant
  MAX_FT_SAMPLES is currently set to 4, so this doesn't affect the behavior.
* Contributors: Gustavo Goretkin, Kei Okada

1.8.15 (2015-06-19)
-------------------
* Added install rules
* Contributors: TheDash

1.8.12 (2015-02-11)
-------------------

1.8.11 (2014-09-30)
-------------------

1.8.9 (2014-09-29)
------------------
