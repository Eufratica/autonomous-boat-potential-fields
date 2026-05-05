# Gazebo plugins in xasv-sim

Many core functionalities of xasv-sim rely on Gazebo plugins, either
shipped with standard ROS/Gazebo distributions or vendored and customized
for the ASV inspection scenarios used in this work.

This document summarizes:

- which plugins are expected from the ROS/Gazebo installation;
- which plugins are versioned locally in this repository;
- how they are built and loaded.

> Note: If you installed dependencies via
>   rosdep install --from-paths src --ignore-src -r -y
> the required ROS/Gazebo packages for the standard plugins should already
> be present in your system, as long as they are declared in the package
> manifests.


## 1. Standard ROS / Gazebo plugins

Some robot-level plugins are not shipped with xasv-sim itself.
They are provided by existing ROS/Gazebo packages and are used as-is:

  Shared library                      | Plugin name (SDF)         | Provided by (ROS pkg)       | Role
  ------------------------------------|---------------------------|-----------------------------|-------------------------------------------------------------
  libgazebo_ros_imu_sensor.so         | imu_plugin                | gazebo_plugins              | IMU sensor publishing sensor_msgs/Imu.
  libhector_gazebo_ros_imu.so         | quadrotor_imu_sim         | hector_gazebo_plugins       | Alternative IMU for debugging / ROS-only pipelines.
  libhector_gazebo_ros_magnetic.so    | quadrotor_magnetic_sim    | hector_gazebo_plugins       | 3D magnetometer with noise.
  libhector_gazebo_ros_gps.so         | quadrotor_gps_sim         | hector_gazebo_plugins       | GPS + velocity (/fix, /fix_velocity).
  libgazebo_ros_control.so            | gazebo_ros_control        | gazebo_ros_control          | Exposes joints via ros_control.

These plugins come from your ROS/Gazebo installation and are not versioned
inside this repository. They should be installed automatically by rosdep
if the corresponding packages are listed in package.xml, for example:

  <exec_depend>gazebo_plugins</exec_depend>
  <exec_depend>gazebo_ros_control</exec_depend>
  <exec_depend>hector_gazebo_plugins</exec_depend>

You can verify that these packages are available with:

  rospack find gazebo_plugins
  rospack find gazebo_ros_control
  rospack find hector_gazebo_plugins


## 2. Vendored and custom plugins in xasv-sim

The following plugins are versioned directly in this repository
(xasv_sim/plugins) to ensure reproducibility and allow customization for
ASV inspection scenarios.

They are grouped into:

- robot-level plugins (attached to the ASV model); and
- world-level plugins (attached to the environment/world).


### 2.1. Robot-level plugins (vendored/custom)

These plugins live under xasv_sim/plugins/src and xasv_sim/plugins/include
and are built by the local CMake configuration:

  Library (.so)                        | Plugin name (SDF tag)       | Role in the simulation
  -------------------------------------|-----------------------------|---------------------------------------------------------------
  libgazebo_motor_model.so             | gazebo_motor_model          | Propeller actuator model mapping motor-speed commands to thrust/torque.
  libgazebo_usv_dynamics_plugin.so     | gazebo_usv_dynamics_plugin  | 6-DOF USV dynamics with hydrodynamic coefficients.
  libbuoyancy_gazebo_plugin.so         | MigbotBuoyancyPlugin        | Buoyancy and drag model for the ASV base_link.
  libfollow_plugin.so                  | MigbotFollowPlugin          | Scripted motion for regression tests and debug scenarios.
  libgazebo_wind_plugin.so             | wind_plugin                 | Wind model with steady and gust components.
  libArduPilotPlugin.so                | ArduPilotPlugin             | Gazebo–ArduPilot X-in-the-Loop interface.
  libgazebo_plugin_msgs.so             | (internal)                  | Protobuf message library used by motor/wind plugins.

Some of these plugins are adapted from publicly available Gazebo/ArduPilot
sources. Their origin and licenses are documented in the main project
documentation and in the upstream repositories.


### 2.2. World-level plugins (custom)

These plugins implement dynamic behavior of the Madeira River environment and
log-boom obstacles:

  Library (.so)                        | Plugin name (SDF tag)      | Role in the simulation
  -------------------------------------|----------------------------|---------------------------------------------------------------
  libTrunkScaler.so                    | trunk_scaler               | Dynamically scales a trunk link to emulate changing debris size.
  libBuoySpawner.so                    | trunk2_buoy_spawner        | Periodically spawns trunk2_buoy obstacles in front of migbot1.
  libWorldGpsAlignMavlink.so           | WorldGpsAlignMavlink       | Experimental world plugin aligning Gazebo coordinates to a GPS frame.


## 3. Building the local plugins

In most use cases, you do not need to build these plugins manually:
they are compiled together with the rest of the workspace when you call
catkin_make (or catkin build). However, for advanced scenarios they can also
be built standalone.


### 3.1. Recommended: build via catkin

When xasv-sim is part of a catkin workspace, building the workspace will
compile the plugins automatically:

  cd ~/ros_ws
  catkin_make         # or: catkin build
  source devel/setup.bash

The resulting libraries are placed in the workspace devel/lib/gazebo_plugins
(or equivalent) and the appropriate paths are exported via the workspace
environment setup scripts, so Gazebo can load them without extra steps.


### 3.2. Advanced: standalone CMake build

For advanced use cases (e.g., integrating only the plugins into another
project), they can also be built with plain CMake from the plugins directory:

  cd ~/ros_ws/src/xasv-sim/xasv_sim/plugins
  mkdir -p build
  cd build
  cmake ..
  make -j$(nproc)

This will generate all plugin libraries, including for example:

  - libgazebo_motor_model.so
  - libgazebo_usv_dynamics_plugin.so
  - libbuoyancy_gazebo_plugin.so
  - libfollow_plugin.so
  - libgazebo_wind_plugin.so
  - libArduPilotPlugin.so
  - libTrunkScaler.so
  - libBuoySpawner.so
  - libWorldGpsAlignMavlink.so
  - libgazebo_plugin_msgs.so

To install them into the detected Gazebo plugin directory (if desired):

  sudo make install

Alternatively, you can keep them in the build tree and extend your
GAZEBO_PLUGIN_PATH:

  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:\
  ~/ros_ws/src/xasv-sim/xasv_sim/plugins/build


## 4. Quick test (TODO)

After building the workspace (or the standalone plugins), you can perform a quick check:

1. Launch a world that uses the ASV and its plugins, for example:

     roslaunch xasv_sim plugins_test.launch

2. Check that Gazebo does not print errors such as
   "Unable to load plugin [...]" for any of the libraries listed above.

3. In the Gazebo console, you should see messages indicating that the custom    plugins (e.g., TrunkScaler, BuoySpawner, WorldGpsAlignMavlink, gazebo_wind_plugin, gazebo_motor_model) have been loaded.

If missing plugin errors appear, double-check that:

- the required ROS packages (gazebo_plugins, gazebo_ros_control,
  hector_gazebo_plugins) are installed (either via rosdep or manually);
- the local plugins were built successfully (make -j$(nproc));
- GAZEBO_PLUGIN_PATH includes the directory with the compiled .so files.

