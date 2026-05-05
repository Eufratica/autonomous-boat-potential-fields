The overall X-in-the-Loop architecture implemented by xasv-sim is shown in
Figure&nbsp;1.

<p align="center">
  <img src="figs/xitl_architecture.png" alt="xasv-sim X-in-the-Loop architecture" width="600"/>
</p>

At the core of the stack, ArduPilot runs either

- in **SITL**, as a software process on the host machine, or  
- in **HITL**, on a physical ArduPilot-compatible board (tested on a Pixhawk 4 using the `ardupilot_hitl` setup).

- **QGroundControl (QGC)** connects to ArduPilot via MAVLINK for mission upload, parameter tuning and real-time monitoring.

- **MAVROS** bridges MAVLINK to ROS topics and services, allowing ROS nodes to read vehicle state and send high-level commands (e.g., mode changes, guided waypoints, velocity or wrench commands).

On the simulation side, **Gazebo** hosts the ASV and the river world:

- The `ardupilot_plugin` (via `gz_api`) implements the vehicle dynamics model (FDM) and feeds simulated sensor measurements back to ArduPilot.
- Additional simulated sensors (IMU, GPS, MAG, BAR, CAM, SON, LID) are
  attached to the ASV model inside Gazebo.
- ROS communicates with Gazebo through original `ros_gz_api` and `gazebo_ros` plugins, enabling access to topics such as poses, wrench commands, camera images and point clouds.

Human interaction and teleoperation are also integrated:

- A joystick (`joy`) node feeds commands into MAVROS, allowing a human operator to control the vehicle or intervene during experiments.
- QGrondControl (QGC) provides a graphical interface for the operator to monitor the mission and issue high-level commands.

The coloured regions in Figure&nbsp;1 highlight the different X-in-the-Loop
modes supported by the stack:

- **MITL** – Model-in-the-Loop, where modules in ROS can act in the control loop.
- **SITL** – ArduPilot running in software, coupled to Gazebo.
- **HITL** – ArduPilot running on hardware, keeping the same MAVLINK
  interfaces.
- **RITL** – Render-in-the-Loop, where Gazebo’s rendering pipeline is used
  for dataset generation and appearance variation.
- **HuITL** – Human-in-the-Loop, where a human operator interacts via
  joystick and QGC.
- **XITL** – the combined architecture, where these loops can be enabled or
  disabled according to the experiment.

This architecture allows experiments ranging from pure MITL to mixed XITL
setups, with optional human and learning components in the loop, all running
inside a realistic ROS + Gazebo river environment.
