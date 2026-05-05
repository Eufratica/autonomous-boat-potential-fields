# X-in-the-Loop (XITL) workflows in xasv-sim

This document collects the detailed workflows for the X-in-the-Loop modes supported in **xasv-sim**:

- **MITL** – Model-in-the-Loop
- **SITL** – Software-in-the-Loop
- **HITL** – Hardware-in-the-Loop
- **RITL** – Render-in-the-Loop
- **HuTIL** – Human-in-the-Loop

The top-level `README.md` keeps only short notes and commands; here we describe the full context and step-by-step procedures.

---

## 1. MITL (Model-in-the-Loop)

### 1.1 Concept

In the **MITL** configuration, model-based components are kept “inside the loop” together with the autopilot and the simulator. This allows you to evaluate new decision and control modules in realistic scenarios before deploying them to the real ASV.

Among the different possible MITL use cases supported by xasv-sim, this repository provides an example based on teleoperated navigation: a human operator drives the ASV in the simulated river environment, and a dedicated script converts the resulting trajectory into an ArduPilot waypoint mission.

### 1.2 Teleoperation-based mission generation (example with `migbot1`)

The reference workflow for the example ASV (`migbot1`) for MITL is:

1. **Start the simulator with ROS-built-in controllers (no ArduPilot):**
   ```bash
   roslaunch xasv_sim xasv_sim.launch        world_name:=madeira_river        robot_name:=migbot1        ardupilot:=false
   ```

2. **Start the control-allocation node for `migbot1`**, so that high-level wrench commands can be converted into individual actuator signals:
   ```bash
   rosrun migbot_allocation migbot_allocation_node __ns:=/migbot1
   ```

3. **Start the keyboard teleoperation node:**
   ```bash
   rosrun teleop_wrench_keyboard teleop_wrench_keyboard.py __ns:=/migbot1
   ```

   By default this node publishes `geometry_msgs/Wrench`; change `_topic` if your controller expects a different topic or namespace.

4. **Record a rosbag during teleoperated navigation**, capturing at least the GPS data topic `/fix` in this example:
   ```bash
   rosbag record -O "${BAG_NAME}" /fix
   ```

   You can add other messages to the bag if you want to compare different runs, and you can choose any convenient name for the bag.

5. **After finishing the teleoperation**, use a dedicated script to convert the recorded trajectory into an ArduPilot mission file (for example, generating a `.plan` file from the rosbag data):
   ```bash
   rosrun xasv_sim rosgps2mission.py        --bag "${BAG_NAME}"        --topic "/fix"        --points "${MISSION_POINTS}"        --format "${MISSION_FORMAT}"        --output "${MISSION_OUT}"        --alt "${MISSION_ALT}"
   ```

   The generated mission will be available in the `xasv_sim/data/mission` directory.

This complete test can be run from a single terminal using the `mitl_test.sh` script located in the `tests` folder of the `xasv_sim` package. From your ROS workspace:

```bash
./src/xasv-sim/xasv_sim/tests/mitl_test.sh
```

We provide a test mission for monitoring in the logboom zone which is also used in the SITL and HITL examples.

<p align="center">
  <img src="figs/mitl_test.png"
       alt="MITL test mission in the logboom zone."
       width="600">
</p>
<p align="center"><em>MITL test mission provided with xasv-sim.</em></p>

---

## 2. SITL (Software-in-the-Loop)

### 2.1 Concept

In the **SITL** configuration, the full ArduPilot Rover firmware runs as a software process on the host machine, while xasv-sim provides the simulated river environment and sensor/actuator interfaces through ROS and Gazebo.

This keeps the autopilot “inside the loop” without requiring any physical hardware, enabling repeatable experiments, fast iteration on parameters, and debugging of guidance, navigation, and control logic under realistic but fully controlled conditions.

Since SITL is fully supported by ArduPilot, you only need to install the desired firmware version and its dependencies from the official repositories. xasv-sim is mostly agnostic to this simulation mode because it uses Lua scripts for the vessel control routines. Therefore, you just need to copy the provided Lua scripts and configuration files into the appropriate folders of your ArduPilot directory.

For convenience, we provide a configuration helper script in `xasv-sim/ardupilot_sitl`.

### 2.2 Setup and workflow

1. **Run the SITL configuration helper (from your ROS workspace):**
   ```bash
   ./src/xasv-sim/ardupilot_sitl/ardupilot_sitl_config.sh
   ```

   This script copies Lua scripts and configuration files into the ArduPilot tree and performs basic setup.

2. **Start the simulator with ArduPilot connections:**
   ```bash
   roslaunch xasv_sim xasv_sim.launch        world_name:=madeira_river        robot_name:=migbot1
   ```

3. **Start an ArduPilot Rover SITL instance.** From the `Rover` folder of your ArduPilot installation:
   ```bash
   ./gzboat.sh
   ```

4. **Optionally start a Ground Control Station** such as QGroundControl or Mission Planner to load the test mission and perform the required operations.

<p align="center">
  <img src="figs/sitl_test.png"
       alt="Test mission executed in SITL mode."
       width="600">
</p>
<p align="center"><em>Test mission executed in SITL mode.</em></p>

As with MITL, this complete test can be run from a single terminal using the `sitl_test.sh` script located in the `tests` folder of the `xasv_sim` package. From your ROS workspace:

```bash
./src/xasv-sim/xasv_sim/tests/sitl_test.sh ${ARDUPILOT_PATH}
```

---

## 3. HITL (Hardware-in-the-Loop)

### 3.1 Concept

In the **HITL** configuration, the full ArduPilot Rover firmware runs on a physical flight controller (Pixhawk-class board in this case), while xasv-sim provides the virtual environment, sensors and actuator loads through ROS and Gazebo.

This setup keeps the real autopilot “inside the loop”, allowing you to test guidance, navigation and control logic under realistic dynamics and I/O timing, but without the risks and logistics of field trials.

Unlike SITL, however, this form of HITL is not supported out of the box by the standard ArduPilot stack. In our setup, the firmware must be able to treat the Gazebo sensors as if they were real onboard sensors and, at the same time, ignore or bypass the built-in sensors on the physical board.

To achieve this behaviour we modified the ArduPilot source code so that selected sensor inputs are fully driven by external data coming from Gazebo, effectively turning the hardware controller into a “host” for the virtual vehicle.

Because ArduPilot is under very active development and internal APIs (functions, data structures and sensor handling logic) change frequently, maintaining a generic, forward-compatible patch would be fragile and error-prone. Instead, we base our HITL integration on a specific [ArduPilot version](https://github.com/ttrindader/ardupilot/tree/hitl).

### 3.2 HITL bridge and firmware

On the ROS side, the HITL bridge is implemented by a dedicated node that interfaces between Gazebo and ArduPilot using custom MAVLink messages. This node subscribes to simulated sensor topics (e.g., pose, IMU, GPS) and publishes them as MAVLink messages towards the physical controller, while also relaying actuator commands back to Gazebo. Together with the patched firmware, this bridge allows the real flight controller to operate as if it were connected to a real boat, even though all sensing and actuation take place in the simulator.

In practice, the HITL support in xasv-sim is implemented by the `ardupilot_hitl` package (available in this repository), which provides a ROS–Gazebo alternative to ArduPilot’s standard HITL modes. The bridge is based on MAVROS plus an extra plugin that sends Gazebo sensor data (currently IMU, MAG, and GPS) to the physical controller via a custom MAVLink message (`GazeboMavlink`). The assessments were carried out in the context of the SIG-SAE project and have been tested on a Pixhawk 4 board.

### 3.3 Basic HITL experiment

1. **Install MAVLink and MAVROS from source** in your catkin workspace, following the [upstream instructions](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation).
   ```bash
   sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
   cd ~/ros_ws
   catkin init
   wstool init src   
   rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall
   rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
   wstool merge -t src /tmp/mavros.rosinstall
   wstool update -t src -j4
   rosdep install --from-paths src --ignore-src -y
   sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
   ``` 

3. **Run the configuration helper (from your workspace):**
   ```bash
   cd ~/ros_ws/src/xasv-sim/ardupilot_hitl/
   chmod +x config/config_desktop.sh
   ./config/config_desktop.sh
   ```

   The script will ask for the paths to your ArduPilot repository and ROS workspace, install the custom MAVLink message definition, and patch the relevant build files.

4. **Run the configuration helper (from your workspace):**
   ```bash
   cd ~/ros_ws
   catkin config --no-skiplist
   catkin build
   source devel/setup.bash
   ```

5. **Build the patched ArduPilot firmware:**
   ```bash
   sudo apt install -y binutils-arm-none-eabi gcc-arm-none-eabi
   cd <YOUR_ARDUPILOT_DIR>
   ./waf configure --board pixhawk4
   ./waf rover
   ```
   Then flash the generated firmware to the Pixhawk 4.

6. **LUA script:**
   Reconnect your board and use mavproxy to transfer the file.
   ```bash
   mavproxy.py --mav20 --console     --out=127.0.0.1:14550     --out=127.0.0.1:14552     --master=/dev/ttyACM0
   ftp put <YOUR_ARDUPILOT_DIR>/Rover/scripts/aeroboat-controlallocation.lua /APM/scripts/
   ```
   

Given this setup, a basic HITL experiment involves launching the simulated boat, connecting MAVProxy to the Pixhawk 4 over USB, and starting the HITL bridge. From five different terminals, run:

```bash
# Terminal 1: Gazebo world + robot
roslaunch xasv_sim xasv_sim.launch     world_name:=madeira_river     robot_name:=migbot1

# Terminal 2: MAVProxy (USB → Pixhawk 4)
mavproxy.py --mav20 --console     --out=127.0.0.1:14550     --out=127.0.0.1:14552     --master=/dev/ttyACM0,115200

# Terminal 3: ArduPilot SITL ↔ Gazebo bridge
roslaunch ardupilot_hitl apm.launch

# Terminal 4: MAVROS / link node
roslaunch ardupilot_hitl link.launch

# Terminal 5: Ground control station
./QGroundControl.AppImage
```

To facilitate reproducibility, the same mission used in the previous examples can be launched from a single terminal using the `hitl_test.sh` script located in the `tests` folder of the `xasv_sim` package. From your ROS workspace, run:

```bash
./src/xasv-sim/xasv_sim/tests/hitl_test.sh
```

---

## 4. Render-In-The-Loop (RITL)

### 4.1 Concept

These tools enable **Render-in-the-Loop (RITL)** experiments, where visual appearance and rendering conditions (e.g., lighting, colors, textures, mesh variants) can be changed while the physics simulation continues to run. This is useful for:

- generating diverse datasets from a single base scenario;
- studying perception algorithms under changing visual conditions;
- exploring how realistic simulators can support inspection and monitoring tasks.

In xasv-sim, RITL is implemented via custom Gazebo plugins that modify meshes and materials at runtime, without resetting the world. As an illustrative example, we provide the plugin `trunk_scaler`, capable of modifying the scale of 3D meshes according to various standards.

### 4.2 Example world and plugin

For evaluation, we use the world `madeira_river_ritl.world`, which can be executed as follows:

```bash
roslaunch xasv_sim xasv_sim.launch     world_name:=madeira_river_ritl     robot_name:=migbot1
```

This plugin can be used as a base for implementing other modifications to world meshes and instantiated in the same way in any world that supports RITL-style experiments.

---

## 5. Human-In-The-Loop (HuTIL)

### 5.1 Concept

**Human-in-the-Loop (HuTIL)** closes the X-in-the-Loop chain by explicitly involving a human operator in the control loop. Instead of leaving all decisions to the autopilot or to model-based modules, the human supervises or directly commands the ASV while the rest of the simulation stack (Gazebo world, vehicle dynamics, ArduPilot, sensors) runs as in a realistic mission.

In xasv-sim, HuTIL is naturally supported through teleoperation interfaces: the operator can drive the vessel in the rendered river environment, react to obstacles, and interact with the mission in real time.

This capability is particularly useful for data collection and training of learning-based modules. In our example use case, the ASV follows a predefined test mission in AUTO mode while the operator simply monitors the run. Whenever an obstacle (e.g., logs or buoys) threatens the planned path, the operator temporarily intervenes by sending RC commands (throttle and yaw) to perform a local avoidance maneuver, after which the mission continues in AUTO. The system logs these RC interventions together with the corresponding sensor readings and state estimates, yielding a dataset of human “override” actions during otherwise autonomous navigation.

### 5.2 HuTIL case study: supervised pre-training

To run this HuTIL case study, we use a challenging SITL arena with large static obstacles and additional obstacles that can be automatically spawned in front of the vessel by a dedicated Gazebo plugin. The ASV follows a predefined test mission in AUTO mode while the human operator applies local avoidance commands through a virtual joystick, generating a supervised dataset of obstacle-avoidance interventions.

A possible workflow is:

1. **Launch xasv-sim with the HuTIL arena (SITL mode):**
   ```bash
   roslaunch xasv_sim xasv_sim.launch        world_name:=Huitl_OA        robot_name:=migbot1        livox_enabled:=true        x:=0 y:=20 Y:=0
   ```

2. **Start ArduPilot SITL for the rover** (same script used in the SITL test). From the `Rover` directory of your ArduPilot installation:
   ```bash
   ./gzboat.sh
   ```

3. **Open QGroundControl (or another GCS):**
   - Connect to the SITL instance.
   - Load the HuTIL mission available under `xasv_sim/data/mission`.

4. **From your ROS workspace, start the HuTIL pre-training node:**
   ```bash
   roslaunch xasv_sim huitl_pre_train.launch
   ```

   This node listens to:
   - the vehicle state and sensor topics; and
   - the operator’s RC commands (virtual joystick) used for obstacle avoidance.

5. **Run the mission and apply HuTIL interventions:**
   - Start the mission in AUTO from QGroundControl.
   - Let the rover follow the mission.
   - Whenever an obstacle threatens the path, use the virtual joystick to send RC avoidance commands (throttle + yaw) while the mission keeps running in AUTO.

6. **Finish the data collection and close the session:**
   - When you want to stop building the dataset, publish:
     ```bash
     rostopic pub /xasv/pretrain_stop std_msgs/Bool "data: true"
     ```
   - The collected dataset will be saved under `xasv_sim/data/xasv_huitl_pretrain`.

This sequence produces a labeled dataset of human-in-the-loop avoidance actions during otherwise autonomous SITL missions, which is then used to pre-train the neural network.

### 5.3 Training and running the avoidance policy

With the collected data, we apply a symmetry-based data augmentation strategy that mirrors left/right avoidance maneuvers. Whenever the operator issues a non-zero yaw command, we create an additional sample by swapping the left and right range measurements and flipping the sign of both the yaw angle and the yaw command. This effectively doubles the dataset and encourages the policy to generalize to obstacle avoidance on either side.

We then train a compact MLP with two hidden layers of 64 ReLU units, which maps seven input features (front/left/right ranges, current speed, heading, distance and bearing to the nearest obstacle) to two outputs (throttle and yaw RC commands). Training uses a weighted mean-squared-error loss that puts more emphasis on yaw steering and on samples where the human operator actually intervened, and is optimized with Adam.

The resulting “intelligent avoidance” policy is saved to `trained_rc_policy.pt` under `xasv_sim/data/xasv_huitl_policy` and can be trained with a simple command from your workspace such as:

```bash
python3 src/xasv-sim/xasv_sim/scripts/train_supervised.py     --csv src/xasv-sim/xasv_sim/data/xasv_huitl_pretrain/*.csv     --epochs 50     --batch_size 64     --symmetry_aug     --out trained_rc_policy.pt
```

After training the avoidance policy, we rerun the same setup used for data collection (simulator, ArduPilot, and GCS configuration), but now the learned network is queried online. Once the environment is up, the policy node is started from your workspace with:

```bash
roslaunch xasv_sim apm.launch
./src/xasv-sim/xasv_sim/scripts/xasv_hitl_policy_wrap.sh
```

This configuration allows the operator to validate the behavior of the learned policy in real time, observing how it reacts to new obstacle encounters without manual intervention. The same HuTIL pattern can be extended to other application scenarios that involve object recognition or higher-level perception modules, for example to trigger context-aware warnings, adjust mission parameters, or refine local avoidance behavior based on additional semantic information from the environment.
