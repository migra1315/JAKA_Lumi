# jaka_lumi_ros

ROS 2 workspace for controlling and simulating the **JAKA Lumi robot**, including its mobile **body** and **minicobo arm** using MoveIt2 and custom service interfaces.

---

## 🔧 Build Instructions

```bash
git clone http://git01.jaka.com/jkzuc2/lumi.git
cd jaka_lumi_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

```

## 📁 Workspace Structure

```bash
jaka_lumi_ros/
├── jaka_lumi_body_node/              # Body topic & service interface node
├── jaka_lumi_body_server/            # Body trajectory action server
├── jaka_lumi_description/            # URDF + meshes
├── jaka_lumi_isaacsim/               # Isaac Sim interface (scripts, launch)
├── jaka_lumi_minicobo_arm_server/    # Arm trajectory action server
└── jaka_lumi_moveit_config/          # MoveIt2 + ROS 2 config
```  

## 🚀 Launch Instructions  

### 1. Launch Body Node
This node can be used standalone for direct communication and motion control of the Lumi body joints.  

```bash
ros2 launch jaka_lumi_body_node lumi_body_node.launch.py
```  

The **jaka_lumi_body_node** runs independently and connects directly to the Lumi robot body, enabling low-level control through:  

**Topics**

`/joint_states` – Publishes current joint positions of the body.

**Services**

`/lumi_body/enable` – Enable body motors (**enableService**)

`/lumi_body/move` – Move to target joint positions (**moveService**)

`/lumi_body/stop` – Stop current motion (**stopService**)

`/lumi_body/reset` – Reset the body system (**resetService**)

`/lumi_body/read_status` – Get current joint status (**readStatusService**)

### 2. Action Server Usage (Trajectory Execution via MoveIt2)
To plan and execute full motion trajectories via RViz:

Start the following:

#### a. jaka_lumi_body_server (body trajectory controller):   
  Receives trajectories from MoveIt2 and controls Lumi body joints via `FollowJointTrajectory`.

  ```bash
    ros2 launch jaka_lumi_body_server lumi_body_server.launch.py
  ```

#### b. jaka_lumi_minicobo_arm_server (arm trajectory controller)
  Controls the minicobo arm via `FollowJointTrajectory` action interface. 

  ```bash
    ros2 launch jaka_lumi_minicobo_arm_server lumi_minicobo_arm_server.launch.py
  ```  

#### c.jaka_lumi_moveit_config (RViz + MoveIt2)
  For planning and sending trajectories via `FollowJointTrajectory` action interface.

  ```bash 
    ros2 launch jaka_lumi_moveit_config demo.launch.py
   ```

### ⚠️ Important Notes

The TF tree for the arm starts from link-2 of the body, not base_link. Therefore, the body_server must be started first before controlling the arm. It publishes the root of the TF chain.

If you're only controlling the body, you can just launch:  
```bash
ros2 launch jaka_lumi_body_server lumi_body_server.launch.py
ros2 launch jaka_lumi_moveit_config demo.launch.py
```   
and it will work without the arm server.  
But for arm control, the following are required:  
```bash
ros2 launch jaka_lumi_body_server lumi_body_server.launch.py
ros2 launch jaka_lumi_minicobo_arm_server lumi_minicobo_arm_server.launch.py
ros2 launch jaka_lumi_moveit_config demo.launch.py
```


### 3. 🎮 Controller Switching in RViz

In RViz:
- Plan and execute trajectories to target poses or joint states.
- Only one planning group (arm or body or full_robot) can be selected at a time.
- A full-robot controller can be enabled to control both body and arm together.

⚠️ Note:
ROS 2 controllers use a claiming property:
- A joint can be controlled by only one controller at a time.
- If the full-robot controller is enabled, the arm-only and body-only controllers are automatically disabled, and vice-versa.

#### 🔧 Switching Controllers

Activate the full-robot controller (disable subgroup controllers):

```bash
ros2 control switch_controllers \
  --activate jaka_lumi_full_robot_controller \
  --deactivate jaka_lumi_body_controller jaka_lumi_minicobo_arm_controller \
  --strict
```

Revert to arm-only + body-only controllers:

```bash
ros2 control switch_controllers \
  --activate jaka_lumi_body_controller jaka_lumi_minicobo_arm_controller \
  --deactivate jaka_lumi_full_robot_controller \
  --strict
```


## 🛰️ Isaac Sim Integration
### 1. 🧰 Installing Isaac Sim

Follow the [official instructions](
https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html) to install Isaac-sim.

### 2. 🚀 Launching Isaac Sim

We provide the ROS 2 launch interface for running Isaac Sim and loading the robot model directly into a USD stage.  

```bash
ros2 launch jaka_lumi_isaacsim run_isaacsim.launch.py
```  

This internally:

- Launches Isaac Sim through `python.sh`.
- Loads the USD from `jaka_lumi_description`.
- Sets up **/isaac_joint_states** and **isaac_joint_commands** topics for communication with MoveIt2 and RViz.

### 3. 🧪 MoveIt Simulation Modes

To support both RViz-only simulation and Isaac Sim integration, we modified the launches.py used by MoveIt2:  

**Modifications**:

- Added **use_rviz_sim** and **use_isaac_sim** arguments.
- Controlled whether fake controllers or Isaac Sim interfaces are spawned based on the argument provided.

⚙️ Setup:

- Replace the original `launches.py` from MoveIt2 with the modified version provided in **jaka_lumi_ros** package.
- To find where to replace:  
  ```bash
  find /opt/ros/humble/ -name launches.py
  ```

### 4. Launch Moveit Simulation Modes  
#### a. RViz-only simulation mode

Launch MoveIt2 in RViz-only simulation mode using the following command:

```bash
ros2 launch jaka_lumi_moveit_config demo.launch.py use_rviz_sim:=true
```

#### b. Isaac Sim simulation mode 

- Start Isaac Sim and load the robot model:
  ```bash
  ros2 launch jaka_lumi_isaacsim run_isaacsim.launch.py
  ```

- Launch MoveIt2 and RViz with Isaac Sim integration enabled:
  ```bash
  ros2 launch jaka_lumi_moveit_config demo.launch.py use_isaac_sim:=true
  ```

This setup allows planning trajectories in RViz and executing them directly in Isaac Sim.

### 🧯 Troubleshooting Isaac Sim
#### ❌ Segmentation Fault When Using ROS2 Launch

If you get a segmentation fault like:

```swift
Fatal Python error: Segmentation fault
...
/isaacsim/code_editor/vscode/extension.py ...
```

Try these solutions:

✅ 1: Run Outside ROS to Verify

```bash
cd jaka_lumi_isaacsim/scripts
./python.sh isaacsim_moveit.py
```
If this works, your Isaac Sim install is fine — the error is likely from conflicting ROS/Isaac launch environments.

✅ 2: Clear Corrupted IsaacSim Configs

```bash
rm -rf ~/.nvidia-omniverse/logs
rm -rf ~/.nvidia-omniverse/config
```

Then relaunch using ROS:

```bash
ros2 launch jaka_lumi_isaacsim run_isaacsim.launch.py
```

### 📝 Notes

- Only one controller setup (full robot or sub-groups) can be active at a time.
- **use_isaac_sim** → enables MoveIt integration with Isaac Sim.
- **use_rviz_sim** → enables RViz-only simulation with fake controllers
- Either of the flags can be passed to the demo launch file to enable simulation.
