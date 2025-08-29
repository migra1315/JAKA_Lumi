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


In RViz:

- Plan and execute trajectories to target poses or joint states.

- Only one planning group (arm or body) can be selected at a time in RViz.

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