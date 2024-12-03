# Logged Waypoints with ROS 2



---

## **Installation Instructions**

1. **Create a new workspace or use an existing one:**
   ```bash
   mkdir -p ~/<your_ws_name>/src
   cd ~/<your_ws_name>/src
   ```

2. **Clone the repository:**
   ```bash
   git clone https://github.com/ERIC-Robotics/logged_waypoint.git
   ```

3. **Install dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the workspace:**
   ```bash
   cd ~/<your_ws_name>
   colcon build
   ```

5. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

---

## **How to Use**

### **1. Launch Navigation**
Before using the waypoint logger or follower, ensure the robot is navigating within the environment.

---

### **2. Log Waypoints**

1. **Run the waypoint logger:**
   ```bash
   ros2 run my_waypoint_package waypoint_logger
   ```

2. **Teleoperate the robot:**
   Move the robot using teleoperation:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. **Log waypoints:**
   - Press `1` to save the current robot pose as a waypoint.
   - Once finished, press `q` to save and exit. The waypoints are stored in the YAML file specified at **line 14** of the code.

4. **Example of a saved YAML file:**
   ```yaml
   waypoints:
   - qw: 0.9999007126790602
     qx: 0.0
     qy: 0.0
     qz: -0.014091301710897897
     x: 0.5927034016440029
     y: -0.05189323222673988
     z: 0.0
   - qw: 0.9997242203348151
     qx: 0.0
     qy: 0.0
     qz: -0.02348368105613012
     x: 1.6418638198667752
     y: -0.10796116086388377
     z: 0.0
   ```

---

### **3. Follow Waypoints**

1. **Run the waypoint follower:**
   ```bash
   ros2 run my_waypoint_package logged_waypoint_follower
   ```

2. **Behavior:**
   - The robot will navigate to each waypoint in the order they were logged.
   - Ensure the `/amcl_pose` topic provides accurate localization.

---

## **Example Workflow with TurtleBot3**

- **Start the Gazebo simulation:**
  ```bash
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```

- **Launch the navigation stack:**
  ```bash
  ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=<path_to_map>
  ```

- **If no map exists, create one:**
  1. Launch SLAM:
     ```bash
     ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
     ```
  2. Save the map:
     ```bash
     ros2 run nav2_map_server map_saver_cli -f mapname
     ```

3. **Log waypoints:**
   - Start the logger:
     ```bash
     ros2 run my_waypoint_package waypoint_logger
     ```
   - Use teleoperation:
     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard
     ```
   - Press `1` to log waypoints and `q` to save and exit.

4. **Follow waypoints:**
   ```bash
   ros2 run my_waypoint_package logged_waypoint_follower
   ```

---

## **Notes**
- The waypoint logger saves waypoints to the YAML file at the location specified in **line 14** of the logger script (`~/waypoints.yaml` by default).
- Ensure the `/amcl_pose` topic is available and publishing valid localization data.
- Test the setup in simulation before deploying on a real robot.
