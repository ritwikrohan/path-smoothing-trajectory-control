# Path Smoothing and Trajectory Control in 2D Space

## Table of Contents
- [Clarification: Nav2 Stack Usage](#clarification-nav2-stack-usage)
- [System Requirements](#system-requirements)
- [Installation and Setup](#installation-and-setup)
- [Testing and Demonstration](#testing-and-demonstration)
- [Documentation](#documentation)
  - [2.1 Setup and Execution Instructions](#21-setup-and-execution-instructions-)
  - [2.2 Design Choices, Algorithms, and Architectural Decisions](#22-design-choices-algorithms-and-architectural-decisions)
  - [2.3 Extension to Real Robot](#23-extension-to-real-robot)
  - [2.4 AI Tools Used](#24-ai-tools-used)
  - [2.5 Extra Credit: Obstacle Avoidance Extension](#25-extra-credit-obstacle-avoidance-extension)

---

## Clarification: Nav2 Stack Usage

**My initial thought process was to write custom path smoother and controller plugins for Nav2 and use behavior trees for dynamic path planning.** I wanted to leverage the Nav2 stack with global/local costmaps for obstacle avoidance, which would have been the industry-standard approach. However, I was uncertain whether using Nav2 was allowed for this assignment and whether we were expected to use the costmap infrastructure.

**Because of this uncertainty, I refrained from using the Nav2 stack entirely.** This led to a different implementation approach, especially for obstacle avoidance. In Nav2, obstacle avoidance is typically handled through behavior trees that trigger replanning at a certain frequency. Since the assignment specified hardcoded waypoints (simulating a global planner output), I couldn't use the typical Nav2 replanning approach.

**If Nav2 had been allowed, I would have:**
- Written custom plugins for Task 1 (path smoother plugin)
- Written custom plugins for Task 2 & 3 (controller plugin)
- Used behavior trees to dynamically trigger global path replanning when obstacles are detected
- Leveraged costmaps for obstacle representation and avoidance

Instead, I implemented a standalone solution that demonstrates the same concepts without the Nav2 framework.

---

## System Requirements

- **OS:** Ubuntu 22.04
- **ROS Distribution:** ROS2 Humble
- **Simulator:** Ignition Gazebo

---

## Installation and Setup

### Clone the Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-repo-name>
```

### Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Build the Workspace

```bash
colcon build
source install/setup.bash
```

---

## Testing and Demonstration

### Task 1, 2, 3: Path Smoothing, Trajectory Generation, and Follower

#### Test 1 - Path Smoothing, Trajectory Generation, and Following (No Obstacle)

**Video (test1.mkv):** (https://drive.google.com/drive/folders/1qYNKODlzpZJmewBlRme4HYfoWMnuN0V2?usp=sharing)

**Launch the simulation:**
```bash
ros2 launch bot_bringup simulated_robot.launch.py
```
This launches all necessary nodes and spawns the robot in an empty Gazebo world.

**Send waypoints (simulating hardcoded global planner):**
```bash
ros2 launch send_waypoint_client waypoints_client.launch.py case:=waypoints_case1
```

**Note:** Waypoints are specified in `send_waypoint_client/params/waypoints.yaml`

**Alternative - Send waypoints via CLI:**
```bash
ros2 service call /smooth_path bot_msgs/srv/WaypointsPath "{waypoints: [ {x: 0.0, y: 0.0, z: 0.0}, {x: 0.195, y: 0.382, z: 0.0}, {x: 0.383, y: 0.707, z: 0.0}, {x: 0.617, y: 0.924, z: 0.0}, {x: 0.866, y: 1.0, z: 0.0}, {x: 1.134, y: 0.924, z: 0.0}, {x: 1.383, y: 0.707, z: 0.0}, {x: 1.618, y: 0.382, z: 0.0}, {x: 1.805, y: 0.195, z: 0.0}, {x: 2.0, y: 0.0, z: 0.0} ]}"
```

**Expected Result:** The robot smoothly follows the generated trajectory from start to goal.

---

### Extra Credit: Obstacle Avoidance

#### Test 2 - Trajectory Following with Obstacle Avoidance

**Video (test2.mkv):** (https://drive.google.com/drive/folders/1qYNKODlzpZJmewBlRme4HYfoWMnuN0V2?usp=sharing)

**Launch the simulation with obstacles:**
```bash
ros2 launch bot_bringup simulated_robot.launch.py world_name:=obstacle
```
This launches all nodes with a world containing obstacles.

**Send waypoints:**
```bash
ros2 launch send_waypoint_client waypoints_client.launch.py case:=waypoints_case2
```

**Alternative - Send waypoints via CLI:**
```bash
ros2 service call /smooth_path bot_msgs/srv/WaypointsPath "{waypoints: [ {x: 0.0, y: 0.0, z: 0.0}, {x: 5.0, y: 0.0, z: 0.0}, {x: 6.0, y: 0.0, z: 0.0} ]}"
```

**Expected Result:** The robot detects the obstacle, generates a detour around it, and returns to the original path to reach the goal.

---

#### Test 3 - Emergency Stop Demonstration (Goal Inside Obstacle)

**Video (test3.mkv):** (https://drive.google.com/drive/folders/1qYNKODlzpZJmewBlRme4HYfoWMnuN0V2?usp=sharing)

**Launch the simulation with obstacles:**
```bash
ros2 launch bot_bringup simulated_robot.launch.py world_name:=obstacle
```

**Send waypoints:**
```bash
ros2 launch send_waypoint_client waypoints_client.launch.py case:=waypoints_case3
```

**Alternative - Send waypoints via CLI:**
```bash
ros2 service call /smooth_path bot_msgs/srv/WaypointsPath "{waypoints: [ {x: 0.0, y: 0.0, z: 0.0}, {x: 2.0, y: 0.0, z: 0.0} ]}"
```

**Expected Result:** The robot attempts to reach the goal but encounters an obstacle blocking the path. It tries to find alternative routes and, when no viable path exists, triggers an emergency stop to prevent collision.

## Documentation (In order of assignment Requirement)

### 2.1 Setup and Execution Instructions ✓

All setup and execution instructions are provided in the [Installation and Setup](#installation-and-setup) and [Testing and Demonstration](#testing-and-demonstration) sections above.

### 2.2 Design Choices, Algorithms, and Architectural Decisions

(To be added...)

### 2.3 Extension to Real Robot

(To be added...)

### 2.4 AI Tools Used

(To be added...)

### 2.5 Extra Credit: Obstacle Avoidance Extension

(To be added...)