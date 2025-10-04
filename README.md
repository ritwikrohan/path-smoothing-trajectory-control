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
git clone https://github.com/ritwikrohan/path-smoothing-trajectory-control.git
```

### Install Dependencies

**Note** If this is your first time using rosdep, initialize it first:
```bash
sudo rosdep init
rosdep update
```
otherwise,  skip the above step

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

## Documentation

### 2.1 Setup and Execution Instructions ✓

All setup and execution instructions are provided in the [Installation and Setup](#installation-and-setup) and [Testing and Demonstration](#testing-and-demonstration) sections above.

### 2.2 Design Choices, Algorithms, and Architectural Decisions

#### Workspace Structure

```
ros2_ws/
└── src/
    ├── bot_bringup/              # Launch files and configurations
    ├── bot_controller/           # Differential drive controller and teleop
    ├── bot_description/          # URDF, meshes, and Gazebo worlds
    ├── bot_msgs/                 # Custom message and service definitions
    ├── path_smoother/            # Task 1: Path smoothing implementation
    ├── trajectory_generator/     # Task 2: Trajectory generation
    ├── trajectory_tracking_controller/  # Task 3: PD controller
    ├── obstacle_avoidance/       # Extra Credit: Obstacle avoidance
    └── send_waypoint_client/     # Test client for sending waypoints
```

#### Package Overview

- [**bot_bringup**](#bot_bringup) - System integration and launch orchestration
- [**bot_controller**](#bot_controller) - Low-level robot control interface
- [**bot_description**](#bot_description) - Robot model and simulation environment
- [**bot_msgs**](#bot_msgs) - Custom ROS2 interfaces
- [**path_smoother**](#path_smoother) - Task 1: Path smoothing algorithm
- [**trajectory_generator**](#trajectory_generator) - Task 2: Time-parameterized trajectory generation
- [**trajectory_tracking_controller**](#trajectory_tracking_controller) - Task 3: PD trajectory tracking controller
- [**obstacle_avoidance**](#obstacle_avoidance) - Extra Credit: Reactive obstacle avoidance
- [**send_waypoint_client**](#send_waypoint_client) - Testing utility for waypoint injection

---

#### bot_bringup

**Purpose:** I created this package to orchestrate the entire system by launching all necessary nodes in the correct order.

**Key Components:**
- `simulated_robot.launch.py` - Main launch file that brings up Gazebo, robot controllers, path planning stack, and RViz
- `params/assignment.yaml` - Centralized parameter file for all assignment nodes (Tasks 1, 2, 3, and obstacle avoidance)
- `rviz/my_rviz.rviz` - RViz configuration for visualizing the robot, laser scans, and planned paths

**Design Choice:** I chose to centralize all parameters in a single YAML file rather than distributing them across packages. This makes tuning the entire system easier since I can adjust all weights, gains, and thresholds from one location.

---

#### bot_controller

**Purpose:** I handle the low-level differential drive control using ROS2 Control framework.

**Key Components:**
- Differential drive controller configuration (`bot_controllers.yaml`)
- Twist multiplexer for priority-based command arbitration
- Joystick teleop interface for manual testing
- `twist_relay.cpp` - Converts between stamped and unstamped Twist messages

**How it works:** The twist_mux prioritizes command sources (navigation > keyboard > joystick). I use this to allow manual override during testing while still letting the trajectory controller have priority during autonomous operation.

---

#### bot_description

**Purpose:** I define the robot's physical structure, sensors, and simulation environment here.

**Key Components:**
- `urdf/bot.urdf.xacro` - Robot description with differential drive base, laser scanner, and IMU
- `worlds/` - Three simulation environments:
  - `empty.world` - Flat ground for basic testing (Test 1)
  - `obstacle.world` - Single box obstacle for avoidance testing (Tests 2 & 3)
  - `small_house.world` & `small_warehouse.world` - Complex environments (unused but available)

**Design Choice:** I kept the robot model simple with just essential sensors (LiDAR for obstacle detection, IMU for orientation). The laser scanner has 360 samples covering full 360° at 20Hz, which gives me enough resolution for reliable obstacle detection.

---

#### bot_msgs

**Purpose:** I defined custom message and service types specific to this assignment's requirements.

**Interfaces:**

1. **WaypointsPath.srv**
   ```
   geometry_msgs/Point[] waypoints    # Input: discrete waypoints
   ---
   nav_msgs/Path path                 # Output: smoothed continuous path
   ```
   I use this service to request path smoothing (Task 1). The client sends raw waypoints and receives a smooth path back.

2. **TrajectoryPoint.msg**
   ```
   float64 x
   float64 y
   float64 t    # time parameter
   ```
   I added the time field to represent time-parameterized trajectory points (Task 2 requirement).

3. **Trajectory.msg**
   ```
   std_msgs/Header header
   TrajectoryPoint[] points
   ```
   I package multiple trajectory points into a single message for efficient publishing.

**Data Flow:**
```
Waypoints → [PathSmoother] → Path → [TrajectoryGenerator] → Trajectory → [Controller] → cmd_vel
```

---

#### path_smoother

**Purpose:** Task 1 implementation - I smooth discrete waypoints into a continuous path using gradient descent optimization.

**Algorithm:** I implemented iterative smoothing based on minimizing two competing objectives:
1. **Data fidelity** (weight `w_data`): Keeps points close to original waypoints
2. **Smoothness** (weight `w_smooth`): Minimizes path curvature using discrete Laplacian

**Update formula:**
```
y[i] = y[i] + w_data * (x[i] - y[i]) + w_smooth * (y[i-1] + y[i+1] - 2*y[i])
```

Where:
- `x[i]` is the original waypoint position
- `y[i]` is the current smoothed position
- Endpoints are fixed (never updated)

**Key parameters:**
- `w_data: 0.7` - I weight data fidelity higher to prevent excessive deviation
- `w_smooth: 0.3` - Moderate smoothness to reduce sharp corners
- `tolerance: 1e-6` - I stop when changes become negligible
- `max_its: 1000` - Safety limit to prevent infinite loops

**Why this algorithm:** I chose gradient descent smoothing because it's simple, predictable, and gives me direct control over the data-vs-smoothness tradeoff. Unlike spline fitting, I can guarantee the path won't diverge far from the original waypoints.

**Service Interface:** I expose `/smooth_path` service so obstacle avoidance can request re-planning dynamically.

---

#### trajectory_generator

**Purpose:** Task 2 implementation - I convert the geometric path into a time-parameterized trajectory with velocity profiles.

**Algorithm:** I use constant velocity motion model:
1. Calculate Euclidean distance between consecutive path points
2. Compute time increment as `Δt = distance / velocity`
3. Accumulate time to create trajectory timestamps

**Key parameter:**
- `velocity: 0.2 m/s` - Constant forward speed

**Why constant velocity:** I chose this simple approach because:
- The path smoother already handles acceleration by creating gentle curves
- Constant velocity makes controller tuning easier (predictable dynamics)
- Trapezoidal profiles would require acceleration/deceleration zones which add complexity

**Extension possibility:** I designed the code so I can easily add trapezoidal velocity profiles later by modifying the time assignment logic.

**Topic:** I publish to `/trajectory` which the controller subscribes to.

---

#### trajectory_tracking_controller

**Purpose:** Task 3 implementation - I follow the time-parameterized trajectory using a PD controller.

**Controller Type:** PD (Proportional-Derivative) controller with separate gains for linear and angular velocity.

**Control Law:**
```
v = Kp * distance_error + Kd * d(distance_error)/dt
ω = Kp * heading_error + Kd * d(heading_error)/dt
```

**How it works:**
1. I look up the robot's current pose from TF (`odom` → `base_footprint`)
2. I interpolate the desired pose from the trajectory based on elapsed time
3. I compute position error (distance to target) and heading error (angle to target)
4. I apply PD control to generate velocity commands
5. I clamp outputs to maximum velocity limits

**Key parameters:**
- `kp: 2.0` - Proportional gain for error correction
- `kd: 0.1` - Derivative gain for damping oscillations
- `max_linear_velocity: 0.3 m/s`
- `max_angular_velocity: 1.0 rad/s`

**Why PD instead of PID:** I didn't include integral term because:
- Our short trajectories don't accumulate significant steady-state error
- Integral windup can cause overshoot during sudden direction changes
- PD is sufficient for tracking smooth paths in simulation

**Safety features I added:**
- Emergency stop subscription (`/emergency_stop`) - Immediately halts robot
- Cancel trajectory service (`/cancel_trajectory`) - Allows obstacle avoidance to abort current path
- Goal tolerance check - Stops when within 0.1m of final point

**Control loop frequency:** 10 Hz (100ms period) - Fast enough for smooth control without excessive CPU usage.

---

#### obstacle_avoidance

**Purpose:** Extra Credit implementation - I detect obstacles using laser scan and generate detour paths dynamically.

**Algorithm Overview:**

I implement a reactive obstacle avoidance strategy with three levels of response:

1. **Critical Distance (< 0.5m):**
   - Immediate emergency stop
   - Cancel active trajectory
   - Robot halts until obstacle clears

2. **Obstacle Distance (< 0.6m):**
   - Cancel current trajectory
   - Generate detour waypoints (sidestep maneuver)
   - Request new smoothed path to goal
   - Resume motion on detour

3. **Clear (> 0.6m):**
   - Clear emergency stop
   - Continue normal trajectory following

**Obstacle Detection:**
```cpp
I analyze the forward sector (±30° from front) of laser scan
I compute minimum clearance on left and right sides separately
I use the closer side to determine obstacle proximity
```

**Detour Generation:**

When I detect an obstacle, I create a detour path:
1. Determine which side has more clearance (dynamic side selection)
2. Generate arc waypoints that:
   - Ramp up lateral offset (0 → full in first 25% of arc)
   - Maintain full offset for middle 50%
   - Ramp down lateral offset (full → 0 in last 25% of arc)
3. Add waypoints directly to the original goal
4. Send complete waypoint set to path smoother

**Key parameters:**
- `obstacle_distance: 0.6m` - Trigger threshold
- `critical_distance: 0.5m` - Emergency stop threshold  
- `sector_width_deg: 30°` - Forward detection cone
- `side_offset: 1.5m` - How far to sidestep
- `forward_offset: 0.5m` - Forward travel during sidestep
- `dynamic_side: true` - Auto-select left/right based on clearance
- `min_reroute_interval_s: 3.0s` - Cooldown to prevent oscillation

**Why not Nav2 costmaps:** Since I couldn't use Nav2, I implemented direct laser-based reactive avoidance. This approach:
- Works without occupancy grids
- Responds immediately to obstacles
- Integrates with existing path smoother
- Cannot plan around complex obstacle fields (would need global replanning)

**Integration with other packages:**
```
LaserScan → [ObstacleAvoidance] → Detour Waypoints → [PathSmoother] → New Path → [Controller]
```

---

#### send_waypoint_client

**Purpose:** I provide a simple test client to inject predefined waypoint sequences for the three test cases.

**How it works:**
1. I load waypoint arrays from `params/waypoints.yaml`
2. I select the appropriate test case via launch argument
3. I convert flat coordinate list into `geometry_msgs/Point` array
4. I call `/smooth_path` service with the waypoints
5. System automatically processes: smooth → generate trajectory → follow

**Test cases defined:**
- `waypoints_case1` - Semi-circular path (10 points) for smooth trajectory following
- `waypoints_case2` - Straight path through obstacle (3 points) for avoidance testing
- `waypoints_case3` - Goal inside obstacle (2 points) for emergency stop demonstration

**Design choice:** I put test waypoints in parameters rather than hardcoding them, so I can easily add more test cases by editing the YAML file.

---

#### System Integration and Communication

**Topic Flow:**

```
/scan (LaserScan)
  └──> [obstacle_avoidance] ──> /smooth_path (service call)
                                      ↓
User waypoints ──> /smooth_path (service) ──> [path_smoother]
                                      ↓
                                /smooth_plan (Path)
                                      ↓
                            [trajectory_generator]
                                      ↓
                               /trajectory (Trajectory)
                                      ↓
                          [traj_controller] ──> /cmd_vel (Twist)
                                      ↓
                                Robot moves
```

**Service/Action Interfaces:**

| Service | Provider | Purpose |
|---------|----------|---------|
| `/smooth_path` | path_smoother | Request path smoothing (Task 1) |
| `/cancel_trajectory` | traj_controller | Abort current trajectory (for obstacle avoidance) |

**Safety Signal Flow:**

```
[obstacle_avoidance] ──> /emergency_stop (Bool) ──> [traj_controller]
                                                          ↓
                                                   Stops robot
```

---

#### Key Design Decisions Summary

1. **Service-based architecture** - I chose services over topics for path smoothing because I need request-response semantics (client waits for smoothed path before proceeding).

2. **Centralized parameters** - All tuning parameters in one YAML file makes system-wide adjustments easier.

3. **Gradient descent smoothing** - Simpler and more controllable than spline fitting; prevents excessive deviation from waypoints.

4. **Constant velocity** - Keeps controller simple and predictable; smoothed paths already handle acceleration.

5. **PD control** - Sufficient for simulation; avoids integral windup issues.

6. **Reactive obstacle avoidance** - Works without Nav2; integrates cleanly with existing path smoother by just injecting new waypoints.

7. **Emergency stop + cancel mechanism** - Two-level safety: immediate stop for critical situations, cancellation for planned rerouting.

### 2.3 Extension to Real Robot

To deploy this system on a real differential drive robot, My first idea would again be to use nav2 and change my codes to a plugin for lifecycle managers which is not very difficult as the logic remains same, just written in ros2 lifecycle node style. But even without nav2, this will work for simple implementation and for this I would need to make several key changes:

#### 1. Hardware Interface for ROS2 Control

I have done this in my previous projects using TCP/IP connection for ur5e or USB for turtlebots. Project example: https://github.com/ritwikrohan/SE3-TCP-Interface.git

**The ROS2 Control Flow:**

```
My Hardware Interface (custom C++ class)
        ↓
   I provide "Loaned Interfaces":
   - State Interfaces (encoder readings) → diff_drive_controller reads these
   - Command Interfaces (motor commands) ← diff_drive_controller writes these
        ↓
diff_drive_controller (existing ROS2 controller)
   - Subscribes to /cmd_vel (from my trajectory controller)
   - Converts Twist → wheel velocities using differential drive kinematics
   - Writes desired velocities to my command interfaces
   - Reads actual velocities from my state interfaces
   - Publishes /odom based on encoder feedback
        ↓
My Hardware Interface receives commands and sends to motors
```

**What I need to implement:**

```cpp
class RealBotHardware : public hardware_interface::SystemInterface
{
public:
  // I export state interfaces (what the controller reads)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    // I provide: wheel_left_joint/position, wheel_left_joint/velocity
    //            wheel_right_joint/position, wheel_right_joint/velocity
  }

  // I export command interfaces (what the controller writes to)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    // I provide: wheel_left_joint/velocity, wheel_right_joint/velocity
  }

  // I read encoder values from my robot's motors
  hardware_interface::return_type read(const rclcpp::Time & time, 
                                       const rclcpp::Duration & period) override
  {
    // I read encoder ticks from motor controller (via serial/USB/CAN)
    int left_ticks = readLeftEncoder();
    int right_ticks = readRightEncoder();
    
    // I convert encoder ticks to velocity
    double dt = period.seconds();
    hw_states_velocities_[LEFT] = (left_ticks - prev_left_ticks_) * 2 * M_PI 
                                  * wheel_radius_ / (encoder_resolution_ * dt);
    hw_states_velocities_[RIGHT] = (right_ticks - prev_right_ticks_) * 2 * M_PI 
                                   * wheel_radius_ / (encoder_resolution_ * dt);
    
    // I update position by integrating velocity
    hw_states_positions_[LEFT] += hw_states_velocities_[LEFT] * dt;
    hw_states_positions_[RIGHT] += hw_states_velocities_[RIGHT] * dt;
    
    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;
    
    return hardware_interface::return_type::OK;
  }

  // I write velocity commands to my robot's motors
  hardware_interface::return_type write(const rclcpp::Time & time, 
                                        const rclcpp::Duration & period) override
  {
    // I read what diff_drive_controller wants the wheels to do
    double left_vel_cmd = hw_commands_velocities_[LEFT];
    double right_vel_cmd = hw_commands_velocities_[RIGHT];
    
    // I convert velocity to motor controller commands (PWM, RPM, etc.)
    // This depends on my specific motor controller
    sendToMotorController(left_vel_cmd, right_vel_cmd);
    
    return hardware_interface::return_type::OK;
  }

private:
  // I store state and command data
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_commands_velocities_;
  
  // I track previous encoder values
  int prev_left_ticks_, prev_right_ticks_;
  
  // I need these calibration values
  double wheel_radius_;      // measured in meters
  double encoder_resolution_; // ticks per wheel revolution
  
  // I communicate with motor controller
  SerialPort* motor_controller_; // could be USB, CAN, I2C, etc.
};
```

**In the hardware interface, I would:**
- Open serial/USB/CAN/TCP/IP connection to the motor controller in `on_init()` or `on_configure()`
- Read encoder counts at each control cycle (typically 100 Hz)
- Convert encoder ticks to wheel velocities using wheel radius and encoder resolution
- Write velocity commands from diff_drive_controller to motor controllers
- Handle communication errors and timeouts gracefully

**The key insight:** The diff_drive_controller already handles the kinematics (Twist → wheel velocities). I just need to provide the hardware interface that talks to my actual motors and encoders.

#### 2. Sensor Integration

**LiDAR:**
- I would replace Gazebo's simulated laser with a real LiDAR driver package
- Common options: `rplidar_ros`, `urg_node` (Hokuyo), `sick_scan` 
- I would launch the appropriate driver that publishes to `/scan` topic
- I need to ensure the `laser_link` TF frame in my URDF matches the physical mounting position
- I would calibrate `angle_min`, `angle_max`, and `range_min/max` to match the real sensor specs

**IMU:**
- I would use a real IMU hardware driver (e.g., `bno055`, `mpu6050_driver`)
- Launch IMU driver that publishes to `/imu` topic
- I need to calibrate IMU offsets and noise parameters
- I might fuse IMU with wheel odometry using `robot_localization` package for better pose estimation

#### 3. Odometry and Localization

**Wheel Odometry Issues:**

In simulation, odometry is perfect. On a real robot, I would face:
- **Wheel slip** - Wheels slip on smooth floors, causing odometry drift
- **Uneven surfaces** - Bumps and inclines affect velocity calculations
- **Encoder noise** - Quantization and mechanical backlash introduce errors
- **Calibration errors** - Slight mistakes in wheel radius or wheelbase measurement accumulate

**My solution - Sensor Fusion:**

I would add the `robot_localization` package to fuse multiple odometry sources:

```yaml
ekf_filter_node:
  frequency: 30
  
  odom0: /bot_controller/odom  # from wheel encoders (diff_drive_controller)
  odom0_config: [true,  true,  false,  # I use x, y from wheels
                 false, false, false,  # I ignore roll, pitch, z
                 false, false, false,  # I ignore velocities from wheels
                 false, false, true,   # I use yaw rate from wheels
                 false, false, false]
  
  imu0: /imu/out               # from IMU sensor
  imu0_config: [false, false, false,  # I ignore position from IMU
                true,  true,  true,   # I use orientation from IMU
                false, false, false,  # I ignore linear velocity
                true,  true,  true,   # I use angular velocity from IMU
                true,  true,  true]   # I use linear acceleration
```

This Extended Kalman Filter gives me more robust odometry that compensates for wheel slip using IMU data.

**For even better accuracy, I would add:**
- **AMCL** (Adaptive Monte Carlo Localization) if I have a pre-built map
- **SLAM Toolbox** for simultaneous mapping and localization in unknown environments

#### 4. Parameter Tuning for Real Hardware

Real robots have different dynamics than simulation, so I would need to re-tune all my parameters:

**Path Smoother Parameters:**
- `w_data: 0.7 → 0.85` - I would increase this to prevent the robot from cutting corners too aggressively on slippery floors
- `w_smooth: 0.3 → 0.15` - I would decrease this since real mechanical constraints already limit sharp turns

**Trajectory Generator:**
- `velocity: 0.2 → 0.1 m/s` - I would start conservative for safety
- I would gradually increase it after validating stability and stopping distance

**PD Controller Gains:**

This is critical - simulation vs reality differs most here:

- `kp: 2.0 → 0.8` - I would **decrease** because real robots have:
  - Inertia and momentum (can't change direction instantly)
  - Motor response delays
  - Mechanical friction
  - Higher Kp causes oscillations on real hardware

- `kd: 0.1 → 0.4` - I would **increase** to:
  - Add more damping to reduce overshoot
  - Smooth out encoder noise
  - Stabilize response to disturbances

- `max_linear_velocity: 0.3 → 0.2 m/s` - I would reduce for safety margin
- `max_angular_velocity: 1.0 → 0.8 rad/s` - I would reduce to prevent wheel slip during sharp turns

**Tuning process I would follow:**
1. Start with very low gains (Kp=0.5, Kd=0.2)
2. Increase Kp until I see small oscillations
3. Back off Kp by 20%
4. Increase Kd to dampen any remaining oscillations
5. Test on different floor surfaces and adjust

**Obstacle Avoidance Parameters:**

- `obstacle_distance: 0.6 → 0.9m` - I would increase to account for:
  - Real stopping distance (momentum + brake delay)
  - Sensor noise and false readings
  - Safety margin for unexpected obstacles

- `critical_distance: 0.5 → 0.7m` - I would increase for safety buffer

- `side_offset: 1.5 → 0.8m` - I would **decrease** because:
  - 1.5m is too wide for most indoor corridors
  - Real robots navigate tighter spaces
  - Need to fit through doorways (~0.9m wide)

- `min_reroute_interval_s: 3.0 → 5.0s` - I would increase because:
  - Real path execution is slower
  - Motor response has delays
  - Prevents excessive replanning oscillations


#### 5. Real-World Calibration Process

**Step 1: Mechanical Calibration**
```cpp
// I would measure with calipers:
double wheel_radius = 0.033;        // measured in meters
double wheelbase = 0.170;           // distance between wheel centers
int encoder_resolution = 2048;      // ticks per revolution (from datasheet)

// I would verify by:
// - Marking wheels, rotating exactly 10 times
// - Recording encoder ticks
// - Calculating: actual_resolution = total_ticks / 10
```

**Step 2: Odometry Calibration**
```cpp
// I would run calibration tests:
// 1. Drive straight 5 meters, measure actual distance
// 2. Rotate 360°, measure actual rotation
// 3. Adjust wheel_radius and wheelbase until odometry matches reality

// Common issues:
// - Wheel radius off by ±1mm causes large drift over distance
// - Wheelbase wrong → rotation drift
```

**Step 3: Sensor Frame Calibration**
```xml
<!-- I would measure and update URDF: -->
<joint name="laser_joint" type="fixed">
  <origin xyz="-0.005 -0.002 0.121" rpy="0 0 3.14"/>
  <!-- Measured with ruler from robot center to LiDAR center -->
</joint>
```

**Step 4: Controller Tuning**
```bash
# I would use these steps:
1. Place robot in open area
2. Send simple straight-line trajectory
3. Record with `ros2 bag record /odom /pd_traj/target_pose`
4. Plot actual vs desired path
5. Adjust Kp/Kd based on error patterns:
   - Oscillations → decrease Kp or increase Kd
   - Sluggish response → increase Kp
   - Overshoot → increase Kd
```

#### 7. Software Architecture Changes

**Launch File Structure:**
```python
# real_robot.launch.py (replaces gazebo.launch.py)

def generate_launch_description():
    return LaunchDescription([
        # I launch hardware drivers instead of simulation
        Node(package='rplidar_ros', executable='rplidar_node'),
        Node(package='bno055', executable='bno055_node'),
        
        # I add robot_localization for sensor fusion
        Node(package='robot_localization', executable='ekf_node',
             parameters=[ekf_config]),
        
        
        # My navigation stack (unchanged!)
        Node(package='path_smoother', ...),
        Node(package='trajectory_generator', ...),
        Node(package='trajectory_tracking_controller', ...),
        Node(package='obstacle_avoidance', ...),
        
        # I add diagnostics and monitoring (mostly comes with robot)
    
    ])
```

**Updated TF Tree:**
```
Simulation TF (didnt use map in this project):
  odom → base_footprint (published by Gazebo)

Real Robot TF:
  map → odom → base_footprint
  (AMCL) (robot_localization EKF)
```


#### 8. What Would Stay The Same

Because I designed the system to be modular, these components work **unchanged**:

**Path smoothing algorithm** - Pure math, no hardware dependencies  
**Trajectory generation** - Just timestamps, platform-agnostic  
**PD controller structure** - Same control law, just different gains  
**Obstacle avoidance logic** - Laser processing is the same  
**ROS2 message/service definitions** - Interfaces remain identical  
**Launch file structure** - Just swap node names, keep the architecture

**What changes:**
Hardware interface (new C++ class)  
Sensor drivers (real instead of Gazebo)  
Parameter values (tuned for real dynamics)  
Calibration (measure real wheel radius, encoder resolution)

**This separation is why I chose ROS2 Control:**
- Algorithm logic lives in controllers (portable)
- Hardware specifics live in hardware interface (robot-specific)
- Same controllers work in sim and reality, just swap the hardware layer

### 2.4 AI Tools Used

I used AI tool (ChatGPT) primarily as debugging assistants and for quick API reference lookups. When I encountered issues like incorrect laser scan sector calculations or TF lookup errors, I would ask AI to help me add diagnostic print statements to visualize what was actually happening in my code. For example, when my obstacle avoidance wasn't triggering correctly, I asked for help printing out the laser scan angles and ranges at each index, which revealed I was checking the wrong angular sector. I also used AI to quickly look up ROS2-specific syntax (service callbacks, QoS settings, parameter declarations). I also went through nav2 github to get some ideas for quick implementation.

### 2.5 Extra Credit: Obstacle Avoidance Extension

For the obstacle avoidance extra credit, I implemented a reactive avoidance system in the [**obstacle_avoidance**](#obstacle_avoidance) package that monitors laser scan data and dynamically reroutes the robot around detected obstacles. The system operates on three levels: when an obstacle is detected within the critical distance threshold (0.5m), I immediately publish an emergency stop and halt the robot to prevent collision; when an obstacle appears within the normal detection range (0.6m), I cancel the current trajectory and generate a new set of detour waypoints that create a smooth arc around the obstacle by gradually ramping lateral offset up and down while maintaining forward progress; and when the path is clear, I resume normal trajectory following. The key design decision was to integrate with the existing path smoother rather than implementing a separate local planner when I detect an obstacle, I simply generate a new set of waypoints (current position → detour arc → original goal) and request the path smoother to create a new smooth path, which then flows through the normal trajectory generation and controller pipeline. This approach reuses all existing infrastructure and maintains consistency in how paths are processed. I chose sector-based laser scan analysis to determine obstacle proximity and implemented dynamic side selection that picks left or right detours based on which direction has more clearance. The system includes safeguards like minimum reroute intervals to prevent oscillations and separate thresholds for warnings versus emergency stops, making it robust enough for real-world deployment while remaining simple enough to debug and tune effectively. However obviously this implementation was done just for this assignment for simple cases. For complex cases I would definitely go for Nav2.