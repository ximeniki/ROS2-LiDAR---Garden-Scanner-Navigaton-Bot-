# ROS2 Autonomous Navigation: ROSBOT_XL Obstacle Avoidance

This project implements a robust autonomous navigation system for the **ROSBOT_XL** platform using **ROS2**. Unlike basic avoidance scripts, this system uses a state-machine approach to process **LiDAR** data and execute precise maneuvers in a **Gazebo** simulated environment.

## Key Technical Implementation

### 1. Advanced LiDAR Processing
* **Specific Scan Arcs:** Instead of raw data, the system analyzes a `wider_arc` (laser samples) to detect obstacles in the robot's path.
* **Range Management:** Implements `min_distance` thresholds to trigger state changes, ensuring the robot maintains a safe buffer.

### 2. State-Machine Control Logic
The navigation is managed through clear operational milestones:
* **APPROACH_OBSTACLE:** Active navigation where the robot moves towards its goal while monitoring the `wider_arc`.
* **STOP_AND_VERIFY:** A safety state triggered when an object is detected. The robot stops to verify the environment before recalculating its path.
* **Dynamic Feedback:** Continuous logging and "labeling" of laser data for easier debugging and real-time monitoring.

### 3. Integrated ROS2 Architecture
* **Nodes:** Developed in Python, utilizing standard ROS2 libraries for node management.
* **Topics:** * `/scan`: Subscribed to fetch and filter LiDAR distance arrays.
    * `/cmd_vel`: Published to execute movement commands (Linear/Angular).
* **Simulation:** Fully compatible with **TheConstruct.ai** ecosystem and **RViz2** for sensor data visualization.

## Tech Stack
* **Middleware:** ROS2 (Humble/Foxy).
* **Robot:** ROSBOT_XL (Husarion).
* **Language:** Python.
* **Simulation:** Gazebo & RViz2.

## Repository Content
* `navigation_logic.py`: Main controller implementing the logic for distance sensing and the state machine.
* `launch/`: Setup files for the simulation environment.

## Project Demonstration
You can watch the full simulation of the ROSBOT_XL navigating and executing its state-machine logic here:

🔗 **[Watch ROS2 Garden Scanner Demo on YouTube](https://www.youtube.com/watch?v=2T5tok8V0GQ&t=8s)**

*In this video, the robot demonstrates real-time decision-making, transitioning from active navigation to safety verification states upon detecting obstacles via LiDAR.*

*Note: In this video, you can observe the transition between `APPROACH_OBSTACLE` and `STOP_AND_VERIFY` states.*
---
*Technical project focused on autonomous robotics, state-machine logic, and sensor-based decision making.*
