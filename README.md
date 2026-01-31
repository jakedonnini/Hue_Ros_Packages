# Hue ROS Packages - Autonomous Navigation System

<!-- Main logo and robot image side by side -->
<table>
  <tr>
    <td align="center">
      <img width="160" height="160" alt="Hue_logo" src="https://github.com/user-attachments/assets/7e132a94-f5e5-468e-9519-a6b3a7d17b28" /><br>
      <sub><b>Project Logo</b></sub>
    </td>
    <td align="center">
      <img width="200" height="160" alt="Bot_Side" src="https://github.com/user-attachments/assets/5ac1333a-ed70-4642-8d2c-58bef7423e53" /><br>
      <sub><b>Robot Side View</b></sub>
    </td>
  </tr>
</table>

![ground_view](https://github.com/user-attachments/assets/131dbb56-9a8b-42b3-8a0d-934ffb7c3479)


## Overview

This repository contains a complete ROS 2 autonomous navigation system designed for GPS-based robotic navigation. The system integrates multiple sensors, implements sensor fusion algorithms, and provides both C++ and Python implementations for high-performance autonomous navigation.

## Media Gallery

### Images

<!-- Gallery table for images with captions -->
<table>
  <tr>
    <td>
      <img width="350" alt="Screenshot iso" src="https://github.com/user-attachments/assets/622d435b-481d-4420-b322-b3eb85101c18" /><br>
      <sub><b>Simulation Screenshot</b></sub>
    </td>
    <td>
      <img width="350" alt="GUI_screen_shot" src="https://github.com/user-attachments/assets/ce73dbbc-e3b3-4a57-93b8-4265bad487eb" /><br>
      <sub><b>GUI Waypoint Selection</b></sub>
    </td>
  </tr>
  <tr>
    <td>
      <img width="350" alt="Outdoor Test" src="https://github.com/user-attachments/assets/cf8ad51c-c857-467f-a49c-ad3ab9613d60" /><br>
      <sub><b>Outdoor Test Run</b></sub>
    </td>
    <td>
      <img width="350" alt="Drone Shot" src="https://github.com/user-attachments/assets/c9054d57-7426-49df-92da-7c7e9c31a595" /><br>
      <sub><b>Drone Photo</b></sub>
    </td>
  </tr>
</table>

### Videos

<table>
  <tr>
    <td align="center">
      <a href="https://youtu.be/IbIYlnNnJFM?si=YOSsBUXNI2kJ4ci4">
        <img width="350" alt="Demo Video 1" src="https://img.youtube.com/vi/IbIYlnNnJFM/0.jpg" /><br>
        <sub><b>Demo Video 1</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://youtu.be/JNXMdchNOyw?si=oNPIDicQ81vm9oLj">
        <img width="350" alt="Demo Video 2" src="https://img.youtube.com/vi/JNXMdchNOyw/0.jpg" /><br>
        <sub><b>Demo Video 2</b></sub>
      </a>
    </td>
  </tr>
</table>

## System Architecture

<p align="center">
  <img width="772" height="350" alt="Bolck_diagram" src="https://github.com/user-attachments/assets/3aa68ec4-ab15-49be-9a6a-afa96b6e574d" />
</p>

### Core Components

#### 1. **Arduino Reader** (`arduino_reader_cpp`)
- **Language**: C++
- **Purpose**: Serial communication interface with Arduino-based encoder systems
- **Features**:
  - High-speed serial communication (460800 baud)
  - Real-time encoder data reading
  - PWM command publishing for motor control
  - Symlink-based port management for robust hardware connections
- **Topics**:
  - Publishes: `encoder` (custom_msg/TwoInt)
  - Subscribes: `PWM` (custom_msg/TwoInt)

#### 2. **GPS Publisher** (`gps_publisher`, `gps_publisher_cpp`)
- **Language**: Python (primary), C++ (fusion node)
- **Purpose**: Multi-GPS sensor data acquisition and fusion
- **Features**:
  - Dual GPS sensor support
  - GPS coordinate fusion and filtering
  - Coordinate system transformation (lat/lon to local cartesian)
  - Real-time position estimation
- **Topics**:
  - Publishes: `gps`, `gps2`, `gps/data`, `gps1_cm`, `gps2_cm`

#### 3. **Dead Reckoning** (`deadreck_cpp`)
- **Language**: C++
- **Purpose**: Inertial navigation and odometry calculation
- **Features**:
  - Encoder-based position estimation
  - Velocity calculation from wheel encoders
  - Integration with GPS data for position correction
- **Topics**:
  - Publishes: `deadReckoning/vel` (custom_msg/Coordinates)
  - Subscribes: `encoder` (custom_msg/TwoInt)

#### 4. **Kalman Filter** (`navigator`, `calm_man_cpp`)
- **Language**: Python (primary), C++
- **Purpose**: Sensor fusion using Kalman filtering
- **Features**:
  - GPS and dead reckoning fusion
  - State estimation with uncertainty quantification
  - Real-time position and velocity filtering
- **Topics**:
  - Publishes: `kalman/data` (custom_msg/GpsData)
  - Subscribes: `gps/data`, `deadReckoning/vel`

#### 5. **Navigation Controller** (`navigator_cpp`)
- **Language**: C++
- **Purpose**: High-level path planning and robot control
- **Features**:
  - Waypoint-based navigation
  - Pure pursuit path following algorithm
  - Dynamic obstacle avoidance
  - PID-based motor control
- **Topics**:
  - Subscribes: `coordinates`, `kalman/data`, `deadReckoning/vel`
  - Publishes: `PWM` (custom_msg/TwoInt)

#### 6. **Waypoint Management** (`get_waypoints`)
- **Language**: Python
- **Purpose**: Waypoint generation and path planning
- **Features**:
  - Text file-based waypoint loading
  - Dynamic waypoint generation
  - Support for complex path patterns (lines, arcs, logos)
- **Topics**:
  - Publishes: `coordinates` (custom_msg/Coordinates)

#### 7. **GUI Interface** (`gui`)
- **Language**: Python
- **Purpose**: Real-time monitoring and control interface
- **Features**:
  - Real-time GPS position plotting
  - Interactive map visualization
  - Image-based waypoint generation
  - System status monitoring
- **Dependencies**: CustomTkinter, TkinterMapView, PIL, Matplotlib

#### 8. **Custom Messages** (`custom_msg`)
- **Language**: ROS 2 Interface Definition
- **Purpose**: Define custom message types for inter-node communication
- **Message Types**:
  - `Coordinates.msg`: X/Y position with toggle flag
  - `GpsData.msg`: X/Y position with heading angle
  - `TwoInt.msg`: Dual integer values with toggle (for encoder/PWM data)

## System Requirements

### Hardware
- Teensy 4.0 microcontroller with encoder interface
- Dual RTK GPS receivers (for improved accuracy and angular positioning)
- Differential drive robot platform
- Linux-based main computer (Intel NUC)

### Software Dependencies
- ROS 2 (Humble or newer)
  - Nodes built completely from scratch so disto indifferent
- Python 3.8+
- C++ compiler with C++17 support
- Serial communication libraries
- Custom dependencies:
  - `libserialport` (for C++ serial communication)
  - `customtkinter`, `tkintermapview`, `PIL`, `matplotlib` (for GUI)
  - `rclcpp`, `rclpy` (ROS 2 client libraries)

## Installation and Setup

### 1. Clone and Build
```bash
# Clone the repository
git clone https://github.com/jakedonnini/Hue_Ros_Packages.git
cd Hue_Ros_Packages

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Hardware Setup
- Connect Arduino to `/dev/ttyRobot1` (or create appropriate symlink)
- Configure GPS receivers on designated serial ports
- Ensure proper power supply for all components

## Running the System

### SSH-Based Remote Operation

#### Full Navigation System (C++ Optimized)
```bash
# SSH into the robot
ssh user@robot_ip

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/Hue_Ros_Packages/install/setup.bash

# Launch complete navigation system
ros2 launch gps_publisher navigation_launch_cpp.py
```

**Active Nodes in navigation_launch_cpp.py:**
- GPS publishers (Python-based for sensor compatibility)
- Arduino reader (C++ for high-speed serial communication)
- Dead reckoning (C++ for real-time performance)
- Kalman filter (Python for algorithm flexibility)
- Navigation controller (C++ for precise motor control)

#### GPS-Only System (For GPS Testing)
```bash
# Launch GPS fusion system only
ros2 launch gps_publisher gps_launch_cpp.py
```

**Active Nodes in gps_launch_cpp.py:**
- GPS sensor publishers
- GPS fusion node (C++ implementation)

#### Teleoperation Mode
```bash
# Launch teleoperation system
ros2 launch gps_publisher teleop_launch.py
```

**Active Nodes in teleop_launch.py:**
- All GPS and sensor nodes
- Manual teleoperation control
- Data logging capabilities

### Individual Node Testing
```bash
# Test individual components
ros2 run arduino_reader_cpp ard_read_exe
ros2 run gps_publisher_cpp gps_fus
ros2 run deadreck_cpp dead_reckoning_node
ros2 run navigator_cpp gps_navigation_node
```

### GUI Monitoring (Local Machine)
```bash
# Run GUI for real-time monitoring
ros2 run gui gui_node
```

## Configuration

### Launch File Customization
The system uses three main launch configurations:
- `navigation_launch_cpp.py`: Full autonomous navigation
- `gps_launch_cpp.py`: GPS testing and calibration
- `teleop_launch.py`: Manual control with sensor logging

### Waypoint Configuration
Waypoints can be loaded from text files in the `get_waypoints/get_waypoints/` directory:
- `apple_logo.txt`: Apple logo pattern
- `Penn_P.txt`: Penn P logo pattern
- `Square.txt`: Simple square pattern
- Custom patterns supported through text file format

## Performance Features

### C++ Implementation Advantages
- **Real-time Performance**: Critical control loops implemented in C++
- **Low Latency**: Direct hardware interface with minimal overhead
- **Deterministic Behavior**: Consistent timing for navigation algorithms
- **Memory Efficiency**: Optimized for embedded systems

### Python Implementation Advantages
- **Rapid Development**: Complex algorithms and GUI components
- **Sensor Integration**: Flexible sensor fusion and data processing
- **Debugging**: Easier troubleshooting and parameter tuning

## Troubleshooting

### Common Issues
1. **Serial Port Access**: Ensure proper permissions for `/dev/ttyRobot1`
2. **GPS Signal**: Verify GPS receivers have clear sky view and correction tower
3. **Network Connectivity**: Check SSH connection for remote operation
4. **Topic Communication**: Use `ros2 topic list` and `ros2 topic echo` for debugging

### Diagnostic Commands
```bash
# Check active nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /gps/data

# Check system performance
ros2 run rqt_graph rqt_graph
```

## Contributing
This project is part of a Senior Design capstone project. For questions or contributions, please contact the development team.

---

**Project Team**: Ethan Ma, Emmet Young, Yein Kung, Eric Wang, Sarah Fahmi, Jake Donnini  
**Institution**: University of Pennsylvania  
**Course**: Senior Design  
**Academic Year**: 2024-2025
