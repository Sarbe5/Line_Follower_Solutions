# Line Follower Robot Simulation (ROS 2 + Gazebo)

This project implements a simple **line-following robot** in **ROS 2 Jazzy** with **Gazebo Harmonic** simulation.  
The robot uses a **front-downward-facing camera** and a custom **Python controller node** (OpenCV-based) to detect and follow a black line on the ground.

---

## 📦 Specifications

- **OS**: Ubuntu 24.04  
- **ROS 2**: Jazzy  
- **Simulator**: Gazebo Harmonic  
- **Control Framework**: `ros2_control` with `diff_drive_controller`  
- **Computer Vision**: OpenCV (for line detection)


---

## 🤖 Robot Description

- **Base**: Simple box-shaped chassis  
- **Wheels**: Two cylindrical rear wheels + one spherical caster wheel  
- **Sensor**: Front-facing camera (downward angled, detects line)  
- **Control**: Differential drive via `cmd_vel` (ROS 2 standard interface)

The robot is controlled through [`diff_drive_controller`](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html), which provides a `cmd_vel` interface for linear and angular velocity commands.

---

## 🧠 Control Logic

1. **Camera Input**  
   - Captures image of the ground.  
   - Converts BGR → HSV color space.  
   - Applies HSV threshold to isolate the **black line**.

2. **Mask & ROI**  
   - Creates a binary mask (`cv2.inRange`).  
   - Restricts detection to a **Region of Interest (ROI)** near the bottom of the image (where the robot expects the line).

3. **Centroid Calculation**  
   - Uses `cv2.moments()` to compute centroid of the line in ROI.  
   - Compares centroid `cX` with image center to determine line position error.

4. **Proportional Control**  
   - Angular velocity = `-Kp * error`  
   - Linear velocity = constant forward speed  
   - Robot adjusts heading until the line is centered.

---

## 🚀 Running the Simulation

Build and source workspace. Then,
```bash
ros2 launch my_robot_description display.launch.xml
ros2 run rqt_image_view rqt_image_view # optional to visualize image feed
ros2 run my_py_pkg line_follower_controller
