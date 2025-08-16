# Line Follower Robot with Camera and OpenCV

This project implements a camera-based line follower robot in ROS2 + Gazebo using OpenCV.
Unlike traditional IR sensor-based bots, this robot uses a camera to detect and follow a line, making it flexible and scalable for advanced robotics applications.

## Features
📷 Camera-based line detection with OpenCV
🤖 4-wheel differential drive robot
🌍 Custom Gazebo world with a rectangular loop
🎮 Teleoperation support for manual control
🔄 Extendable to real robots with USB/RPi cameras

## Installation & Setup
### Step1: Download & Extract:
Download this repository as a ZIP file
Extract the contents

### Step2: Create a ROS2 Workspace:
`mkdir -p ~/line_follower_ws/src`
`cd ~/line_follower_ws`

### Step3: Add the Robot Package
Copy the extracted boffin_robot folder into your workspace:

`cp -r ~/Downloads/boffin_robot ~/line_follower_ws/src/`

### Step4: Build & Source the Workspace
`cd ~/line_follower_ws`
`colcon build`
`source install/setup.bash`

### Step5: Launch Gazebo with the Line Follower World
`ros2 launch boffin_robot linefollower.launch.py`

👉 This opens Gazebo with a rectangular loop track.

#### Step6: Make the Line Follower Script Executable
Open new terminal 
`cd ~/line_follower_ws`
`chmod +x src/boffin_robot/src/line_follower.py`

### Step7: Run the Gazebo file
Press the play button in Gazebo

## The robot will now detect and follow the line inside Gazebo.

### Repository Structure
├── urdf/               # Robot model (URDF/Xacro files)
├── launch/             # Launch files for Gazebo & nodes
├── scripts/            # Python script (line_follower.py)
├── worlds/             # Custom Gazebo world with track
└── README.md           # Documentation

## Applications

🤖 Autonomous ground robots
🎓 Educational robotics projects
🔍 Vision-based navigation research

## Demo
<img width="1303" height="742" alt="Screenshot from 2025-08-14 02-31-56" src="https://github.com/user-attachments/assets/981caef2-adc8-48b2-8efd-de16588e478a" />


## Tech Stack

ROS2 (Humble/Foxy)
Gazebo (Ignition or Classic)
OpenCV (Python)

## Contribution

Pull requests are welcome! If you have improvements or suggestions, feel free to contribute.
