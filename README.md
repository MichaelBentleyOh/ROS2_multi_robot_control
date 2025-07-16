# ROS2 Multi-Robot Control Example

This repository provides an example ROS 2 package for controlling multiple robots.

The original package can be found here:  [Original GitLab Repository](https://gitlab.ensta-bretagne.fr/zerrbe/ign-ros2-multi-robots-control-update/-/tree/master/simple_test/resource?ref_type=heads)

This revised version enhances the original package to fully utilize the ROS 2 controller framework, focusing on stability and ease of expansion.
The original revises "gz_ros2_control" but this repo doesn't have to revise any code given by binary execution.

I think this approach is more convinent for beginners like me.

---

## How to Use

Follow the steps below to build and run the package.

---

### 0. Create a Workspace

```bash
mkdir -p ~/multi_robot_example_ws/src
cd ~/multi_robot_example_ws/src
```

---

### 1. Clone this Repository

```bash
git clone https://github.com/MichaelBentleyOh/ROS2_multi_robot_control.git
```

---

### 2. Build the Package

Navigate to the workspace root and build:

```bash
cd ~/multi_robot_example_ws
colcon build --symlink-install
```

---

### 3. Source the Workspace

Source the workspace environment before running ROS 2 nodes:

```bash
source install/setup.bash
```

---

### 4. Launch the Example

Run the multi-robot launch file:

```bash
ros2 launch simple_test simple_diff_drive_two_robots.launch.py
```

---
---

### 5. Analyse this package and apply to your project

Utilize this example as a first step for your project!!!!

---

## Notes

- The package uses ROS 2 controllers for managing robot control interfaces.
- Designed for easy extension to additional robots or controllers.
- email : michael1015999@gmail.com
- maintainer : minsik oh
