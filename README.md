# My_Beginner_Tutorials

## Overview
This project contains a simple package, `beginner_tutorials`, to demonstrate publisher and subscriber nodes in ROS 2 Humble. This package includes a custom message publisher and a subscriber node that listens to the published messages.



## Author
Harsh Senjaliya (hsenjali@umd.edu)

## Prerequisites
Please ensure that the following are set up as prerequisites for building and running this package:

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **colcon** (for building the workspace)
- **clang tools** (for linting and code quality checks)

## Cloning and Building the Package

1. **Source ROS 2 Underlay**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Create a New Workspace**:
    ```bash
    mkdir -p ~/my_beginner_tutorials/src
    cd ~/my_beginner_tutorials/src
    ```
3. **Clone the Repository**:
   ```bash
   git clone https://github.com/harshsenjaliya/my_beginner_tutorials.git
   ```
4. **Build the Workspace with colcon**:
   Build the Workspace with colcon
   ```bash
   colcon build
   ```
5. **Source Overlay**:
   ```bash
   source install/setup.bash
   ```
## Running the Nodes
**Run the Publisher Node (Talker)**
```bash
ros2 run beginner_tutorials talker
```
**Run the Subscriber Node (Listener)**
```bash
# Source ROS 2 underlay
source /opt/ros/humble/setup.bash

# Source the workspace overlay
cd ~/ros2_ws
source install/setup.bash

# Run the subscriber node
ros2 run beginner_tutorials listener
```


