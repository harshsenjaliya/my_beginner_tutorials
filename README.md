# my_beginner_tutorials

## Overview
This project offers beginner-friendly tutorials for ROS 2 Humble, showcasing a simple publisher-subscriber structure to illustrate key ROS 2 functions. It features a Talker node that publishes messages to a topic and a Listener node that subscribes to the same topic. Additionally, the Talker node includes a service that enables real-time modification of the published message, and extensive logging is implemented across all five ROS 2 logging levels (DEBUG, INFO, WARN, ERROR, FATAL) for detailed feedback. A launch file allows users to adjust parameters, such as publishing frequency, via command-line inputs, making this package a practical starting point for understanding ROS 2 essentials.

## Author 
Harsh Senjaliya (hsenjali@umd.edu)

## Prerequisites
- ROS2 Humble
- Colcon build tool
- C++17 compatible compiler

## Installation and Building Instructions

1. **Set Up the Folder Structure**
    ```bash 
    cd 
    mkdir -p ros2_ws/src/beginner_tutorials 
    cd ~/ros2_ws/src/beginner_tutorials 
    ```

2. **Download the Source Code**
   Extract the downloaded file (zip or tar.gz) and place its contents into this directory: `ros2_ws/src/beginner_tutorials`.

3. **Check and Install Dependencies**

   Navigate to the workspace root and run rosdep:

    ```bash
    cd ~/ros2_ws 
    rosdep install -i --from-path src --rosdistro humble -y
    ```

4. **Build the Package**
   Use colcon to build:

    ```bash
    colcon build --packages-select beginner_tutorials
    ```

5. **Source the Environment**

    Run the setup script to overlay this workspace:

    ```bash
    source install/setup.bash
    ```

## Running the Package 

1. **Run the Publisher Node** 

    Start the talker node with:

    ```bash
    ros2 run beginner_tutorials talker --ros-args --log-level debug
    ```

2. **Run the Subscriber Node**

    Start the listener node with:

    ```bash
    ros2 run beginner_tutorials listener --ros-args --log-level debug
    ```

3. **Run the Server Client Node**

    Launch the server_client node with custom output:

    ```bash
    ros2 run beginner_tutorials server_client "Changed Output" --ros-args --log-level debug
    ```

## Using the Launch File

**Launching with Custom Parameters**

To start nodes using a custom launch file and parameters (e.g., adjusting frequency):

```bash
ros2 launch beginner_tutorials custom_launch.yaml frequency:=1
```

Ensure that the frequency value is an integer. This launches the publisher, subscriber, and server nodes within the publisher. The service client must be run separately to modify the message (see the above step for running the service client).

## Node Desciption 
**Talker Node**
  
  The talker node publishes messages to the /topic topic, acting as the main publisher and broadcasting a customizable string message that can be modified via the server_client node's requests.

**Listener Node**
  
  The listener node subscribes to the /topic topic, receiving and displaying messages from the talker node, thereby allowing real-time observation of message updates.

**Server Client Node**
  
  This node acts as a client to the embedded server within the talker node, sending requests to change the published message content.

**Server Node**
  
  The server resides in the talker node, handling requests from the server_client node and updating the message broadcasted by the talker node dynamically.

## Using ROS2 launch file

You can use a launch file to launch both the talker and listener nodes at the same time. The launch files launches the talker node with a default frequency of 2 Hz. It also publishes a static tf broadcast from 'world' frame to 'talker' frame

```bash
ros2 launch beginner_tutorials talker_listener_launcher.launch.py 
```

### Using launch arguments to modify frequency or record rosbag

As we know, the talker node has a parameter called 'freq' that can be used to set the frequency of publishing in Hz, we can use the following command to launch the nodes as well as set the frequency using launch files.

The below command will start the node with a frequency of 5.0 Hz and will also record a ros bag

```bash
# Usage: ros2 launch beginner_tutorials talker_listener_launcher.launch.py freq:=<New_Frequency> record_bag:=<true/false>
ros2 launch beginner_tutorials talker_listener_launcher.launch.py freq:=5.0 record_bag:=true
```

## ROS2 Logging demo

In the given nodes, I have implemented various levels of logging. In the screenshot below, we can see that there are DEBUG and ERROR level logs registered in rqt_console.

![rqt console image](./results/rqt_console_1.png)

The warning message is generated because the listener node is run before the talker node and it's not able to find the subscribed topic.

I have also printed out a DEBUG log giving out the current publisher rate.

## ROS bag file and replaying the topics

The launch file contains the parameter "record_bag" if you want to record a bag file. By default, the bag files are stored in the bag_files directory in the package. To play and verify the rosbag file, follow the following steps.

In one terminal, run the following commands

```bash
# Source underlay
source /opt/ros/humble/setup.bash

# Source overlay
cd ~/ros2_ws
source install/setup.bash

# Run the Subscriber node
ros2 run beginner_tutorials listener
```

In another terminal, run the following commands to play the bag file

```bash
cd ~/ros2_ws/src/my_beginner_tutorials/bag_files

# Usage ros2 bag play </BagFile>
ros2 bag play rosbag2_2024_11_15-00_45_33/
```

## Visualizing frames using tf2

The talker node publishes a static tf transform from 'world' frame to 'talker' frame. To inspect the tf frame. Run the following commands.

### Run a talker node

```bash
ros2 run  beginner_tutorials talker
```

In another terminal, run

```bash
ros2 run tf2_ros tf2_echo world talker
```

It should generate an output like

```bash
At time 1731652060.106289092
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.707, 0.707, 0.000, 0.000]
- Rotation: in RPY (radian) [3.142, -0.000, 1.571]
- Rotation: in RPY (degree) [180.000, -0.000, 90.000]
- Matrix:
  0.000  1.000  0.000  1.000
  1.000 -0.000  0.000  2.000
  0.000  0.000 -1.000  3.000
  0.000  0.000  0.000  1.000
At time 1731652061.106147762
```

Alternatively, you can also run the following command to generate a pdf

```bash
ros2 run tf2_tools view_frames
```

## Run Integration test using Catch2

To run integration test, using Catch2. Run the following commands.

```bash
colcon test --packages-select beginner_tutorials
cat log/latest_test/beginner_tutorials/stdout_stderr.log
```

## ROS2 Resource

More ROS2 resources can be found at [ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials.html) page

## License
  
  This project is licensed under the Apache 2.0 License. Refer to the LICENSE file in this repository for details.
