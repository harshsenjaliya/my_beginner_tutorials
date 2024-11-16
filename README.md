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

## License
  
  This project is licensed under the Apache 2.0 License. Refer to the LICENSE file in this repository for details.




