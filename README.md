
## Setup Instructions

1. **Create a workspace and clone the project**

    ```bash
    mkdir mecanum_ws
    cd mecanum_ws
    git clone --recursive https://github.com/Armangb1/roboomni.git .
    ```

2. **Clone the micro-ROS repository**

    ```bash
    git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    ```

3. **Install package dependencies using rosdep**

    ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

4. **Build packages and create the agent workspace**

    ```bash
    colcon build
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    ```

5. **Source and launch the system**

    ```bash
    source install/setup.bash
    ros2 launch mecanum_bringup system_bringup.launch.py
    ```

