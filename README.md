# Setup Instructions

## Environment Setup

### 1. Clone the Project

```bash
git clone -b jazzy https://github.com/Armangb1/roboomni.git
```

### 2. Download Repositories

```bash
cd roboomni/src
vcs import < .repos
```

### 3. Build the Workspace

```bash
cd ..
colcon build
```

### 4. (Optional) Micro-ROS Setup

If you want to use Micro-ROS:

```bash
source install/setup.bash
```

#### Build Micro-ROS Agent

```bash
ros2 run micro_ros_setup create_agent_ws.sh src/third_party
colcon build
```
