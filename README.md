# simulate-navigation-ros2

# 🧠 FSM-Based TurtleBot3 Navigation with Visual Tracking and Obstacle Avoidance (ROS2 Foxy)

---

## 🛠️ Features

- FSM-based intelligent navigation
- Obstacle avoidance using LiDAR (`/scan`)
- Visual object tracking using camera image processing:
  - Follows red objects when detected
  - Prioritizes green if both red and green are present
- Switches between search, track, and avoid states dynamically
- ROS2 Foxy-compatible and built with `colcon`

---

## 📦 Dependencies

Install the following before running the project:

```bash
sudo apt update
sudo apt install -y \
  ros-foxy-desktop \
  ros-foxy-cv-bridge \
  ros-foxy-image-transport \
  python3-colcon-common-extensions \
  python3-opencv \
  gazebo11 \
  ros-foxy-gazebo-ros-pkgs \
  ros-foxy-turtlebot3* \
  libopencv-dev \
  git
```

⚠️ **Make sure to set the TurtleBot3 model:**

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## 📁 Project Structure

```
ros2_ws/
├── src/
│   └── fsm_navigation/
│       ├── fsm_navigation/
│       │   ├── __init__.py
│       │   ├── fsm_navigation_node.py
│       ├── launch/
│       │   └── fsm_nav.launch.py
│       ├── worlds/
│       │   └── red_ball.world
│       ├── models/
│       │   └── red_ball/
│       │       ├── model.sdf
│       │       └── model.config
│       ├── package.xml
│       └── setup.py
```

## 🚀 Build and Run

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/simulate-navigation-ros2.git fsm_navigation
cd ~/ros2_ws
```

### 2. Build the Workspace

```bash
colcon build
source install/setup.bash
```

### 3. Launch the World with Red Ball and Obstacles

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=$HOME/ros2_ws/src/fsm_navigation/worlds/red_ball.world
```

### 4. Run the FSM Navigation Node

```bash
ros2 run fsm_navigation fsm_navigation_node
```

## 🧠 FSM Logic

- **SEARCH**: Wander forward, avoid obstacles
- **TRACK_RED / TRACK_GREEN**: Turn and approach colored object
- **AVOID**: Turns away if an obstacle is within 0.3 meters
- **REACHED**: Stops when close to red ball

All decisions are made using ROS topics:

- `/scan` → Obstacle detection via LaserScan
- `/camera/image_raw` → Object tracking via OpenCV + cv_bridge

## 🧪 Testing Tips

**Add more red balls via:**

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity red_ball_2 \
  -file ~/.gazebo/models/red_ball/model.sdf \
  -x 2 -y 2 -z 0.1
```

**View image stream:**

```bash
rqt_image_view /camera/image_raw
```

**Check velocity:**

```bash
ros2 topic echo /cmd_vel
```
