# My ROS Project

## Installation

```bash
# Clone the repository
git clone https://github.com/your-username/your-repo.git
cd your-repo

# Install dependencies
./install_dependencies.sh

# Build the workspace (for ROS1)
catkin_make

# Source the workspace
source devel/setup.bash


---

## Running the project

For each of the following operations, open one different terminal
### Starting ros
``roscore``
### Running rvo
``roslaunch my_robot_pkg load_urdf.launch``
### Running map publisher
``rosrun my_robot_pkg map_publisher``
### Running planner
``rosrun my_robot_pkg planner``

![Image description](https://github.com/federicomatarante/ROS1-planner/rviz_example.png)
