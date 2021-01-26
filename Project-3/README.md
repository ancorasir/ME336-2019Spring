# BionicDL-CobotLearning-Project3
The project is illustrated with Aubo-i5 and realsense 400 series. If you are doing the project with a different robot arm, just replacing the aubo_robot directory with you robot arm's ROS package and lanuch the corresponding moveit group node.

# Project Setup
Before running the project, make sure you've installed ROS kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu) and catkin_tools (https://catkin-tools.readthedocs.io/en/latest/installing.html).

Please go to the each package under this repository for detailed explanations and installation guidance. Please install the [Realsense SDK](https://realsense.intel.com/sdk-2/#install) before you run the following steps.

For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/ancorasir/BionicDL-CobotLearning-Project3.git
```

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Build the project:
```sh
$ cd ~/catkin_ws
$ catkin build aubo_msgs
$ catkin build
```

Add following to your .bashrc file:
```
source ~/catkin_ws/devel/setup.bash
```

# Run hand eye calibration by stick a checkerboard on the center of the flange of the robot arm
The calibration results of the above methods is not good enough for usage. We recommend option2 method if you have you camera installed fixed reference to the robot base.
1. Launch the robot's movegroup node to monitor, control and visualize the robot. Please check the ip of you robot and replace the default value below.
```sh
$ roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.102
```

2. Publish the handeye transformation to /tf, run:
```sh
$ python calculate_publish_handeye_matrix.py
```

3. Complete the python script of minimax for tic tac toe game in /BionicDL-CobotLearning-Project3/aubo_robot/aubo_i5_moveit_config/scripts/game.py

4. Complete the main python script of the robot player in  /BionicDL-CobotLearning-Project3/aubo_robot/aubo_i5_moveit_config/scripts/project_grasp_3x3.py
