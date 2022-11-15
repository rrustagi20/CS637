# CS637 Course Project Repository

Implementing a MPC based Control for autonomous landing on a moving platform. Theory from [here](https://ieeexplore.ieee.org/document/9214043).

## Landing on static platform

https://user-images.githubusercontent.com/77167720/201858564-995e0bc0-8bc6-4767-8ce9-386ec0fc6fce.mp4

## Installation and Dependencies

You need **ROS Melodic/Noetic** to run this workspace.
To install, please refer [here](http://wiki.ros.org/noetic/Installation) for installing desktop-full version.
If you're installing ROS Melodic, please change the version name accordingly below.

```bash
sudo apt update
sudo apt-get install liblapacke-dev
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
```
Install ```vcstool``` for merging dependencies into the workspace
```bash
sudo apt install python3-vcstool
```
The following is to setup this root folder:

```bash
cd workspace
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin init  # initialize your catkin workspace

cd ../
mkdir -p workspace2/src
cd workspace2/ # for dependency only  catkin workspace
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin init

# Merge the dependencies into workspace
vcs import src < ../install/ssh.rosinstall

catkin build -j8 # wait till it finishes building, usually takes ~10minutes
source devel/setup.bash

cd ../workspace
catkin build # should build within 1 minute
source devel/setup.bash
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc

echo $ROS_PACKAGE_PATH | tr ':' '\n' # confirm that both workspace packages are in ROS Path
```

## Running simulation

Once the above has been successfully completed, open a series of terminals for the following codeblocks:

```bash
# Terminal 1
roslaunch rotors_gazebo mav.launch
```

This launches firefly, the mav which we'll be using for the simulation.

```bash
# Terminal 2
roslaunch platform_description spawn_platform.launch # for static platform
# OR
roslaunch platform_gazebo trajectory_follower.launch # for moving platform
```

This launches a static platform OR a platform that should start moving in a semi-random circular path.

```bash
# Terminal 3
roslaunch mav_linear_mpc mav_linear_mpc_sim.launch mav_name:=firefly
```

This launches the MPC Controller used for position control of firefly. Note that MPC itself has been implemented by [ETHZ](https://github.com/ethz-asl/mav_control_rw.git).

```bash
# Terminal 4
rosservice call /firefly/takeoff
rosrun iris_gazebo follow_ugv_node
```

Man and have the MAV takeoff through the rosservice. Now run the node `follow_ugv_node` that makes the mav land over the platform.

## Notes

- Any mesh-related folder will contain .sdf file, which will be directly accessed by another .sdf when that folder is included.
- If you need to source multiple workspaces, source one of them, then rebuild the other and then try sourcing it. This is because while building, setup.sh is formed according to the prexisting ROS_PACKAGE_PATH
- You can't launch a model file until its world is defined
- optenv is for providing you an option to directly export the variable on BASH, which will be picked up for that attribute, if not available, default value is used
- Currently we feed only position information to the mpc_controller. If velocity is also included in the mpc state vector then we might achieve more effecient landing (in the case of moving platform)

## Problems

- <s>how is robot_description being passed to spawn_model in spawn_husky.launch</s>: robot_description is being saved at the parameter server, and it is being passed from an inner launch file
- <s>custom model.urdf and mav.urdf cannot be rendered together in gazebo even though both their robot_description files is being passed.</s>: Actually the problem was the model path was being given incorrectly, and the topic published to was incorrect as well.
