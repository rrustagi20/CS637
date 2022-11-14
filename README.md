# CS637 Course Project Repository

Implementing a MPC based Control for autonomous landing on a moving platform. Theory from [here](https://ieeexplore.ieee.org/document/9214043).

## Dependencies
```
$ sudo apt update
$ sudo apt install ros-melodic-husky-gazebo ros-melodic-teleop-twist-keyboard

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin init  # initialize your catkin workspace

$ sudo apt-get install liblapacke-dev
$ git clone https://github.com/catkin/catkin_simple.git
$ git clone https://github.com/ethz-asl/rotors_simulator.git
$ git clone https://github.com/ethz-asl/mav_comm.git
$ git clone https://github.com/ethz-asl/eigen_catkin.git

$ git clone https://github.com/ethz-asl/mav_control_rw.git
``` 
### Installation instructions
```
$ mkdir -p ~/mpc_ws/src
$ cd ~/mpc_ws/src
$ catkin init
$ git clone <repo_link>
$ catkin build
$ echo "source ~/mpc_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

## Notes

- Any mesh-related folder will contain .sdf file, which will be directly accessed by another .sdf when that folder is included.
- If you need to source multiple workspaces, source one of them, then rebuild the other and then try sourcing it. This is because while building, setup.sh is formed according to the prexisting ROS_PACKAGE_PATH
- You can't launch a model file until its world is defined
- optenv is for providing you an option to directly export the variable on BASH, which will be picked up for that attribute, if not available, default value is used
-Ifinclude velocity is included in MPC state vector we might achive more effiecient landing

## Problems

- how is robot_description being passed to spawn_model in spawn_husky.launch
- custom model.urdf and mav.urdf cannot be rendered together in gazebo even though both their robot_description files is being passed.
