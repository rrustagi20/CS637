# CS637 Course Project Repository

Implementing a MPC based Control for autonomous landing on a moving platform. Theory from [here](https://ieeexplore.ieee.org/document/9214043).

## Notes

- Any mesh-related folder will contain .sdf file, which will be directly accessed by another .sdf when that folder is included.
- If you need to source multiple workspaces, source one of them, then rebuild the other and then try sourcing it. This is because while building, setup.sh is formed according to the prexisting ROS_PACKAGE_PATH
- You can't launch a model file until its world is defined
- optenv is for providing you an option to directly export the variable on BASH, which will be picked up for that attribute, if not available, default value is used

## Problems

- how is robot_description being passed to spawn_model in spawn_husky.launch
