#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <queue>
#include <geometry_msgs/Transform.h>

int buffer_size_queue = 3;
bool flag = false;
float gain_factor_for_push = 200.0;
const int husky_index = 2;
std::queue<float> history_xy;
float sum_xy = 0;

float x_ugv2, y_ugv2;
float x_ugv = 0, y_ugv = 0, z_ugv = 0.245 * 2, x_uav, y_uav, z_uav;
void get_pose_ugv(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    x_ugv2 = x_ugv, y_ugv2 = y_ugv;
    x_ugv = msg->pose[husky_index].position.x;
    y_ugv = msg->pose[husky_index].position.y;
    // z_ugv = msg->pose[husky_index].position.z;
}

void get_pose_uav(const geometry_msgs::Pose::ConstPtr &msg)
{
    x_uav = msg->position.x;
    y_uav = msg->position.y;
    z_uav = msg->position.z;
}