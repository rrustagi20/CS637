#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <queue>
#include <string>
#include <geometry_msgs/Transform.h>

struct coord
{
    float x, y, z;
};

const int husky_index = 2;
int buffer_size_queue = 3;
float gain_factor_for_push = 200.0;
std::queue<float> history_xy;
float sum_xy = 0;

coord ugv, ugv2, uav;

void get_pose_ugv(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    ugv2.x = ugv.x;
    ugv2.y = ugv.y;
    ugv.x = msg->pose[husky_index].position.x;
    ugv.y = msg->pose[husky_index].position.y;
    ugv.z = msg->pose[husky_index].position.z + 0.324; // value according to height of ugv
}

void get_pose_uav(const geometry_msgs::Pose::ConstPtr &msg)
{
    uav.x = msg->position.x;
    uav.y = msg->position.y;
    uav.z = msg->position.z;
}