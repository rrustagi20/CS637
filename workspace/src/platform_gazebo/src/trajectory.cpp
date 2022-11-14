/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
    */
// #define OMEGA 1.0
#define FORWARD_VEL 0.9
#define ANGULAR_VEL 0.18

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

geometry_msgs::Twist curr_pose;

void poseCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    curr_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "platform_trajectory");
    ros::NodeHandle nh;
    std::string velocityControllerTopic;
    ros::Publisher trajectory_pub;
    bool gotTopic = nh.getParam("platform_cmd_vel", velocityControllerTopic);
    if (gotTopic)
        trajectory_pub = nh.advertise<geometry_msgs::Twist>(velocityControllerTopic, 10);
    else
    {
        ROS_INFO("Didn't find parameter platform_cmd_vel");
        trajectory_pub = nh.advertise<geometry_msgs::Twist>("/platform/husky_velocity_controller/cmd_vel", 10);
    }
    ROS_INFO("Started platform trajectory.");

    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(3.0).sleep();
    ROS_INFO("Started Publishing\n");
    geometry_msgs::Twist trajectory_msg;
    // ros::Duration(5.0).sleep();

    ros::Rate rate(100.0);
    while (ros::ok())
    {
        trajectory_msg.linear.x = FORWARD_VEL;
        trajectory_msg.angular.z = ANGULAR_VEL;
        trajectory_pub.publish(trajectory_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
