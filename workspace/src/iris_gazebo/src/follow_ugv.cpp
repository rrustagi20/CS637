#include <iris_gazebo/follow_ugv.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "follow_ugv");
    ros::NodeHandle nh;

    ros::Subscriber uav_pose_sub;
    ros::Subscriber ugv_pose_sub = nh.subscribe("gazebo/model_states", 10, &get_pose_ugv);
    ros::Publisher trajectory_pub;

    std::string mavPoseTopic, mavCmdTrajectoryTopic;
    if (nh.getParam("mav_pose", mavPoseTopic))
        uav_pose_sub = nh.subscribe(mavPoseTopic, 10, &get_pose_uav);
    else
        uav_pose_sub = nh.subscribe("/firefly/ground_truth/pose", 10, &get_pose_uav);
    if (nh.getParam("mav_cmd_trajectory", mavCmdTrajectoryTopic))
        trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mavCmdTrajectoryTopic, 10);
    else
        trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 10);

    ROS_INFO("Started UAV trajectory.");
    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(3.0).sleep();
    ROS_INFO("Started Publishing\n");

    ros::Rate rate(20);
    while (ros::ok())
    {
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint pt;
        geometry_msgs::Transform tf;

        float x = (ugv.x - uav.x) / 2 + ugv.x;
        float y = (ugv.y - uav.y) / 2 + ugv.y;
        float z = (ugv.z - uav.z) / 2 + ugv.z;
        float xy_radius = x * x + y * y;
        history_xy.push(xy_radius);
        float popped = 0;
        if (history_xy.size() > buffer_size_queue)
        {
            popped = history_xy.front();
            history_xy.pop();
        }
        sum_xy += xy_radius;
        sum_xy -= popped;

        if (abs(sum_xy / buffer_size_queue - xy_radius) < 5e-2)
        {
            tf.translation.z = z;
            x = ugv.x + gain_factor_for_push * (ugv.x - ugv2.x), y = ugv.y + gain_factor_for_push * (ugv.y - ugv2.y);
        }
        else
        {
            tf.translation.z = ugv.z + 0.4; // for maintaining a fixed height until hovering just above the platform
        }

        tf.translation.x = x;
        tf.translation.y = y;
        ROS_INFO("UGV: %f %f %f", ugv.x, ugv.y, ugv.z);
        ROS_INFO("UAV: %f %f %f", uav.x, uav.y, uav.z);
        ROS_INFO("To go to: %f %f %f", x, y, z);
        if (tf.translation.z < 0.45)
            ROS_DEBUG("Below threshold height!");

        pt.transforms.push_back(tf);
        trajectory_msg.points.push_back(pt);
        trajectory_pub.publish(trajectory_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}