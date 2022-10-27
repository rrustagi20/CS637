#include <iris_gazebo/follow_ugv.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "follow_ugv");
    ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");
    ros::Subscriber ugv_pose_sub = nh.subscribe("gazebo/model_states", 10, &get_pose_ugv);
    ros::Subscriber uav_pose_sub = nh.subscribe("/firefly/ground_truth/pose", 10, &get_pose_uav);
    ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 10);
    ROS_INFO("Started UAV trajectory.");

    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(3.0).sleep();
    ROS_INFO("Started Publishing\n");

    // ros::Duration(5.0).sleep();

    ros::Rate rate(20);
    while (ros::ok())
    {
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint pt;
        geometry_msgs::Transform tf;
        float x = (x_ugv - x_uav) / 2 + x_ugv, y = (y_ugv - y_uav) / 2 + y_ugv, z = (z_ugv - z_uav) / 2 + z_ugv;
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

        if (abs(sum_xy / buffer_size_queue - xy_radius) < 1e-1)
        {
            tf.translation.z = z;
            x = x_ugv + gain_factor_for_push * (x_ugv - x_ugv2), y = y_ugv + gain_factor_for_push * (y_ugv - y_ugv2);
        }
        else
            tf.translation.z = z_uav;

        tf.translation.x = x;
        tf.translation.y = y;
        ROS_INFO("UGV: %f %f %f", x_ugv, y_ugv, z_ugv);
        ROS_INFO("UAV: %f %f %f", x_uav, y_uav, z_uav);
        ROS_INFO("To go to: %f %f %f", x, y, z);
        if (tf.translation.z < 0.45)
            ROS_DEBUG("Below threshold!");
        pt.transforms.push_back(tf);
        trajectory_msg.points.push_back(pt);
        trajectory_pub.publish(trajectory_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}