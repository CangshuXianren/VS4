#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

ros::Publisher vir_odom_pub;
double v, w;
double yaw = 0.0;
nav_msgs::Odometry vir_odom;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_environment");
    ros::NodeHandle nh;

    vir_odom_pub = nh.advertise<visualization_msgs::Marker>("obstacle_info", 1);

    vir_odom.header.frame_id = "map";
    vir_odom.header.stamp = ros::Time::now();

    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}