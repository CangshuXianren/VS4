#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

ros::Publisher vir_odom_pub;
double v, w;
double yaw = 0.0;
nav_msgs::Odometry vir_odom;

void CmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msgs){
    v = cmd_msgs->linear.x;
    w = cmd_msgs->angular.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "A*_node");
    ros::NodeHandle nh;

    ros::Subscriber mypoint = nh.subscribe("/cmd_vel", 2, CmdCallback);

    vir_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);

    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        vir_odom.pose.pose.position.x += v * 0.01 * cos(yaw);
        vir_odom.pose.pose.position.y += v * 0.01 * sin(yaw);
        yaw += w * 0.01;
        rate.sleep();
    }

    return 0;
}