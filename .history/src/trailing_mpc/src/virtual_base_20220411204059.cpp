#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

ros::Publisher vir_odom_pub;
double v, w;
nav_msgs::Odometry vir_odom;

void CmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msgs){

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "A*_node");
    ros::NodeHandle nh;

    ros::Subscriber mypoint = nh.subscribe("/cmd_vel", 2, CmdCallback);

    vir_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);

    nh.param("overall_length", overall_length, 1.0);

    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        
        rate.sleep();
    }

    return 0;
}