#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "custom_messages/planning_info.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_environment");
    ros::NodeHandle nh;

    ros::Publisher virtual_environment_pub = nh.advertise<visualization_msgs::Marker>("obstacle_info", 1);

    ros::Rate rate(100);
    while(ros::ok())
    {
        custom_messages::planning_info vir_mapmodel;
        custom_messages::vehicle_status
        virtual_environment_pub.publish(node_vis);


        rate.sleep();
    }

    return 0;
}