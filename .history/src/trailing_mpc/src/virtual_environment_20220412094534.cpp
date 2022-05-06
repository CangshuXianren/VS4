#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "custom_messages/mapmodel.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_environment");
    ros::NodeHandle nh;

    ros::Publisher virtual_environment_pub = nh.advertise<visualization_msgs::Marker>("obstacle_info", 1);

    ros::Rate rate(100);
    while(ros::ok())
    {
        custom_messages::map vir_mapmodel;
        geometry_msgs::Point tmp1;
        tmp1.x = 3.0;
        tmp1.y = 0.0;
        tmp1.z = 0.2;
        vir_mapmodel.states.push_back(tmp1);
        geometry_msgs::Point tmp1;
        tmp1.x = 3.0;
        tmp1.y = 0.0;
        tmp1.z = 0.2;
        vir_mapmodel.states.push_back(tmp1);
        geometry_msgs::Point tmp1;
        tmp1.x = 3.0;
        tmp1.y = 0.0;
        tmp1.z = 0.2;
        vir_mapmodel.states.push_back(tmp1);
        geometry_msgs::Point tmp1;
        tmp1.x = 3.0;
        tmp1.y = 0.0;
        tmp1.z = 0.2;
        vir_mapmodel.states.push_back(tmp1);
        virtual_environment_pub.publish(vir_mapmodel);
        rate.sleep();
    }

    return 0;
}