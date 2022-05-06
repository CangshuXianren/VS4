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

    ros::Publisher virtual_environment_pub = nh.advertise<visualization_msgs::Marker>("virtual_obstacle_info", 1);

    ros::Rate rate(100);
    while(ros::ok())
    {
        custom_messages::mapmodel vir_mapmodel;
        geometry_msgs::Point tmp1;
        tmp1.x = 3.0;
        tmp1.y = 0.0;
        tmp1.z = 0.2;
        vir_mapmodel.mapinfo.push_back(tmp1);
        geometry_msgs::Point tmp2;
        tmp2.x = 5.0;
        tmp2.y = 4.5;
        tmp2.z = 0.3;
        vir_mapmodel.mapinfo.push_back(tmp2);
        geometry_msgs::Point tmp3;
        tmp3.x = 6.0;
        tmp3.y = 6.0;
        tmp3.z = 0.2;
        vir_mapmodel.mapinfo.push_back(tmp3);
        // geometry_msgs::Point tmp1;
        // tmp1.x = 3.0;
        // tmp1.y = 0.0;
        // tmp1.z = 0.2;
        // vir_mapmodel.mapinfo.push_back(tmp1);
        virtual_environment_pub.publish(vir_mapmodel);
        rate.sleep();
    }

    return 0;
}