#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_environment");
    ros::NodeHandle nh;

    ros::Publisher virtual_environment_pub = nh.advertise<visualization_msgs::Marker>("obstacle_info", 1);

    ros::Rate rate(100);
    while(ros::ok())
    {
        visualization_msgs::Marker node_vis;
        node_vis.header.frame_id = "map";
        node_vis.header.stamp = ros::Time::now();

        node_vis.ns = "perceive/obs";
        
        node_vis.type = visualization_msgs::Marker::CYLINDER; // type表示物体类型，CUBE_LIST表示立方体列表是一系列立方体，除了位姿所有的属性都一样。使用这个物体类型替代 visualization_msgs/MarkerArray允许rviz批处理显示，这让他们的显示更快。附加说明是它们所有都必须有相同的颜色/尺寸。
        node_vis.action = visualization_msgs::Marker::ADD; // 0=add/modify an object, 1=(deprecated), 2=deletes an object, 3=deletes all objects
        node_vis.id = 0; // 分配给marker的唯一的id

        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;

        // 调整颜色
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
        node_vis.scale.x = 2*0.2; // marker的尺寸，在位姿/方向之前应用，[1,1,1]意思是尺寸是1m*1m*1m
        node_vis.scale.y = 0.1;
        node_vis.scale.z = 1.0;
        geometry_msgs::Point pt;
        pt.x = 3.0;
        pt.y = 0.0;

        node_vis.points.push_back(pt);
        virtual_environment_pub.publish(node_vis);
        rate.sleep();
    }

    return 0;
}