/************************************************
@jyf
函数名称 ： front_gps2xy
函数功能 ： 前车GPS经纬度换算成x,y坐标
备注 ： 转换函数transform()定义注释在self_gps2xy.cpp中
*************************************************/
///<param name="l">经度</param>
///<param name="B">纬度</param>
///<param name="xc">X坐标</param>
///<param name="yc">Y坐标</param>

#ifndef FRONTGPS2XY_H
#define FRONTGPS2XY_H

#include <math.h>
#include "ros/ros.h"
#include "custom_messages/vehicle_status.h"
#include <sensor_msgs/NavSatFix.h>
#include <LinktrackNodeframe0.h>

class GPS2XY
{
public:

    GPS2XY(ros::Publisher pub);

    ~GPS2XY();

    void transformCallback(const nlink_parser::LinktrackNodeframe0::ConstPtr& msg);

//变量
    ros::Publisher pub;
};


#endif