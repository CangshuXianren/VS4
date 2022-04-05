/************************************************
@jyf
函数名称 ： self_gps2xy
函数功能 ： 自身GPS经纬度换算成x,y坐标
备注 ： 
*************************************************/
///<param name="l">经度</param>
///<param name="B">纬度</param>
///<param name="xc">X坐标</param>
///<param name="yc">Y坐标</param>

#ifndef SELFGPS2XY_H
#define SELFGPS2XY_H

#include <math.h>
#include "ros/ros.h"
#include "custom_messages/vehicle_status.h"
#include "custom_messages/msgNav982Pkt20.h"
#include <sensor_msgs/NavSatFix.h>
#include "novatel_oem7_msgs/INSPVAX.h"
#include "custom_messages/vehicle_status.h"
#include "custom_messages/planning_info.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"

#endif
