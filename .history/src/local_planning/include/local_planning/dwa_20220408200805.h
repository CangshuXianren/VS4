/*
@jyf
功能包名称 ： local_planning
函数功能 ：局部轨迹规划
备注 ： DWA算法中所有方向都为角度制！！！
*/
#ifndef DWA_H
#define DWA_H


#include <vector>
#include <iostream>
#include <algorithm>
#include <math.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <custom_messages/mapmodel.h>
#include <custom_messages/planning_info.h>
#include <custom_messages/vehicle_status.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/PoseArray.h"
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
using namespace std;
#define PI 3.1415926

struct CarState
{
    double x;
    double y;
    double yaw;
    double speed;
    double angular_speed;
    CarState(){};
    CarState(double x_, double y_, double yaw_, double speed_, double angular_speed_):
        x(x_), y(y_), yaw(yaw_), speed(speed_), angular_speed(angular_speed_)
    {}
};

class DWA
{
public:
    DWA(ros::Publisher pub);
    void PathCallback(const nav_msgs::Path::ConstPtr& path_msg);
    // void SelfCallback(const custom_messages::vehicle_status::ConstPtr& self_msg);
    void SelfCallback(const nav_msgs::Odometry::ConstPtr& self_msg);
    // void SelfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init_loc_msg);
    void MapModelCallback(const custom_messages::mapmodel::ConstPtr& mapmodel_msg);
    void CmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
    void planning();
    vector<double> dwa_control(const CarState &carstate);
    vector<double> calc_dw(const CarState &carstate);
    vector<double> calc_best_speed(const CarState &carstate, const vector<double> &dw);
    void predict_trajectory(const CarState &carstate, const double &speed, const double &angular_speed, vector<CarState> &trajectory);
    CarState motion_model(const CarState &carstate, const double &speed, const double &angular_speed);
    double calc_goal_cost(const vector<CarState> &trajectory);
    double calc_obstacle_cost(const vector<CarState> &trajectory);
    double calc_journey_cost(const vector<CarState> &trajectory);

    vector<vector<CarState>> trajectory;
    ros::Publisher trajectory_pub;
    CarState mystate;
    CarState myend;
    nav_msgs::Path ends;
    custom_messages::mapmodel mymapmodel;
    int iteration = 0; // 路径点索引
    bool first = true;
};

#endif // DWA_H
