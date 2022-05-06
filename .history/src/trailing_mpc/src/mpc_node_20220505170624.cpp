#include "quadprog/Array.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include <queue>
#include <time.h>
#include "quadprog/QuadProg++.h"
#include "trailing_mpc/mpc.h"
#include "math.h"
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <iterator>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "custom_messages/planning_info.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// launch参数
float T = 0.05;     // T为单位采样时间
int _Nx, _Nu, _Np, _Nc, _Row;      // 位姿量个数，控制量个数，预测域，控制域，松弛因子权重
int q1, q2, q3;                               // 分别为x，y，th的权重
int r1, r2;                                           // 分别为v和w的权重
double delta_v, delta_w;          // 分别为v和w的控制增量约束
double vmin, vmax, wmin, wmax;  // 分别为v和w的控制量最小值最大值
double overall_length;
double mpc_hz;
double tolerance; 

// 其他参数
nav_msgs::Path Trajectory;                  // 目前走过的轨迹
custom_messages::vehicle_status Bim_con;       // mpc计算的控制量
custom_messages::vehicle_status Aim_con;       // 参考控制量
custom_messages::vehicle_status ref_pos;               // 参考轨迹位姿
custom_messages::vehicle_status now_pos;           // 当前实际位姿
std::vector<quadprogpp::pathpoint> refTraj;       // 参考轨迹

// 参考轨迹跟踪点索引
int idx = 0;

// 构建mpc算法类
quadprogpp::mpc* MPCnode = new quadprogpp::mpc();

void LocationCallback(const nav_msgs::Odometry::ConstPtr& self_loc_msg){
    now_pos.xPos = self_loc_msg->pose.pose.position.x;
    now_pos.yPos = self_loc_msg->pose.pose.position.y;
    tf::Quaternion RQ2;
    double roll,pitch,yaw;
    tf::quaternionMsgToTF(self_loc_msg->pose.pose.orientation,RQ2);  
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  
    now_pos.yaw = yaw;
}

// 参考轨迹回调函数
void refTrajCallback(const custom_messages::planning_info::ConstPtr& trajectory_msg){
    refTraj.clear();
    int pointnum = trajectory_msg->states.size();
    for(int i = 0; i < pointnum; i++)
    {
        quadprogpp::pathpoint temppoint;
        temppoint.x_r = trajectory_msg->states[i].xPos;
        temppoint.y_r = trajectory_msg->states[i].yPos;
        temppoint.fai_r = trajectory_msg->states[i].yaw;
        temppoint.vx_r = trajectory_msg->states[i].speed;
        temppoint.vw_r = trajectory_msg->states[i].angular_speed;
        refTraj.emplace_back(temppoint);
    }
    idx = 0;
}

void FrontCmdCallback(const geometry_msgs::Twist::ConstPtr& front_cmd_msg){
    front_cmd.linear.x = front_cmd_msg->linear.x;
    front_cmd.angular.z = front_cmd_msg->angular.z;
}

bool ifAchieved(custom_messages::vehicle_status now_pos, custom_messages::vehicle_status ref_pos)
{
    double dis = sqrt(pow(now_pos.xPos - ref_pos.xPos, 2) + pow(now_pos.yPos - ref_pos.yPos, 2));
    // std::cout << "[control] : sqrt : " << dis << ", tol : " << 0.5*overall_length << std::endl;
    if(dis <= tolerance) return true;
    return false;
}

void UpdateRefPoint(custom_messages::vehicle_status &instate, custom_messages::vehicle_status &incmd, const std::vector<quadprogpp::pathpoint> &refTrajectory){
    instate.xPos = refTraj[idx].x_r;
    instate.yPos = refTraj[idx].y_r;
    instate.yaw = refTraj[idx].fai_r;
    incmd.speed = refTraj[idx].vx_r;
    incmd.angular_speed = refTraj[idx].vw_r;
    // instate.xPos = refTraj.back().x_r;
    // instate.yPos = refTraj.back().y_r;
    // instate.yaw = refTraj.back().fai_r;
    // incmd.speed = refTraj.back().vx_r;
    // incmd.angular_speed = refTraj.back().vw_r;
}

int main(int argc, char  **argv)
{
    sleep(3);
    ros::init(argc,argv,"mpc_node");
    ros::NodeHandle nh;

    // 走过的轨迹可视化
    ros::Publisher Vis_traj = nh.advertise<nav_msgs::Path>("VisTraj", 100);
    // 发布控制量
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 20);

    ros::Subscriber mypoint = nh.subscribe("self_location", 2, LocationCallback);
    ros::Subscriber mypathpose = nh.subscribe("trajectory_dwa", 10, refTrajCallback);

    nh.param<int>("Nx", _Nx, 3);
    nh.param<int>("Nu", _Nu, 2);
    nh.param<int>("Np", _Np, 60);
    nh.param<int>("Nc", _Nc, 30);
    nh.param<int>("Row", _Row, 10);
    nh.param<int>("q1", q1, 10);
    nh.param<int>("q2", q2, 10);
    nh.param<int>("q3", q3, 1);
    nh.param<int>("r1", r1, 0);
    nh.param<int>("r2", r2, 0);
    nh.param<double>("mpc_node/delta_v", delta_v, 0.05);
    nh.param<double>("mpc_node/delta_w", delta_w, 0.05);
    nh.param<double>("vmin", vmin, -0.6);
    nh.param<double>("vmax", vmax, 1.2);
    nh.param<double>("wmin", wmin, -1.57);
    nh.param<double>("wmax", wmax, 1.57);
    nh.param<double>("overall_length", overall_length, 0.6);
    nh.param<double>("mpc_node/mpc_hz", mpc_hz, 50.0);
    nh.param<double>("tolerance", tolerance, 0.1);

    MPCnode->init(_Nx, _Nu, _Np, _Nc, _Row, delta_v,  delta_w, q1, q2, q3);
    front_cmd.linear.x = 0;
    front_cmd.angular.z = 0;

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    Trajectory.header.stamp = current_time;
    Trajectory.header.frame_id = "map";

    geometry_msgs::PoseStamped TrajectoryPoint;
    TrajectoryPoint.header.frame_id="map";
    
    ros::Rate loop_rate(mpc_hz);
    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        TrajectoryPoint.pose.position.x=now_pos.xPos;
        TrajectoryPoint.pose.position.y=now_pos.yPos;
        Trajectory.poses.push_back(TrajectoryPoint);
        Vis_traj.publish(Trajectory);

        if(refTraj.empty())
        {
            Bim_con.speed = 0;
            Bim_con.angular_speed = 0; 
            ROS_WARN("[control] : NO reference trajectory! ");
            geometry_msgs::Twist command;
            command.linear.x=Bim_con.speed;
            command.angular.z=Bim_con.angular_speed;
            cmd_pub.publish(command);
            continue;
        }


        // for(int i = 0; i < refTraj.size();++i){
        //     std::cout<<"refTraj x : "<< refTraj[i].x_r<<", y : "<<refTraj[i].y_r<<std::endl;
        // }
        // std::cout<< "idx : " << idx << ", refTraj size : " << refTraj.size() << std::endl;
        UpdateRefPoint(ref_pos, Aim_con, refTraj);
        while (ifAchieved(now_pos, ref_pos))
        {
            // std::cout << "idx : " << idx << ", size : " << refTraj.size() << std::endl;
            if(++idx < refTraj.size()){
                UpdateRefPoint(ref_pos, Aim_con, refTraj);
            }
            else{
                break;
            }
        }
        if(idx >= refTraj.size()){
            geometry_msgs::Twist command;
            command.linear.x=0;
            command.angular.z=0;
            cmd_pub.publish(command);

            // cmd_pub.publish(front_cmd);
            // std::cout << "ref: x : "<< ref_pos.xPos << ", y : " << ref_pos.yPos << ", yaw : " << ref_pos.yaw << ", v : " << Aim_con.speed << ", w : " << Aim_con.angular_speed <<std::endl;
            // std::cout << "cur: x : " << now_pos.xPos << ", y : " << now_pos.yPos << ", yaw : " << now_pos.yaw << std::endl;
            ROS_WARN("[control] : Arrive!");
            continue;
        }

        ros::Time time1 = ros::Time::now();
        Bim_con=MPCnode->mpc_output(T, Aim_con, Bim_con, ref_pos, now_pos, vmin, vmax, wmin, wmax);
        ros::Time time2 = ros::Time::now();
        // ROS_INFO("[control] : MPC Time consume is : %f",(time2 - time1).toSec() );
        // std::cout << "ref: x : "<< ref_pos.xPos << ", y : " << ref_pos.yPos << ", yaw : " << ref_pos.yaw << ", v : " << Aim_con.speed << ", w : " << Aim_con.angular_speed <<std::endl;
        // std::cout << "current= x : " << now_pos.xPos << ", y : " << now_pos.yPos << ", yaw : " << now_pos.yaw << std::endl;
        // ROS_WARN("[control] : MPC output : v = %f , w = %f", Bim_con.speed, Bim_con.angular_speed);
        geometry_msgs::Twist command;
        command.linear.x=Bim_con.speed;
        command.angular.z=Bim_con.angular_speed;
        cmd_pub.publish(command);     
    }
    return 0;
}