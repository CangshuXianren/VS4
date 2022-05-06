#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

ros::Publisher vir_odom_pub;

void CmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msgs){

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "A*_node");
    ros::NodeHandle nh;

    ros::Subscriber mypoint = nh.subscribe("/cmd_vel", 2, CmdCallback);

    vir_odom_pub = nh.advertise<nav_msgs::Odometry>("grid_path_vis", 1);

    nh.param("overall_length", overall_length, 1.0);

    //    _jps_path_finder    = new JPSPathFinder();
    //    _jps_path_finder    -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status)
    {
        ros::spinOnce();
        vector<Vector2d> grid_obs;
        for(int i = 0; i <_astar_path_finder->_slam_map.data.size(); ++i){
            if(_astar_path_finder->_slam_map.data[i] == 100){
                int tx = i % _astar_path_finder->GLX_SIZE;
                int ty = i / _astar_path_finder->GLX_SIZE;
                Vector2d tmp;
                tmp << _astar_path_finder->gl_xl + tx * _astar_path_finder->resolution, _astar_path_finder->gl_yl + ty * _astar_path_finder->resolution;
                grid_obs.emplace_back(tmp);
            }
        }
        visGridObs(grid_obs);
        status = ros::ok();
        rate.sleep();
    }
    // ros::spin();

    return 0;
}