#include "dwa.h"

double max_speed;
double min_speed;
double max_angular_speed;
double min_angular_speed;

double max_accel;
double max_angular_speed_rate;

double v_resolution;     // 速度采样分辨率
double yaw_rate_resolution;
double dt;                //运动学模型预测时间
double predict_time;
double goal_cost_gain;
double speed_cost_gain;
double obstacle_cost_gain;
double distance_cost_gain;
double journey_cost_gain;
double overall_length;
int ID;

ros::Publisher trajectory_vis_pub, DWA_path_point_pub;
geometry_msgs::PoseArray trajectory_vis;

AstarPathFinder* dwamap = new AstarPathFinder();

void visDWAPath(vector<Eigen::Vector2d> nodes){
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "DWA/path";
    
    node_vis.type = visualization_msgs::Marker::CUBE_LIST; // type表示物体类型，CUBE_LIST表示立方体列表是一系列立方体，除了位姿所有的属性都一样。使用这个物体类型替代 visualization_msgs/MarkerArray允许rviz批处理显示，这让他们的显示更快。附加说明是它们所有都必须有相同的颜色/尺寸。
    node_vis.action = visualization_msgs::Marker::ADD; // 0=add/modify an object, 1=(deprecated), 2=deletes an object, 3=deletes all objects
    node_vis.id = 0; // 分配给marker的唯一的id

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    // 调整颜色
    node_vis.color.a = 0.7;
    node_vis.color.r = 0.5;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.5;

    node_vis.scale.x = 0.1; // marker的尺寸，在位姿/方向之前应用，[1,1,1]意思是尺寸是1m*1m*1m
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 0.1;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Eigen::Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);

        node_vis.points.push_back(pt);
    }

    DWA_path_point_pub.publish(node_vis);
}

DWA::DWA(ros::Publisher pub){
    trajectory_pub = pub;
}

// void DWA::PathCallback(const custom_messages::vehicle_status::ConstPtr& path_msg){
//     myend.x = path_msg->xPos;
//     myend.y = path_msg->yPos;
//     myend.yaw = path_msg->yaw;
// }
void DWA::PathCallback(const nav_msgs::Path::ConstPtr& path_msg){
    ends = *path_msg;
    iteration = 0;
    first = true;
}
void DWA::FLPathCallback(const custom_messages::vehicle_status::ConstPtr& path_msg){
    ends.poses.clear();
    geometry_msgs::PoseStamped tmp;
    tmp.position.x = path_msg->xPos;
    tmp.position.y = path_msg->yPos;
    ends.poses.emplace_back(tmp);
}

// void DWA::SelfCallback(const custom_messages::vehicle_status::ConstPtr& self_msg){
//     mystate.x = self_msg->xPos;
//     mystate.y = self_msg->yPos;
//     mystate.yaw = self_msg->yaw;
//     // mystate.speed = self_msg->speed;
//     // mystate.angular_speed = self_msg->angular_speed;
// }
void DWA::SelfCallback(const nav_msgs::Odometry::ConstPtr& self_msg){
    mystate.x = self_msg->pose.pose.position.x;
    mystate.y = self_msg->pose.pose.position.y;
    tf::Quaternion RQ2;
    double roll,pitch,yaw;
    //输入四元数，转化成欧拉角数在终端输出
    tf::quaternionMsgToTF(self_msg->pose.pose.orientation,RQ2);  
    // tf::Vector3 m_vector3; 方法2
    // m_vector3=RQ2.getAxis();
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  
    mystate.yaw = yaw;
}


void DWA::CmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg){
    mystate.speed = cmd_msg->linear.x;
    mystate.angular_speed = cmd_msg->angular.z;
}

inline bool CheckDWAArrive(CarState now, CarState end){
    if(sqrt(pow(now.x - end.x, 2) + pow(now.y - end.y, 2)) <= 2.5*overall_length) return true;
    else return false;
}

// 主算法
void DWA::FLplanning()
{
    if(ends.poses.empty()) return;
    CarState currentState(mystate.x, mystate.y, mystate.yaw, mystate.speed, mystate.angular_speed);
    // cout<<"mystate.x"<<mystate.x<<"mystate.y"<<mystate.y<<"mystate.speed"<<mystate.speed<<"mystate.aspeed"<<mystate.angular_speed<<endl;
    // cout<<"currentState.x : "<<currentState.x<<"currentState.y : "<<currentState.y<<endl;
    // cout<<"myend.x : "<<myend.x<<"myend.y : "<<myend.y<<endl;
    // cout<<"sqrt : "<<sqrt(pow(currentState.x - myend.x, 2) + pow(currentState.y - myend.y, 2))<< ", tol : " << 2.5*overall_length <<endl;
    myend.x = ends.poses[iteration].pose.position.x;
    myend.y = ends.poses[iteration].pose.position.y;
    if(CheckDWAArrive(currentState, myend)){
        ROS_WARN("[local planning] : too close to plan");
        return;
    }

    vector<Eigen::Vector2d> DWA_path_point_vis;
    Eigen::Vector2d pt;
    pt<< myend.x, myend.y;
    DWA_path_point_vis.emplace_back(pt);

    vector<CarState> currentTrajectory;

    vector<double> speed(2);     //v[0]为速度, v[1]角速度
    speed = dwa_control(currentState);
    // cout << "final speed:(" << speed[0] << ", " << speed[1] << ")" << endl;
    currentTrajectory.clear();
    predict_trajectory(currentState, speed[0], speed[1], currentTrajectory);
    trajectory.push_back(currentTrajectory);

    trajectory_vis.poses.clear();
    trajectory_vis.header.frame_id = "map";
    trajectory_vis.header.stamp = ros::Time::now();

    custom_messages::planning_info trajectory2control;
    custom_messages::vehicle_status tmp;
    geometry_msgs::Pose tmppose;
    for(int i = 0; i < currentTrajectory.size(); ++i){
        tmp.xPos = currentTrajectory[i].x;
        tmp.yPos = currentTrajectory[i].y;
        tmp.yaw = currentTrajectory[i].yaw;
        tmp.speed = currentTrajectory[i].speed;
        tmp.angular_speed = currentTrajectory[i].angular_speed;
        trajectory2control.states.push_back(tmp);

        tmppose.position.x = currentTrajectory[i].x;
        tmppose.position.y = currentTrajectory[i].y;
        double tmpyaw = currentTrajectory[i].yaw;
        //输入欧拉角，转化成四元数在终端输出
        geometry_msgs::Quaternion q;
        q=tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpyaw);
        tmppose.orientation.w = q.w;
        tmppose.orientation.x = q.x;
        tmppose.orientation.y = q.y;
        tmppose.orientation.z = q.z;
        trajectory_vis.poses.emplace_back(tmppose);
    }
    trajectory_vis_pub.publish(trajectory_vis);
    trajectory_pub.publish(trajectory2control);
    visDWAPath(DWA_path_point_vis);
}
void DWA::planning()
{
    if(ends.poses.empty()) return;
    CarState currentState(mystate.x, mystate.y, mystate.yaw, mystate.speed, mystate.angular_speed);
    // cout<<"mystate.x"<<mystate.x<<"mystate.y"<<mystate.y<<"mystate.speed"<<mystate.speed<<"mystate.aspeed"<<mystate.angular_speed<<endl;
    if(!first){
        // cout<<"currentState.x : "<<currentState.x<<"currentState.y : "<<currentState.y<<endl;
        // cout<<"myend.x : "<<myend.x<<"myend.y : "<<myend.y<<endl;
        // cout<<"sqrt : "<<sqrt(pow(currentState.x - myend.x, 2) + pow(currentState.y - myend.y, 2))<< ", tol : " << 2.5*overall_length <<endl;
        while(CheckDWAArrive(currentState, myend)){
            if(++iteration <= ends.poses.size()){
                myend.x = ends.poses[iteration].pose.position.x;
                myend.y = ends.poses[iteration].pose.position.y;
                cout<< "[local_planning] : Going to next end : " <<myend.x<<" , "<<myend.y<<endl;
            }
            else{
                ROS_WARN("[local_planning] : Waiting for new routing path");
                return;
            }
        }
    }
    else{
        first = false;
        myend.x = ends.poses[iteration].pose.position.x;
        myend.y = ends.poses[iteration].pose.position.y;
        while(CheckDWAArrive(currentState, myend)){
            if(++iteration <= ends.poses.size()){
                myend.x = ends.poses[iteration].pose.position.x;
                myend.y = ends.poses[iteration].pose.position.y;
                cout<< "[local_planning] : Going to next end : " <<myend.x<<" , "<<myend.y<<endl;
            }
            else{
                ROS_WARN("[local_planning] : Waiting for new routing path");
                return;
            }
        }
    }

    vector<Eigen::Vector2d> DWA_path_point_vis;
    Eigen::Vector2d pt;
    pt<< myend.x, myend.y;
    DWA_path_point_vis.emplace_back(pt);

    vector<CarState> currentTrajectory;

    vector<double> speed(2);     //v[0]为速度, v[1]角速度
    speed = dwa_control(currentState);
    // cout << "final speed:(" << speed[0] << ", " << speed[1] << ")" << endl;
    currentTrajectory.clear();
    predict_trajectory(currentState, speed[0], speed[1], currentTrajectory);
    trajectory.push_back(currentTrajectory);

    trajectory_vis.poses.clear();
    trajectory_vis.header.frame_id = "map";
    trajectory_vis.header.stamp = ros::Time::now();

    custom_messages::planning_info trajectory2control;
    custom_messages::vehicle_status tmp;
    geometry_msgs::Pose tmppose;
    for(int i = 0; i < currentTrajectory.size(); ++i){
        tmp.xPos = currentTrajectory[i].x;
        tmp.yPos = currentTrajectory[i].y;
        tmp.yaw = currentTrajectory[i].yaw;
        tmp.speed = currentTrajectory[i].speed;
        tmp.angular_speed = currentTrajectory[i].angular_speed;
        trajectory2control.states.push_back(tmp);

        tmppose.position.x = currentTrajectory[i].x;
        tmppose.position.y = currentTrajectory[i].y;
        double tmpyaw = currentTrajectory[i].yaw;
        //输入欧拉角，转化成四元数在终端输出
        geometry_msgs::Quaternion q;
        q=tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpyaw);
        tmppose.orientation.w = q.w;
        tmppose.orientation.x = q.x;
        tmppose.orientation.y = q.y;
        tmppose.orientation.z = q.z;
        trajectory_vis.poses.emplace_back(tmppose);
    }
    trajectory_vis_pub.publish(trajectory_vis);
    trajectory_pub.publish(trajectory2control);
    visDWAPath(DWA_path_point_vis);
}

//动态窗口法
vector<double> DWA::dwa_control(const CarState &carstate)
{
    vector<double> dw(4);     //dw[0]为最小速度，dw[1]为最大速度，dw[2]为最小角速度，dw[3]为最大角速度
    //计算动态窗口
    dw = calc_dw(carstate);
    // for(int i = 0; i < dw.size(); ++i){
    //     cout<<"dw[i] : "<<dw[i]<<endl;
    // }
    //计算最佳（v, w）
    vector<double> v_w(2);
    v_w = calc_best_speed(carstate, dw);
    return v_w;
}
// 计算动态窗口
vector<double> DWA::calc_dw(const CarState &carstate)
{
    // 机器人速度属性限制的动态窗口
    vector<double> dw_robot_state{min_speed, max_speed, min_angular_speed, max_angular_speed};
    // 机器人模型限制的动态窗口
    vector<double> dw_robot_model(4);
    dw_robot_model[0] = carstate.speed - max_accel;
    dw_robot_model[1] = carstate.speed + max_accel;
    dw_robot_model[2] = carstate.angular_speed - max_angular_speed_rate;
    dw_robot_model[3] = carstate.angular_speed + max_angular_speed_rate;
    // cout<<"dw_robot_model  "<< dw_robot_model[0] << ", "<< dw_robot_model[1] << ", "<< dw_robot_model[2] << ", "<< dw_robot_model[3] <<endl;
    vector<double> dw{max(dw_robot_state[0], dw_robot_model[0]),
                     min(dw_robot_state[1], dw_robot_model[1]),
                     max(dw_robot_state[2], dw_robot_model[2]),
                     min(dw_robot_state[3], dw_robot_model[3])};
    return dw;
}
//在dw中计算最佳速度和角速度
vector<double> DWA::calc_best_speed(const CarState &carstate, const vector<double> &dw)
{
    vector<double> best_speed{0, 0};
    vector<CarState> trajectoryTmp;
    double min_cost = 1<<20;
    double final_cost;
    double goal_cost;
    double speed_cost = 0;
    double obstacle_cost = 0;
    double distance_cost = 0;
    double journey_cost = 0;
    for(double i = dw[0]; i < dw[1]; i += v_resolution)
    {
        for (double j = dw[2]; j < dw[3]; j += yaw_rate_resolution)
        {
            //预测轨迹
            trajectoryTmp.clear();
            predict_trajectory(carstate, i, j, trajectoryTmp);
            //计算代价
            goal_cost = goal_cost_gain * calc_goal_cost(trajectoryTmp);
            speed_cost = speed_cost_gain * (max_speed - trajectoryTmp.back().speed);
            obstacle_cost = obstacle_cost_gain * calc_obstacle_cost(trajectoryTmp, carstate, dwamap);
            distance_cost = distance_cost_gain * sqrt(pow(myend.x - trajectoryTmp.back().x, 2) + pow(myend.y - trajectoryTmp.back().y, 2));
            journey_cost = journey_cost_gain * calc_journey_cost(trajectoryTmp);
            final_cost = goal_cost + speed_cost + obstacle_cost + distance_cost + journey_cost;

            if(final_cost < min_cost)
            {
                min_cost = final_cost;
                best_speed[0] = i;
                best_speed[1] = j;
                // cout << "tmp_best_speed : " <<best_speed[0]<<", "<<best_speed[1]<<endl;
            }
            if(best_speed[0] < 0.001 && carstate.speed < 0.001)
                best_speed[1] = -max_angular_speed_rate;
        }
    }
    return best_speed;
}
// 在一段时间内预测轨迹
void DWA::predict_trajectory(const CarState &carstate, const double &speed, const double &angular_speed, vector<CarState> &trajectory)
{
    double time = 0;
    CarState nextState = carstate;
    nextState.speed = speed;
    nextState.angular_speed = angular_speed;
    while(time < predict_time)
    {
        nextState = motion_model(nextState, speed, angular_speed);
        // cout << "nextState:(" << nextState.x << ", " << nextState.y << ", " << nextState.yaw * 180 / PI << ")" << nextState.speed << "  " << nextState.angular_speed << endl;
        trajectory.push_back(nextState);
        time += dt;
    }
}
//根据动力学模型计算下一时刻状态
CarState DWA::motion_model(const CarState &carstate, const double &speed, const double &angular_speed)
{
    CarState nextState;
    nextState.x = carstate.x + speed * dt * cos(carstate.yaw);
    nextState.y = carstate.y + speed * dt * sin(carstate.yaw);
    nextState.yaw = carstate.yaw + angular_speed * dt;
    // std::cout << "nextStateYaw : " <<nextState.yaw << std::endl;
    nextState.speed = carstate.speed;
    nextState.angular_speed = carstate.angular_speed;
    return nextState;
}
// 计算方位角代价
double DWA::calc_goal_cost(const vector<CarState> &trajectory)
{
    double error_yaw = atan2(myend.y - trajectory.back().y, myend.x - trajectory.back().x);
    double goal_cost = error_yaw - trajectory.back().yaw;
    goal_cost = atan2(sin(goal_cost), cos(goal_cost));
    if(goal_cost >= 0)
        return goal_cost;
    else
        return -goal_cost;
}
// 计算障碍代价
double DWA::calc_obstacle_cost(const vector<CarState> &trajectory, const CarState &carstate, const AstarPathFinder* tmpmap)
{
    double distance;
    for (int i = 0; i < mymapmodel.mapinfo.size(); i ++) {
        for (int j = 0; j < trajectory.size(); j ++) {
            distance = sqrt(pow(mymapmodel.mapinfo[i].x - trajectory[j].x, 2) + pow(mymapmodel.mapinfo[i].y - trajectory[j].y, 2));
            if(distance <= mymapmodel.mapinfo[i].z + overall_length/2)
                return 1<<19;
        }
    }

    double window = predict_time * carstate.speed + 0.5 * max_accel * pow(predict_time, 2);
    vector<Eigen::Vector2d> dwamap_obs;

    for(int i = carstate.x - window/2; i < carstate.x + window/2; i += dwamap->resolution){
        for(int j = carstate.y - window/2; i < carstate.y + window/2; i += dwamap->resolution){
            Eigen::Vector2d tmp;
            tmp << i, j;
            Eigen::Vector2i tmp2 = dwamap->coord2gridIndex(tmp);
            if(dwamap->isOccupied(tmp2)){
                for (int k = 0; k < trajectory.size(); k++) {
                    distance = sqrt(pow(mymapmodel.mapinfo[k].x - trajectory[k].x, 2) + pow(mymapmodel.mapinfo[k].y - trajectory[k].y, 2));
                    if(distance <= mymapmodel.mapinfo[i].z + overall_length/2)
                        return 1<<19;
                }
            }
        }
    }
    return 0;
}

double DWA::calc_journey_cost(const vector<CarState> &trajectory){
    double sum = 0;
    for(int i = 1; i < trajectory.size(); ++i){
        sum += sqrt(pow(trajectory[i].x - trajectory[i - 1].x, 2) + pow(trajectory[i].y - trajectory[i - 1].y, 2));
    }
    return sum;
}

// 加载感知
void DWA::MapModelCallback(const custom_messages::mapmodel::ConstPtr& mapmodel_msg){
    if(mapmodel_msg->mapinfo.empty()){
        // cout << "[local planning] : all clear" << endl;
        return;
    }
    mymapmodel = *mapmodel_msg;
}

void AstarPathFinder::SlamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& slam_map_msgs)
{
    if(!_slam_map.data.empty() || slam_map_msgs == nullptr || slam_map_msgs->data.empty()) return;

    _slam_map = *slam_map_msgs;

    GLX_SIZE = _slam_map.info.width;
    GLY_SIZE = _slam_map.info.height;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE; 
    resolution = _slam_map.info.resolution;
    inv_resolution = 1 / resolution;
    gl_xl = _slam_map.info.origin.position.x;
    gl_yl = _slam_map.info.origin.position.y;
    gl_xu = gl_xl + GLX_SIZE * resolution;
    gl_yu = gl_yl + GLY_SIZE * resolution;
    
	// ofstream out("/home/csxr/out1.txt", ios::app);
    // for (int j = 0; j <_slam_map.data.size(); ++j) {
    //     out<<(int)_slam_map.data[j]<<" ";
    // }
    dwamap->inflation();
	// ofstream out2("/home/csxr/out2.txt", ios::app);
    // for (int j = 0; j <_slam_map.data.size(); ++j) {
    //     out2<<(int)_slam_map.data[j]<<" ";
    // }
    initGridMap();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "DWA");
    ros::NodeHandle n;

    n.param<int>("ID", ID, 0);
    n.param<double>("vmax", max_speed, 1.0);
    n.param<double>("vmin", min_speed, -0.5);
    n.param<double>("wmax", max_angular_speed, 40 * PI / 180.0);
    n.param<double>("wmin", min_angular_speed, -40 * PI / 180.0);
    n.param<double>("dwa/dv", max_accel, 0.2);
    n.param<double>("dwa/dw", max_angular_speed_rate, 20 * PI / 180);
    n.param<double>("v_resolution", v_resolution, 0.01);
    n.param<double>("yaw_rate_resolution", yaw_rate_resolution, 0.1 * PI / 180);
    n.param<double>("dt", dt, 0.1);
    n.param<double>("predict_time", predict_time, 2.0);
    n.param<double>("goal_cost_gain", goal_cost_gain, 0.2);
    n.param<double>("speed_cost_gain", speed_cost_gain, 1.0);
    n.param<double>("obstacle_cost_gain", obstacle_cost_gain, 1.0);
    n.param<double>("distance_cost_gain", distance_cost_gain, 1.0);
    n.param<double>("journey_cost_gain", journey_cost_gain, 0.5);
    n.param<double>("overall_length", overall_length, 0.7);

    ros::Subscriber dwa_map_sub = n.subscribe("slam_ret", 1,  &AstarPathFinder::SlamMapCallback, dwamap);

    ros::Publisher trajectory_pub = n.advertise<custom_messages::planning_info>("trajectory_dwa", 1);
    trajectory_vis_pub = n.advertise<geometry_msgs::PoseArray>("trajectory_dwa_vis", 1, true);
    DWA_path_point_pub = n.advertise<visualization_msgs::Marker>("dwa_path_point_vis", 1, true);

    DWA* mydwa = new DWA(trajectory_pub);

    ros::Subscriber mapmodel_sub = n.subscribe("obstacle_info", 1, &DWA::MapModelCallback, mydwa);
    ros::Subscriber self_sub = n.subscribe("self_location", 1, &DWA::SelfCallback, mydwa);
    ros::Subscriber cmd_sub = n.subscribe("/cmd_vel", 1, &DWA::CmdCallback, mydwa);
    ros::Subscriber path_sub;
    ros::Subscriber FLpath_sub;

    if(!ID){
        path_sub = n.subscribe("routing_path", 1, &DWA::PathCallback, mydwa);
    }
    else{
        FLpath_sub = n.subscribe("/No_0/myodom", 1, &DWA::FLPathCallback, mydwa);
    }
        

    ros::Rate rate(1);
    if(!ID){
        while(ros::ok()){
            ros::spinOnce();
            mydwa->planning();
            rate.sleep();
        }
    }
    else{
        while(ros::ok()){
            ros::spinOnce();
            mydwa->FLplanning();
            rate.sleep();
        }
    }

    return 0;
}