#include "self_gps2xy.h"

//角度制转弧度制
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
ros::Publisher location_pub;
//全局变量
static double EARTH_RADIUS = 6378.137;//地球半径
bool init = false;
custom_messages::vehicle_status init_pose; // 用xPos和yPos分别暂存经度和纬度

double offset_x, offset_y, offset_yaw, offset_pitch, offset_roll;
double dp_x,dp_y,dp_z,dp_w;
nav_msgs::Odometry offset;
ros::Publisher myodom_pub, mycloud_vis_pub;
bool offsetflag = false;


void GpsCallback(const novatel_oem7_msgs::INSPVAX::ConstPtr& gps_msg)
{
  //初始化
  if(!init)
  {
      init_pose.xPos = gps_msg->longitude;
      init_pose.yPos = gps_msg->latitude;
      init_pose.yaw = gps_msg->azimuth;
      init = true;
  }
  else
  {
  //计算相对位置
  double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long;
  radLat1 = rad(init_pose.yPos);
  radLong1 = rad(init_pose.xPos);
  radLat2 = rad(gps_msg->latitude);
  radLong2 = rad(gps_msg->longitude);
  //计算x
  delta_lat = radLat2 - radLat1;
  delta_long = 0;
  double post_x = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
  post_x = post_x*EARTH_RADIUS*1000;

  //计算y
  delta_lat = 0;
  delta_long = radLong1  - radLong2;
  double post_y = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
  post_y = post_y*EARTH_RADIUS*1000;

  ROS_INFO("( x : %0.6f , y : %0.6f , yaw : %0.6f)", post_x, post_y, gps_msg->azimuth);

  custom_messages::planning_info self_location;
  self_location.states.push_back(init_pose);
  custom_messages::vehicle_status current;
  current.xPos = post_x;
  current.yPos = post_y;
  current.yaw = gps_msg->azimuth;
  self_location.states.push_back(current);

  location_pub.publish(self_location);
  }
}

void OffsetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& offset_msg){
  if(offsetflag) return;
  offset_x = offset_msg->pose.pose.position.x;
  offset_y = offset_msg->pose.pose.position.y;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(offset_msg->pose.pose.orientation, quat);
  double roll, pitch, yaw;//定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(offset_roll, offset_pitch, offset_yaw);//进行转换
  dp_x = offset_msg->pose.pose.orientation.x;
  dp_y = offset_msg->pose.pose.orientation.y;
  dp_z = offset_msg->pose.pose.orientation.z;
  dp_w = offset_msg->pose.pose.orientation.w;
  offsetflag = true;
}

// 转换到绝对坐标系
inline void coneTrans(const Eigen::Vector2d& pi, Eigen::Vector2d& po, Eigen::Vector3d test)
{
    po[0] = pi[0] * cos(test[2]) - pi[1] * sin(test[2]) + test[0];
    po[1] = pi[0] * sin(test[2]) + pi[1] * cos(test[2]) + test[1];
}

//转换到相对坐标系
inline void coneAntiTrans(const Eigen::Vector2d& pi, Eigen::Vector2d& po, Eigen::Vector3d& test)
{
    po[0] = pi[0] * cos(test[2]) + pi[1] * sin(test[2]) - (test[0] * cos(test[2]) + test[1] * sin(test[2]));
    po[1] = -pi[0] * sin(test[2]) + pi[1] * cos(test[2]) - (-test[0] * sin(test[2]) + test[1] * cos(test[2]));   
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  if(!offsetflag) return;
  offset = *odom_msg;
  // std::cout << "before odom: x = " << offset.pose.pose.position.x << ", y = " << offset.pose.pose.position.y << std::endl;
  offset.header.frame_id = "map";
  Eigen::Vector2d pi, po;
  pi << offset.pose.pose.position.x, offset.pose.pose.position.y;
  Eigen::Vector3d move;
  move << offset_x, offset_y, offset_yaw;
  coneTrans(pi, po, move);
  offset.pose.pose.position.x = po[0];
  offset.pose.pose.position.y = po[1];
  // std::cout << "after odom: x = " << offset.pose.pose.position.x << ", y = " << offset.pose.pose.position.y << std::endl;

  // The 4 wrong version
  // offset.pose.pose.position.x += offset_x;
  // offset.pose.pose.position.y += offset_y;
  // method4 : manipulate rpy, and give quaternion finally
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);
  double tmproll, tmppitch, tmpyaw;
  tf::Matrix3x3(quat).getRPY(tmproll, tmppitch, tmpyaw);
  double nowyaw = offset_yaw + tmpyaw;
  // std::cout<<"nowyaw , offset_yaw , tmpyaw" << nowyaw << " | " << offset_yaw << " | " << tmpyaw << std::endl;
  tf::Quaternion now(0, 0, nowyaw); // roll is around zaxis here actually!!!
  now.normalize();
  offset.pose.pose.orientation.w = now.w();
  offset.pose.pose.orientation.x = now.x();
  offset.pose.pose.orientation.y = now.y();
  offset.pose.pose.orientation.z = now.z();
  // method3 : p` = Rotationmatrix * p + translation
  // Eigen::Vector3d q0()
  // Eigen::Quaterniond dq(offset.pose.pose.orientation.w, offset.pose.pose.orientation.x, offset.pose.pose.orientation.y, offset.pose.pose.orientation.z);
  // Eigen::Quaterniond q1 = dq.toRotationMatrix() * 
  // method1 : rpy of tf
  // tf::Quaternion dq (offset_yaw, offset_pitch, offset_roll);
  // tf::Quaternion q0 (offset.pose.pose.orientation.x, offset.pose.pose.orientation.y, offset.pose.pose.orientation.z, offset.pose.pose.orientation.w);
  // method2 : quaternion directly give
  // tf::Quaternion dq (dp_x,dp_y,dp_z,dp_w);
  // tf::Quaternion q0 (offset.pose.pose.orientation.x, offset.pose.pose.orientation.y, offset.pose.pose.orientation.z, offset.pose.pose.orientation.w);
  // tf::Quaternion q1 = dq*q0;
  // std::cout<<"dq : "<<dq.x()<<", "<<dq.y()<<", "<<dq.z()<<", "<<dq.w()<<", "<<std::endl;
  // std::cout<<"q0 : "<<q0.x()<<", "<<q0.y()<<", "<<q0.z()<<", "<<q0.w()<<", "<<std::endl;
  // std::cout<<"q1 : "<<q1.x()<<", "<<q1.y()<<", "<<q1.z()<<", "<<q1.w()<<", "<<std::endl;
  // q1.normalize();
  // offset.pose.pose.orientation.w = q1.w();
  // offset.pose.pose.orientation.x = q1.x();
  // offset.pose.pose.orientation.y = q1.y();
  // offset.pose.pose.orientation.z = q1.z();
  myodom_pub.publish(offset);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  Eigen::Quaterniond q (offset.pose.pose.orientation.w, offset.pose.pose.orientation.x, offset.pose.pose.orientation.y, offset.pose.pose.orientation.z);
  Eigen::Vector3d t (offset.pose.pose.position.x, offset.pose.pose.position.y, 0.0);
  Eigen::Matrix4d T (Eigen::Matrix4d::Identity());
  // T.block(0,0,3,3) = q.toRotationMatrix();
  T.block(0,3,3,1) = t;

  pcl::transformPointCloud(*cloud, *cloud, T);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cloud, msg_out);
  msg_out.header.frame_id = "map";
  mycloud_vis_pub.publish(msg_out);
}

int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "self_GPS2XY");  

  ros::NodeHandle n;

  offset.header.frame_id = "map";

  mycloud_vis_pub = n.advertise<sensor_msgs::PointCloud2>("mycloud_vis", 10); 
  location_pub = n.advertise<custom_messages::planning_info>("self_location", 10); 
  myodom_pub = n.advertise<nav_msgs::Odometry>("myodom", 1);
  ros::Subscriber sub = n.subscribe("/novatel/oem7/inspvax", 10, GpsCallback);
  ros::Subscriber offset_sub = n.subscribe("initialpose", 2, OffsetCallback);
  ros::Subscriber odom_sub = n.subscribe("odom", 2, OdomCallback);
  ros::Subscriber cloud_sub = n.subscribe("velodyne_points", 2, cloudCallback);

  ros::spin();  

  return 0;  
}  