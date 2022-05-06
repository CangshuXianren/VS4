#include "perceive.h"
using namespace std;

nav_msgs::OccupancyGrid mymapping;

ros::Publisher obstacle_pub, obs_vis_pub;

custom_messages::vehicle_status now_pos;

void visObstacle(vector<Eigen::Vector3d> nodes){
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

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Eigen::Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
            node_vis.scale.x = 0.1; // marker的尺寸，在位姿/方向之前应用，[1,1,1]意思是尺寸是1m*1m*1m
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 0.1;

        node_vis.points.push_back(pt);
    }

    DWA_path_point_pub.publish(node_vis);
}

void SlamCallback(const nav_msgs::OccupancyGrid::ConstPtr& slam_msg){
    if(slam_msg->data.empty()) return;
    mymapping = *slam_msg;
}

void LocationCallback(const custom_messages::vehicle_status::ConstPtr& self_loc_msg){
    now_pos.xPos = self_loc_msg->xPos;
    now_pos.yPos = self_loc_msg->yPos;
    now_pos.yaw = self_loc_msg->yaw;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(msg == nullptr) return;
    now_pos.xPos = msg->pose.pose.position.x;
    now_pos.yPos = msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double Roll, Pitch, Yaw;
    tf::Matrix3x3(quat).getRPY(Roll, Pitch, Yaw);
    now_pos.yaw = Yaw;
}

void PerceiveCallback(const sensor_msgs::PointCloud2::ConstPtr& perceive_msg){
    pcl::PointCloud<PointType>::Ptr laserCloudIn (new pcl::PointCloud<PointType>); // 对x字段的滤波
    pcl::PointCloud<PointType>::Ptr laserCloudIn2 (new pcl::PointCloud<PointType>); // 对y字段的滤波
    pcl::PointCloud<PointType>::Ptr laserCloudIn3 (new pcl::PointCloud<PointType>); // 对z字段的滤波
    pcl::PointCloud<PointType>::Ptr filterCloudLatest (new pcl::PointCloud<PointType>); // 三层过滤之后最终的点云
    pcl::PointCloud<PointType>::Ptr groundCloudLatest (new pcl::PointCloud<PointType>); // 地面点云
    pcl::PointCloud<PointType>::Ptr objectCloudLatest (new pcl::PointCloud<PointType>); // 目标物点云
    pcl::PointCloud<PointType>::Ptr coneCouldLatest (new pcl::PointCloud<PointType>); // 将目标物模型化后的障碍物点云
    
    pcl::fromROSMsg(*perceive_msg, *laserCloudIn);

    // fliter
    pcl::PassThrough<PointType> pass1;
    pcl::PassThrough<PointType> pass2;
    pcl::PassThrough<PointType> pass3;
    pass1.setInputCloud (laserCloudIn);
    pass1.setFilterFieldName ("x");
    pass1.setFilterLimits (0, 13.0);
    pass1.filter(*laserCloudIn2);

    pass2.setInputCloud (laserCloudIn2);
    pass2.setFilterFieldName ("y");
    pass2.setFilterLimits (-4.0, 4.0);
    pass2.filter (*laserCloudIn3);

    pass3.setInputCloud (laserCloudIn3);
    pass3.setFilterFieldName ("z");
    pass3.setFilterLimits (-1.5, 0.5);
    pass3.filter (*filterCloudLatest);

    // Plane Model Segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud (filterCloudLatest);
    seg.segment (*inliers, *coefficients);
	
	// float A, B, C, D;
	// A = coefficients->values[0];
	// B = coefficients->values[1];
	// C = coefficients->values[2];
	// D = coefficients->values[3];
// 	ROS_INFO("Plane value : a = %f b = %f c = %f d = %f", 
// 			 coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    // 提取地面
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (filterCloudLatest);
    extract.setIndices (inliers);
    extract.filter (*groundCloudLatest);
    // 提取除地面外的物体
    extract.setNegative (true);
    extract.filter (*objectCloudLatest);
    
    // ROS_INFO("cluster ...");
    std::vector<pcl::PointIndices> cluster_indices;
    // cluster
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    pcl::EuclideanClusterExtraction<PointType> ec;
    cluster_indices.clear();
    tree->setInputCloud(objectCloudLatest);
    //search strategy tree
    ec.setSearchMethod (tree);
    ec.setInputCloud (objectCloudLatest);
    ec.extract (cluster_indices);
	// std::cout << "cluster size: " << cluster_indices.size() << std::endl;

    //push cone points
    std::vector<PointType> obj;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        PointType p;
        p.z = 0.0; // 点云质心到自身的距离
        p.intensity = 0.0; // 最大半径，储存点云质心到最远点的半径
		int num = 0;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			p.x += objectCloudLatest->points[*pit].x;
			p.y += objectCloudLatest->points[*pit].y;
			num++;
		}
		// 计算点云质心
        p.x = p.x / float(num);
        p.y = p.y / float(num);

		float range = 0.0;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            range = pow(objectCloudLatest->points[*pit].x - p.x, 2) + pow(objectCloudLatest->points[*pit].y - p.y, 2);
            if (range > p.intensity) p.intensity = range;
        }
        if (p.intensity == 0.0)
		{
			// ROS_WARN("no radius");
			continue;
		}
		p.intensity = sqrt(p.intensity);
		p.z = sqrt(p.x*p.x + p.y*p.y);
        coneCouldLatest->points.push_back(p);
        obj.push_back(p);

        // std::cout << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << std::endl;
        // std::cout << it->indices.size() << std::endl;
    }
    // if (cluster_indices.size() != coneCouldLatest->points.size())
    //     ROS_WARN("cluster_indices.size != coneCouldLatest->points.size");
    custom_messages::mapmodel obstacle_msg;
    for(auto &my : obj){
        geometry_msgs::Point tmp;
        tmp.x = now_pos.xPos + my.x;
        tmp.y = now_pos.yPos + my.y;
        tmp.z = my.intensity;
        obstacle_msg.mapinfo.push_back(tmp);
    }
    obstacle_pub.publish(obstacle_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perceive_node");
    ros::NodeHandle nh;

    ros::Subscriber slam_ret_sub = nh.subscribe("slam_ret", 1, SlamCallback);
    ros::Subscriber mypoint = nh.subscribe("self_location", 2, odomCallback);
    ros::Subscriber lidar_pub = nh.subscribe("velodyne_points", 1, PerceiveCallback);
    obstacle_pub = nh.advertise<custom_messages::mapmodel>("obstacle_info", 1);
    ros::spin();

    return 0;
}