/*
'''    
    Purpose: Transform Robosense Front LIDAR Points ("rslidar_F") to Velodyne Frame ("velodyne")
    Subscribed topics: /velodyne_points
    Published topic: /velodyne_points/transformed

    Project: WATonoBus
    Author: Neel Bhatt
'''    
*/

#include "ros/ros.h"
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

#include <ctime>

std::clock_t start;

sensor_msgs::PointCloud2 transformed_cloud_msg;
ros::Publisher pub_transformed_cloud;

Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
// Transform listener
void callback_velodyne_transform(const std_msgs::Float32MultiArray::ConstPtr& velodyne_transform){
    start = std::clock();
    // std::cout << "--Start--" << std::endl;
    int k = 0;
    for (int i=0;i<4;i++){
        for (int j=0; j<4;j++){
            transform_1(i,j) = velodyne_transform->data[k];
            // std::cout<<velodyne_transform->data[i]<<std::endl;
            k++;
        }
    }
    // std::cout<<std::setprecision(16)<<transform_1<<std::endl;
    // std::cout << "--End--" << std::endl;
}

// Transformer
void callback_pointcloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& raw_cloud){
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    
    // Convert from PC2 ROS msg to PCL (XYZI) object
    pcl::fromROSMsg(*raw_cloud, pcl_cloud);

    // Transformation per point
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::transformPointCloud(pcl_cloud, *transformed_cloud, transform_1);
    
    // Convert from PCL (XYZI) object to PC2 ROS msg
    pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
    
    // Publish transformed cloud msg    
    transformed_cloud_msg.header.stamp = raw_cloud->header.stamp;
    transformed_cloud_msg.header.frame_id = "map";
    pub_transformed_cloud.publish(transformed_cloud_msg);
    std::cout << "Took: "<<(std::clock() - start)/(double) CLOCKS_PER_SEC << " ms" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_transform_node");

    ros::NodeHandle nh;

    ros::Subscriber sub_velodyne_transform = nh.subscribe("/transforms/LUTM_T_L", 1, callback_velodyne_transform);
    ros::Subscriber sub_velodyne_points = nh.subscribe("/velodyne_points", 1, callback_pointcloud);
    pub_transformed_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/UTM", 1);
    ros::spin();

  return 0;
}