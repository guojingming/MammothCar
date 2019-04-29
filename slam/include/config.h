#pragma once
#include<iostream>
#include<vector>
#include<pcl\point_types.h>
#include<pcl\point_cloud.h>
#include<pcl\filters\conditional_removal.h>
#include<pcl\filters\conditional_removal.h>
#include<pcl\filters\voxel_grid.h>
#include<pcl\kdtree\kdtree_flann.h>
#include<pcl\io\pcd_io.h>
#include<opencv2\opencv.hpp>

#include<io.h>
#include<direct.h>

struct registrationMessage
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_flat;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_less_flat;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_sharp;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_less_sharp;
	pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_2;
};
struct odometryMessage
{
	float transform[6];//½ÃÕý»û±ä
	float transformSum[6];//laser_odom_to_init
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf_last;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_last;
	pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_3;
};
struct mappingMessage
{
	float transformAftMapped[6];//aft_mapped_to_init
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surround;
	pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_registered;
};
