//#pragma once
#include"config.h"

class scanRegistration
{
public:
	scanRegistration() :laserCloudIn(new pcl::PointCloud<pcl::PointXYZRGB>()), cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
		cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()), surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
		surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()), laserCloud(new pcl::PointCloud<pcl::PointXYZI>())
	{
	}
	int getRingForAngle(const float& angle);
	int CAfilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered);
	int divideBeam(pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudIn);
	int laserHandler(pcl::PointCloud<pcl::PointXYZRGB>::Ptr velodyne_points);
	int extractFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud);
	registrationMessage registrationProcess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr velodyne_points);
private:
	float scanPeriod = 0.1;
    static const int N_SCANS = 33;   //线数 40线：40 ；32线：33

	float cloudCurvature[200000];
	int cloudSortInd[200000];
	int cloudNeighborPicked[200000];
	int cloudLabel[200000];

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudIn;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp;
	pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat;
	pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
};