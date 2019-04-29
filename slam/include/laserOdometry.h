#pragma once
#include "config.h"

class laserOdometry
{
public:
	laserOdometry() :cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()), cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
		surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
		laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),
		laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
		coeffSel(new pcl::PointCloud<pcl::PointXYZI>()),kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
		kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>())
	{
		systemInited = false;
		memset(transform, 0, sizeof(transform));
		memset(transformSum, 0, sizeof(transformSum));
	}
	void save2txt(float transform[6], int j,std::string filename);
	void save2pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes1, int j,std::string filename);
	void laserCloudSharpHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_sharp);
	void laserCloudLessSharpHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_less_sharp);
	void laserCloudFlatHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_flat);
	void laserCloudLessFlatHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_less_flat);
	void laserCloudFullResHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_2);
	void TransformToStart(pcl::PointXYZI *pi, pcl::PointXYZI *po);
	void TransformToEnd(pcl::PointXYZI *pi, pcl::PointXYZI *po);
	void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, float &ox, float &oy, float &oz);
	odometryMessage odometryProcess(registrationMessage regisMessage);
private:
	int laserCloudCornerLastNum, laserCloudSurfLastNum;
	float scanPeriod = 0.1;
	int skipFrameNum = 1;
	bool systemInited;
	
	float transform[6];
	float transformSum[6];

	int pointSelCornerInd[40000];
	int pointSearchCornerInd1[40000];
	float pointSearchCornerInd2[40000];

	int pointSelSurfInd[40000];
	float pointSearchSurfInd1[40000];
	float pointSearchSurfInd2[40000];
	float pointSearchSurfInd3[40000];

	pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp;
	pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat;
	pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri;
	pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel;

	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;//保留的是上一时刻的less
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;

};