#pragma once
#include"config.h"

class laserMapping
{
public:
	laserMapping() :laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
		laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()), laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>()),
		laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>()), laserCloudCornerStack2(new pcl::PointCloud<pcl::PointXYZI>()),
		laserCloudSurfStack2(new pcl::PointCloud<pcl::PointXYZI>()), laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
		coeffSel(new pcl::PointCloud<pcl::PointXYZI>()), laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>()),
		laserCloudSurround2(new pcl::PointCloud<pcl::PointXYZI>()), laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>()),
	    laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>()), kdtreeCornerFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
		kdtreeSurfFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>())
	{
		laserCloudCenWidth = 10;
		laserCloudCenHeight = 5;
		laserCloudCenDepth = 10;

		memset(transformSum, 0, sizeof(transformSum));
		memset(transformIncre, 0, sizeof(transformIncre));
		memset(transformBefMapped, 0, sizeof(transformIncre));
		memset(transformTobeMapped, 0, sizeof(transformTobeMapped));
		memset(transformAftMapped, 0, sizeof(transformAftMapped));

		for (int i = 0; i < laserCloudNum; i++) {
			laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
			laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
			laserCloudCornerArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
			laserCloudSurfArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
		}
	}
	void save2txt(float transform[6],int j,std::string filename);
	void save2pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes1, int j,std::string filename);
	void laserCloudCornerLastHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_last);
	void laserCloudSurfLastHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf_last);
	void laserCloudFullResHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_3);
	void laserOdometryHandler(float* laser_odom_to_init);

	void transformAssociateToMap();
	void transformUpdate();
	void pointAssociateToMap(pcl::PointXYZI *pi, pcl::PointXYZI *po);
	void pointAssociateTobeMapped(pcl::PointXYZI *pi, pcl::PointXYZI *po);

	mappingMessage mappingProcess(odometryMessage odoMessage);
private:
	float scanPeriod = 0.1;
	static const int stackFrameNum = 1, mapFrameNum = 5;
	int laserCloudCenWidth, laserCloudCenHeight, laserCloudCenDepth;
	static const int laserCloudWidth = 21, laserCloudHeight = 11, laserCloudDepth = 21;
	static const int laserCloudNum = laserCloudWidth * laserCloudHeight*laserCloudDepth;

	int laserCloudValidInd[125];
	int laserCloudSurroundInd[125];

	float transformSum[6], transformIncre[6], transformTobeMapped[6], transformBefMapped[6], transformAftMapped[6];

	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes;

	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack2;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack2;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri;

	pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround2;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerFromMap;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfFromMap;

	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerArray[laserCloudNum];
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfArray[laserCloudNum];
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerArray2[laserCloudNum];
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfArray2[laserCloudNum];

	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap;
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfFromMap;

};