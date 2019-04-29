#include "laserMapping.h"

void laserMapping::save2txt(float transform[6], int j,std::string filename)
{
	if (_access(filename.data(), 0) != 0)
	{
		_mkdir(filename.data());
	}
	std::ofstream trans(filename + std::to_string(j) + ".txt");
	trans << transform[0] << " " << transform[1] << " " << transform[2] << std::endl;
	trans << transform[3] << " " << transform[4] << " " << transform[5] << std::endl;
	std::cout << "Save2txtMap" << std::endl;
}

void laserMapping::save2pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes1, int j,std::string filename)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr destCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	destCloud->points.resize(laserCloudFullRes1->points.size());
	for (int k = 0; k < laserCloudFullRes1->points.size(); k++)
	{	//ROS转JiZhang时用这个
		//destCloud->points[k].x = -laserCloudFullRes1->points[k].z;
		//destCloud->points[k].y = -laserCloudFullRes1->points[k].x;
		//destCloud->points[k].z = laserCloudFullRes1->points[k].y;

		destCloud->points[k].x = -laserCloudFullRes1->points[k].x;
		destCloud->points[k].y = laserCloudFullRes1->points[k].z;
		destCloud->points[k].z = laserCloudFullRes1->points[k].y;

		int I = laserCloudFullRes1->points[k].intensity;
		destCloud->points[k].g = -(I % 100) + 32;   //40线39；32线32
		destCloud->points[k].r = (int)(I / 100);
		destCloud->points[k].b = 200;
		destCloud->points[k].a = 255;
	}
	destCloud->height = 1;
	destCloud->width = destCloud->points.size();
	if (_access(filename.data(), 0)!=0)
	{
		_mkdir(filename.data());
	}
	pcl::io::savePCDFileBinary(filename + std::to_string(j) + "_after.pcd", *destCloud);
	std::cout << "after mapping :" << j << std::endl;
}

void laserMapping::laserCloudCornerLastHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_last)
{
	laserCloudCornerLast->clear();
	*laserCloudCornerLast = *laser_cloud_corner_last;
}

void laserMapping::laserCloudSurfLastHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf_last)
{
	laserCloudSurfLast->clear();
	*laserCloudSurfLast = *laser_cloud_surf_last;
}

void laserMapping::laserCloudFullResHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_3)
{
	laserCloudFullRes->clear();
	*laserCloudFullRes = *velodyne_cloud_3;
}

void laserMapping::laserOdometryHandler(float * laser_odom_to_init)
{
	for (int i = 0; i < 6; i++)
		transformSum[i] = laser_odom_to_init[i];
}

void laserMapping::transformAssociateToMap()
{
	float x1 = cos(transformSum[1])*(transformBefMapped[3] - transformSum[3]) - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
	float y1 = transformBefMapped[4] - transformSum[4];
	float z1 = sin(transformSum[1])*(transformBefMapped[3] - transformSum[3]) + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

	float x2 = x1;
	float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
	float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

	transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
	transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
	transformIncre[5] = z2;

	float sbcx = sin(transformSum[0]);
	float cbcx = cos(transformSum[0]);
	float sbcy = sin(transformSum[1]);
	float cbcy = cos(transformSum[1]);
	float sbcz = sin(transformSum[2]);
	float cbcz = cos(transformSum[2]);

	float sblx = sin(transformBefMapped[0]);
	float cblx = cos(transformBefMapped[0]);
	float sbly = sin(transformBefMapped[1]);
	float cbly = cos(transformBefMapped[1]);
	float sblz = sin(transformBefMapped[2]);
	float cblz = cos(transformBefMapped[2]);

	float salx = sin(transformAftMapped[0]);
	float calx = cos(transformAftMapped[0]);
	float saly = sin(transformAftMapped[1]);
	float caly = cos(transformAftMapped[1]);
	float salz = sin(transformAftMapped[2]);
	float calz = cos(transformAftMapped[2]);

	float srx = -sbcx * (salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly)
		- cbcx * cbcz*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
			- calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
		- cbcx * sbcz*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
			- calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz);
	transformTobeMapped[0] = -asin(srx);

	float srycrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
		- calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
		- (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
			- calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
		+ cbcx * sbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
	float crycrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
		- calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
		- (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
			- calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
		+ cbcx * cbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
	transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
		crycrx / cos(transformTobeMapped[0]));

	float srzcrx = sbcx * (cblx*cbly*(calz*saly - caly * salx*salz)
		- cblx * sbly*(caly*calz + salx * saly*salz) + calx * salz*sblx)
		- cbcx * cbcz*((caly*calz + salx * saly*salz)*(cbly*sblz - cblz * sblx*sbly)
			+ (calz*saly - caly * salx*salz)*(sbly*sblz + cbly * cblz*sblx)
			- calx * cblx*cblz*salz) + cbcx * sbcz*((caly*calz + salx * saly*salz)*(cbly*cblz
				+ sblx * sbly*sblz) + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz)
				+ calx * cblx*salz*sblz);
	float crzcrx = sbcx * (cblx*sbly*(caly*salz - calz * salx*saly)
		- cblx * cbly*(saly*salz + caly * calz*salx) + calx * calz*sblx)
		+ cbcx * cbcz*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
			+ (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly)
			+ calx * calz*cblx*cblz) - cbcx * sbcz*((saly*salz + caly * calz*salx)*(cblz*sbly
				- cbly * sblx*sblz) + (caly*salz - calz * salx*saly)*(cbly*cblz + sblx * sbly*sblz)
				- calx * calz*cblx*sblz);
	transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
		crzcrx / cos(transformTobeMapped[0]));

	x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
	y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
	z1 = transformIncre[5];

	x2 = x1;
	y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
	z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

	transformTobeMapped[3] = transformAftMapped[3]
		- (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
	transformTobeMapped[4] = transformAftMapped[4] - y2;
	transformTobeMapped[5] = transformAftMapped[5]
		- (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void laserMapping::transformUpdate()
{
	for (int i = 0; i < 6; i++) {
		transformBefMapped[i] = transformSum[i];
		transformAftMapped[i] = transformTobeMapped[i];
	}
}

void laserMapping::pointAssociateToMap(pcl::PointXYZI * pi, pcl::PointXYZI * po)
{
	float x1 = cos(transformTobeMapped[2]) * pi->x
		- sin(transformTobeMapped[2]) * pi->y;
	float y1 = sin(transformTobeMapped[2]) * pi->x
		+ cos(transformTobeMapped[2]) * pi->y;
	float z1 = pi->z;

	float x2 = x1;
	float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
	float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

	po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
		+ transformTobeMapped[3];
	po->y = y2 + transformTobeMapped[4];
	po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
		+ transformTobeMapped[5];
	po->intensity = pi->intensity;
}

void laserMapping::pointAssociateTobeMapped(pcl::PointXYZI * pi, pcl::PointXYZI * po)
{
	float x1 = cos(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
		- sin(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
	float y1 = pi->y - transformTobeMapped[4];
	float z1 = sin(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
		+ cos(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);

	float x2 = x1;
	float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
	float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

	po->x = cos(transformTobeMapped[2]) * x2
		+ sin(transformTobeMapped[2]) * y2;
	po->y = -sin(transformTobeMapped[2]) * x2
		+ cos(transformTobeMapped[2]) * y2;
	po->z = z2;
	po->intensity = pi->intensity;
}

mappingMessage laserMapping::mappingProcess(odometryMessage odoMessage)
{
	laserCloudCornerLastHandler(odoMessage.laser_cloud_corner_last);
	laserCloudSurfLastHandler(odoMessage.laser_cloud_surf_last);
	laserCloudFullResHandler(odoMessage.velodyne_cloud_3);
	laserOdometryHandler(odoMessage.transformSum);
	mappingMessage mapMessage;

	std::vector<int> pointSearchInd;
	std::vector<float> pointSearchSqDis;

	pcl::PointXYZI pointOri, pointSel, pointProj, coeff;

	cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
	cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

	cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

	bool isDegenerate = false;
	cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner;
	downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
	downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterMap;
	downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);

	int frameCount = stackFrameNum - 1;//stackFrameNum初始值为1，frameCount初始值为0
	int mapFrameCount = mapFrameNum - 1;//mapFrameNum初始值为5，mapFrameCount初始值为4

	frameCount++;
	if (frameCount >= stackFrameNum) {//stackFrameNum初始值为1
		transformAssociateToMap();//计算得到Twk(tk+1)

		int laserCloudCornerLast1Num = laserCloudCornerLast->points.size();
		for (int i = 0; i < laserCloudCornerLast1Num; i++) {
			pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
			laserCloudCornerStack2->push_back(pointSel);//laserCloudCornerStack2存的是Qk
		}

		int laserCloudSurfLast1Num = laserCloudSurfLast->points.size();
		for (int i = 0; i < laserCloudSurfLast1Num; i++) {
			pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
			laserCloudSurfStack2->push_back(pointSel);//laserCloudSurfStack2中存的是Qk
		}
	}

	if (frameCount >= stackFrameNum) {
		frameCount = 0;

		pcl::PointXYZI pointOnYAxis;
		pointOnYAxis.x = 0.0;
		pointOnYAxis.y = 10.0;
		pointOnYAxis.z = 0.0;
		pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

		int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
		int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
		int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;

		if (transformTobeMapped[3] + 25.0 < 0) centerCubeI--;
		if (transformTobeMapped[4] + 25.0 < 0) centerCubeJ--;
		if (transformTobeMapped[5] + 25.0 < 0) centerCubeK--;

		while (centerCubeI < 3) {
			for (int j = 0; j < laserCloudHeight; j++) {
				for (int k = 0; k < laserCloudDepth; k++) {
					int i = laserCloudWidth - 1;
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					for (; i >= 1; i--) {
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					}
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeCornerPointer;
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeSurfPointer;
					laserCloudCubeCornerPointer->clear();
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeI++;
			laserCloudCenWidth++;
		}

		while (centerCubeI >= laserCloudWidth - 3) {
			for (int j = 0; j < laserCloudHeight; j++) {
				for (int k = 0; k < laserCloudDepth; k++) {
					int i = 0;
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					for (; i < laserCloudWidth - 1; i++) {
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					}
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeCornerPointer;
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeSurfPointer;
					laserCloudCubeCornerPointer->clear();
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeI--;
			laserCloudCenWidth--;
		}

		while (centerCubeJ < 3) {
			for (int i = 0; i < laserCloudWidth; i++) {
				for (int k = 0; k < laserCloudDepth; k++) {
					int j = laserCloudHeight - 1;
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					for (; j >= 1; j--) {
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
					}
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeCornerPointer;
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeSurfPointer;
					laserCloudCubeCornerPointer->clear();
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeJ++;
			laserCloudCenHeight++;
		}

		while (centerCubeJ >= laserCloudHeight - 3) {
			for (int i = 0; i < laserCloudWidth; i++) {
				for (int k = 0; k < laserCloudDepth; k++) {
					int j = 0;
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					for (; j < laserCloudHeight - 1; j++) {
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
					}
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeCornerPointer;
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeSurfPointer;
					laserCloudCubeCornerPointer->clear();
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeJ--;
			laserCloudCenHeight--;
		}

		while (centerCubeK < 3) {
			for (int i = 0; i < laserCloudWidth; i++) {
				for (int j = 0; j < laserCloudHeight; j++) {
					int k = laserCloudDepth - 1;
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					for (; k >= 1; k--) {
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
					}
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeCornerPointer;
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeSurfPointer;
					laserCloudCubeCornerPointer->clear();
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeK++;
			laserCloudCenDepth++;
		}

		while (centerCubeK >= laserCloudDepth - 3) {
			for (int i = 0; i < laserCloudWidth; i++) {
				for (int j = 0; j < laserCloudHeight; j++) {
					int k = 0;
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					for (; k < laserCloudDepth - 1; k++) {
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
					}
					laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeCornerPointer;
					laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
						laserCloudCubeSurfPointer;
					laserCloudCubeCornerPointer->clear();
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeK--;
			laserCloudCenDepth--;
		}

		int laserCloudValidNum = 0;
		int laserCloudSurroundNum = 0;
		for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
			for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
				for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
					if (i >= 0 && i < laserCloudWidth &&
						j >= 0 && j < laserCloudHeight &&
						k >= 0 && k < laserCloudDepth) {

						float centerX = 50.0 * (i - laserCloudCenWidth);
						float centerY = 50.0 * (j - laserCloudCenHeight);
						float centerZ = 50.0 * (k - laserCloudCenDepth);

						bool isInLaserFOV = false;
						for (int ii = -1; ii <= 1; ii += 2) {
							for (int jj = -1; jj <= 1; jj += 2) {
								for (int kk = -1; kk <= 1; kk += 2) {
									float cornerX = centerX + 25.0 * ii;
									float cornerY = centerY + 25.0 * jj;
									float cornerZ = centerZ + 25.0 * kk;

									float squaredSide1 = (transformTobeMapped[3] - cornerX)
										* (transformTobeMapped[3] - cornerX)
										+ (transformTobeMapped[4] - cornerY)
										* (transformTobeMapped[4] - cornerY)
										+ (transformTobeMapped[5] - cornerZ)
										* (transformTobeMapped[5] - cornerZ);

									float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
										+ (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
										+ (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

									float check1 = 100.0 + squaredSide1 - squaredSide2
										- 10.0 * sqrt(3.0) * sqrt(squaredSide1);

									float check2 = 100.0 + squaredSide1 - squaredSide2
										+ 10.0 * sqrt(3.0) * sqrt(squaredSide1);

									if (check1 < 0 && check2 > 0) {
										isInLaserFOV = true;
									}
								}
							}
						}

						if (isInLaserFOV) {
							laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j
								+ laserCloudWidth * laserCloudHeight * k;
							laserCloudValidNum++;
						}
						laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j
							+ laserCloudWidth * laserCloudHeight * k;
						laserCloudSurroundNum++;
					}
				}
			}
		}

		laserCloudCornerFromMap->clear();
		laserCloudSurfFromMap->clear();
		for (int i = 0; i < laserCloudValidNum; i++) {
			*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
			*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
		}
		int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
		int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

		int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();
		for (int i = 0; i < laserCloudCornerStackNum2; i++) {
			pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
		}

		int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();
		for (int i = 0; i < laserCloudSurfStackNum2; i++) {
			pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
		}

		laserCloudCornerStack->clear();
		downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
		downSizeFilterCorner.filter(*laserCloudCornerStack);
		int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

		laserCloudSurfStack->clear();
		downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
		downSizeFilterSurf.filter(*laserCloudSurfStack);
		int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

		laserCloudCornerStack2->clear();
		laserCloudSurfStack2->clear();

		if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100) {
			kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
			kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

			for (int iterCount = 0; iterCount < 10; iterCount++) {
				laserCloudOri->clear();
				//laserCloudSel->clear();
				//laserCloudCorr->clear();
				//laserCloudProj->clear();
				coeffSel->clear();

				for (int i = 0; i < laserCloudCornerStackNum; i++) {
					pointOri = laserCloudCornerStack->points[i];
					pointAssociateToMap(&pointOri, &pointSel);
					kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

					if (pointSearchSqDis[4] < 1.0) {
						float cx = 0;
						float cy = 0;
						float cz = 0;
						for (int j = 0; j < 5; j++) {
							cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
							cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
							cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
						}
						cx /= 5;
						cy /= 5;
						cz /= 5;

						float a11 = 0;
						float a12 = 0;
						float a13 = 0;
						float a22 = 0;
						float a23 = 0;
						float a33 = 0;
						for (int j = 0; j < 5; j++) {
							float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
							float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
							float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

							a11 += ax * ax;
							a12 += ax * ay;
							a13 += ax * az;
							a22 += ay * ay;
							a23 += ay * az;
							a33 += az * az;
						}
						a11 /= 5;
						a12 /= 5;
						a13 /= 5;
						a22 /= 5;
						a23 /= 5;
						a33 /= 5;

						matA1.at<float>(0, 0) = a11;
						matA1.at<float>(0, 1) = a12;
						matA1.at<float>(0, 2) = a13;
						matA1.at<float>(1, 0) = a12;
						matA1.at<float>(1, 1) = a22;
						matA1.at<float>(1, 2) = a23;
						matA1.at<float>(2, 0) = a13;
						matA1.at<float>(2, 1) = a23;
						matA1.at<float>(2, 2) = a33;

						cv::eigen(matA1, matD1, matV1);

						if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(1, 0)) {

							float x0 = pointSel.x;
							float y0 = pointSel.y;
							float z0 = pointSel.z;
							float x1 = cx + 0.1 * matV1.at<float>(0, 0);
							float y1 = cy + 0.1 * matV1.at<float>(0, 1);
							float z1 = cz + 0.1 * matV1.at<float>(0, 2);
							float x2 = cx - 0.1 * matV1.at<float>(0, 0);
							float y2 = cy - 0.1 * matV1.at<float>(0, 1);
							float z2 = cz - 0.1 * matV1.at<float>(0, 2);

							float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
								* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
								+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
								* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
								+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
								* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

							float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

							float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
								+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

							float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
								- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

							float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
								+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

							float ld2 = a012 / l12;

							pointProj = pointSel;
							pointProj.x -= la * ld2;
							pointProj.y -= lb * ld2;
							pointProj.z -= lc * ld2;

							float s = 1 - 0.9 * fabs(ld2);

							coeff.x = s * la;
							coeff.y = s * lb;
							coeff.z = s * lc;
							coeff.intensity = s * ld2;

							if (s > 0.1) {
								laserCloudOri->push_back(pointOri);
								//laserCloudSel->push_back(pointSel);
								//laserCloudProj->push_back(pointProj);
								//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[0]]);
								//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[1]]);
								//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[2]]);
								//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[3]]);
								//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[4]]);
								coeffSel->push_back(coeff);
							}
						}
					}
				}

				for (int i = 0; i < laserCloudSurfStackNum; i++) {
					pointOri = laserCloudSurfStack->points[i];
					pointAssociateToMap(&pointOri, &pointSel);
					kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

					if (pointSearchSqDis[4] < 1.0) {
						for (int j = 0; j < 5; j++) {
							matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
							matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
							matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
						}
						cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

						float pa = matX0.at<float>(0, 0);
						float pb = matX0.at<float>(1, 0);
						float pc = matX0.at<float>(2, 0);
						float pd = 1;

						float ps = sqrt(pa * pa + pb * pb + pc * pc);
						pa /= ps;
						pb /= ps;
						pc /= ps;
						pd /= ps;

						bool planeValid = true;
						for (int j = 0; j < 5; j++) {
							if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
								pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
								pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
								planeValid = false;
								break;
							}
						}

						if (planeValid) {
							float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

							pointProj = pointSel;
							pointProj.x -= pa * pd2;
							pointProj.y -= pb * pd2;
							pointProj.z -= pc * pd2;

							float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
								+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));

							coeff.x = s * pa;
							coeff.y = s * pb;
							coeff.z = s * pc;
							coeff.intensity = s * pd2;

							if (s > 0.1) {
								laserCloudOri->push_back(pointOri);
								//laserCloudSel->push_back(pointSel);
								//laserCloudProj->push_back(pointProj);
								//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[0]]);
								//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[1]]);
								//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[2]]);
								//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[3]]);
								//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[4]]);
								coeffSel->push_back(coeff);
							}
						}
					}
				}

				float srx = sin(transformTobeMapped[0]);
				float crx = cos(transformTobeMapped[0]);
				float sry = sin(transformTobeMapped[1]);
				float cry = cos(transformTobeMapped[1]);
				float srz = sin(transformTobeMapped[2]);
				float crz = cos(transformTobeMapped[2]);

				int laserCloudSelNum = laserCloudOri->points.size();
				if (laserCloudSelNum < 50) {
					continue;
				}

				cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
				cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
				cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
				cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
				cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
				cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
				for (int i = 0; i < laserCloudSelNum; i++) {
					pointOri = laserCloudOri->points[i];
					coeff = coeffSel->points[i];

					float arx = (crx*sry*srz*pointOri.x + crx * crz*sry*pointOri.y - srx * sry*pointOri.z) * coeff.x
						+ (-srx * srz*pointOri.x - crz * srx*pointOri.y - crx * pointOri.z) * coeff.y
						+ (crx*cry*srz*pointOri.x + crx * cry*crz*pointOri.y - cry * srx*pointOri.z) * coeff.z;

					float ary = ((cry*srx*srz - crz * sry)*pointOri.x
						+ (sry*srz + cry * crz*srx)*pointOri.y + crx * cry*pointOri.z) * coeff.x
						+ ((-cry * crz - srx * sry*srz)*pointOri.x
							+ (cry*srz - crz * srx*sry)*pointOri.y - crx * sry*pointOri.z) * coeff.z;

					float arz = ((crz*srx*sry - cry * srz)*pointOri.x + (-cry * crz - srx * sry*srz)*pointOri.y)*coeff.x
						+ (crx*crz*pointOri.x - crx * srz*pointOri.y) * coeff.y
						+ ((sry*srz + cry * crz*srx)*pointOri.x + (crz*sry - cry * srx*srz)*pointOri.y)*coeff.z;

					matA.at<float>(i, 0) = arx;
					matA.at<float>(i, 1) = ary;
					matA.at<float>(i, 2) = arz;
					matA.at<float>(i, 3) = coeff.x;
					matA.at<float>(i, 4) = coeff.y;
					matA.at<float>(i, 5) = coeff.z;
					matB.at<float>(i, 0) = -coeff.intensity;
				}
				cv::transpose(matA, matAt);
				matAtA = matAt * matA;
				matAtB = matAt * matB;
				cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

				if (iterCount == 0) {
					cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
					cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
					cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

					cv::eigen(matAtA, matE, matV);
					matV.copyTo(matV2);

					isDegenerate = false;
					float eignThre[6] = { 100, 100, 100, 100, 100, 100 };
					for (int i = 5; i >= 0; i--) {
						if (matE.at<float>(i, 0) < eignThre[i]) {
							for (int j = 0; j < 6; j++) {
								matV2.at<float>(i, j) = 0;
							}
							isDegenerate = true;
						}
						else {
							break;
						}
					}
					matP = matV.inv() * matV2;
				}

				if (isDegenerate /*&& 0*/) {
					cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
					matX.copyTo(matX2);
					matX = matP * matX2;

					//ROS_INFO ("laser mapping degenerate");
				}

				transformTobeMapped[0] += matX.at<float>(0, 0);
				transformTobeMapped[1] += matX.at<float>(1, 0);
				transformTobeMapped[2] += matX.at<float>(2, 0);
				transformTobeMapped[3] += matX.at<float>(3, 0);
				transformTobeMapped[4] += matX.at<float>(4, 0);
				transformTobeMapped[5] += matX.at<float>(5, 0);

				float deltaR = sqrt(matX.at<float>(0, 0) * 180 / CV_PI * matX.at<float>(0, 0) * 180 / CV_PI
					+ matX.at<float>(1, 0) * 180 / CV_PI * matX.at<float>(1, 0) * 180 / CV_PI
					+ matX.at<float>(2, 0) * 180 / CV_PI * matX.at<float>(2, 0) * 180 / CV_PI);
				float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
					+ matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
					+ matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);

				if (deltaR < 0.05 && deltaT < 0.05) {
					break;
				}

				//ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
			}

			transformUpdate();
		}

		for (int i = 0; i < laserCloudCornerStackNum; i++) {
			pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

			int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
			int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
			int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

			if (pointSel.x + 25.0 < 0) cubeI--;
			if (pointSel.y + 25.0 < 0) cubeJ--;
			if (pointSel.z + 25.0 < 0) cubeK--;

			if (cubeI >= 0 && cubeI < laserCloudWidth &&
				cubeJ >= 0 && cubeJ < laserCloudHeight &&
				cubeK >= 0 && cubeK < laserCloudDepth) {
				int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
				laserCloudCornerArray[cubeInd]->push_back(pointSel);
			}
		}

		for (int i = 0; i < laserCloudSurfStackNum; i++) {
			pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

			int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
			int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
			int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

			if (pointSel.x + 25.0 < 0) cubeI--;
			if (pointSel.y + 25.0 < 0) cubeJ--;
			if (pointSel.z + 25.0 < 0) cubeK--;

			if (cubeI >= 0 && cubeI < laserCloudWidth &&
				cubeJ >= 0 && cubeJ < laserCloudHeight &&
				cubeK >= 0 && cubeK < laserCloudDepth) {
				int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
				laserCloudSurfArray[cubeInd]->push_back(pointSel);
			}
		}//把当前帧的特征点云封装在不同的CUBE中

		for (int i = 0; i < laserCloudValidNum; i++) {
			int ind = laserCloudValidInd[i];

			laserCloudCornerArray2[ind]->clear();
			downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
			downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

			laserCloudSurfArray2[ind]->clear();
			downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
			downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

			pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
			laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
			laserCloudCornerArray2[ind] = laserCloudTemp;

			laserCloudTemp = laserCloudSurfArray[ind];
			laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
			laserCloudSurfArray2[ind] = laserCloudTemp;
		}

		mapFrameCount++;
		if (mapFrameCount >= mapFrameNum) {
			mapFrameCount = 0;

			laserCloudSurround2->clear();
			for (int i = 0; i < laserCloudSurroundNum; i++) {
				int ind = laserCloudSurroundInd[i];
				*laserCloudSurround2 += *laserCloudCornerArray[ind];
				*laserCloudSurround2 += *laserCloudSurfArray[ind];
			}

			laserCloudSurround->clear();
			downSizeFilterCorner.setInputCloud(laserCloudSurround2);
			downSizeFilterCorner.filter(*laserCloudSurround);
		}

		int laserCloudFullResNum = laserCloudFullRes->points.size();
		for (int i = 0; i < laserCloudFullResNum; i++) {
			pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);//laserCloudFullRes1是/velodyne_cloud_3转换得到的，在这儿将这些点转到世界坐标系下,得到registrated图
		}
//		std::cout << transformAftMapped[0] << " " << transformAftMapped[1] << " " << transformAftMapped[2] << std::endl;
//		std::cout << transformAftMapped[3] << " " << transformAftMapped[4] << " " << transformAftMapped[5] << std::endl;

		mapMessage.laser_cloud_surround = laserCloudSurround;
		mapMessage.velodyne_cloud_registered = laserCloudFullRes;
		for (int i = 0; i < 6; i++)
			mapMessage.transformAftMapped[i] = transformAftMapped[i];
	}
	return mapMessage;
}
