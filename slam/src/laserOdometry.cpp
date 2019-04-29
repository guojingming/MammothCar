#include "laserOdometry.h"


void laserOdometry::save2txt(float transform[6], int j,std::string filename)
{
	if (_access(filename.data(), 0) != 0)
	{
		_mkdir(filename.data());
		}
		std::ofstream trans(filename + std::to_string(j) + ".txt");
		trans << transform[0] << " " << transform[1] << " " << transform[2] << std::endl;
		trans << transform[3] << " " << transform[4] << " " << transform[5] << std::endl;
		std::cout << "Save2txtOdo" << std::endl;
}

void laserOdometry::save2pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes1, int j, std::string filename)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr destCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	destCloud->points.resize(laserCloudFullRes1->points.size());
	for (int k = 0; k < laserCloudFullRes1->points.size(); k++)
	{	//ROS转JiZhang时用这个
		destCloud->points[k].x = -laserCloudFullRes1->points[k].x;
		destCloud->points[k].y = laserCloudFullRes1->points[k].z;
		destCloud->points[k].z = laserCloudFullRes1->points[k].y;

		int I = laserCloudFullRes1->points[k].intensity;
		destCloud->points[k].g = -(I % 100) + 31;    //40线：39  32线：32
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
	std::cout << "after odo :" << j << std::endl;
}

void laserOdometry::laserCloudSharpHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_sharp)
{
	//int temp = laser_cloud_sharp->points.size();
	cornerPointsSharp->clear();
	*cornerPointsSharp = *laser_cloud_sharp;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
	int temp1 = cornerPointsSharp->points.size();
}

void laserOdometry::laserCloudLessSharpHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_less_sharp)
{
	cornerPointsLessSharp->clear();
	*cornerPointsLessSharp = *laser_cloud_less_sharp;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, indices);
}

void laserOdometry::laserCloudFlatHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_flat)
{
	surfPointsFlat->clear();
	*surfPointsFlat = *laser_cloud_flat;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*surfPointsFlat, *surfPointsFlat, indices);
}

void laserOdometry::laserCloudLessFlatHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_less_flat)
{
	surfPointsLessFlat->clear();
	*surfPointsLessFlat = *laser_cloud_less_flat;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, indices);
}

void laserOdometry::laserCloudFullResHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_2)
{
	laserCloudFullRes->clear();
	*laserCloudFullRes = *velodyne_cloud_2;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudFullRes, *laserCloudFullRes, indices);
}

void laserOdometry::TransformToStart(pcl::PointXYZI * pi, pcl::PointXYZI * po)
{
	float s = 10 * (pi->intensity - int(pi->intensity));

	float rx = s * transform[0];
	float ry = s * transform[1];
	float rz = s * transform[2];
	float tx = s * transform[3];
	float ty = s * transform[4];
	float tz = s * transform[5];

	float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
	float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
	float z1 = (pi->z - tz);

	float x2 = x1;
	float y2 = cos(rx) * y1 + sin(rx) * z1;
	float z2 = -sin(rx) * y1 + cos(rx) * z1;

	po->x = cos(ry) * x2 - sin(ry) * z2;
	po->y = y2;
	po->z = sin(ry) * x2 + cos(ry) * z2;
	po->intensity = pi->intensity;
}

void laserOdometry::TransformToEnd(pcl::PointXYZI * pi, pcl::PointXYZI * po)
{
	float s = 10 * (pi->intensity - int(pi->intensity));

	float rx = s * transform[0];
	float ry = s * transform[1];
	float rz = s * transform[2];
	float tx = s * transform[3];
	float ty = s * transform[4];
	float tz = s * transform[5];

	float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
	float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
	float z1 = (pi->z - tz);

	float x2 = x1;
	float y2 = cos(rx) * y1 + sin(rx) * z1;
	float z2 = -sin(rx) * y1 + cos(rx) * z1;

	float x3 = cos(ry) * x2 - sin(ry) * z2;
	float y3 = y2;
	float z3 = sin(ry) * x2 + cos(ry) * z2;

	rx = transform[0];
	ry = transform[1];
	rz = transform[2];
	tx = transform[3];
	ty = transform[4];
	tz = transform[5];

	float x4 = cos(ry) * x3 + sin(ry) * z3;
	float y4 = y3;
	float z4 = -sin(ry) * x3 + cos(ry) * z3;

	float x5 = x4;
	float y5 = cos(rx) * y4 - sin(rx) * z4;
	float z5 = sin(rx) * y4 + cos(rx) * z4;

	float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
	float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
	float z6 = z5 + tz;

	po->x = x6;
	po->y = y6;
	po->z = z6;
	po->intensity = int(pi->intensity);
}

void laserOdometry::AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, float & ox, float & oy, float & oz)
{
	float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
	ox = -asin(srx);

	float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz)
		+ sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
	float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy)
		- cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
	oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

	float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz)
		+ sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
	float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz)
		- cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
	oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

odometryMessage laserOdometry::odometryProcess(registrationMessage regisMessage)
{
	laserCloudSharpHandler(regisMessage.laser_cloud_sharp);
	laserCloudLessSharpHandler(regisMessage.laser_cloud_less_sharp);
	laserCloudFlatHandler(regisMessage.laser_cloud_flat);
	laserCloudLessFlatHandler(regisMessage.laser_cloud_flat);
	laserCloudFullResHandler(regisMessage.velodyne_cloud_2);

	odometryMessage odoMessage;

	std::vector<int> pointSearchInd;
	std::vector<float> pointSearchSqDis;
	pcl::PointXYZI pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;
	bool isDegenerate = false;
	cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
	int frameCount = skipFrameNum;
	if (!systemInited)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
		cornerPointsLessSharp = laserCloudCornerLast;
		laserCloudCornerLast = laserCloudTemp;

		laserCloudTemp = surfPointsLessFlat;
		surfPointsLessFlat = laserCloudSurfLast;
		laserCloudSurfLast = laserCloudTemp;

		kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
		kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

		laserCloudCornerLastNum = laserCloudCornerLast->size();
		laserCloudSurfLastNum = laserCloudSurfLast->size();

		odoMessage.laser_cloud_corner_last = laserCloudCornerLast;
		odoMessage.laser_cloud_surf_last = laserCloudSurfLast;
		odoMessage.velodyne_cloud_3 = laserCloudFullRes;
		memset(transformSum, 0, 6);
		memset(transform, 0, 6);
		for (int k = 0; k < 6; k++)
		{
			odoMessage.transform[k] = transform[k];
			odoMessage.transformSum[k] = transformSum[k];
		}
		systemInited = true;
		return odoMessage;
	}
//	std::cout << "TransformBef    " << transform[0] << " " << transform[1] << " " << transform[2] << std::endl;
//	std::cout << "                " << transform[3] << " " << transform[4] << " " << transform[5] << std::endl;
	if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
		int cornerPointsSharpNum = cornerPointsSharp->points.size();
		int surfPointsFlatNum = surfPointsFlat->points.size();

		for (int iterCount = 0; iterCount < 25; iterCount++) {
			laserCloudOri->clear();
			coeffSel->clear();
			for (int i = 0; i < cornerPointsSharpNum; i++) {
				TransformToStart(&cornerPointsSharp->points[i], &pointSel);
				//	pointSel = cornerPointsSharp->points[i];

				if (iterCount % 5 == 0) {
					std::vector<int> indices;
					pcl::removeNaNFromPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, indices);
					kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

					int closestPointInd = -1, minPointInd2 = -1;
					if (pointSearchSqDis[0] < 25) {
						closestPointInd = pointSearchInd[0];
						int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity) % 100;

						float pointSqDis, minPointSqDis2 = 25;
					//	for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
						for (int j = closestPointInd + 1; j < laserCloudCornerLastNum; j++) {//laserCloudCornerLastNum=1641,而laserCloudCornerLast才是1560
							if (int(laserCloudCornerLast->points[j].intensity) % 100 > closestPointScan + 2.5) {
								break;
							}

							pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
								(laserCloudCornerLast->points[j].x - pointSel.x) +
								(laserCloudCornerLast->points[j].y - pointSel.y) *
								(laserCloudCornerLast->points[j].y - pointSel.y) +
								(laserCloudCornerLast->points[j].z - pointSel.z) *
								(laserCloudCornerLast->points[j].z - pointSel.z);

							if (int(laserCloudCornerLast->points[j].intensity) % 100 > closestPointScan) {
								if (pointSqDis < minPointSqDis2) {
									minPointSqDis2 = pointSqDis;
									minPointInd2 = j;
								}
							}
						}
						for (int j = closestPointInd - 1; j >= 0; j--) {
							if (int(laserCloudCornerLast->points[j].intensity) % 100 < closestPointScan - 2.5) {
								break;
							}

							pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
								(laserCloudCornerLast->points[j].x - pointSel.x) +
								(laserCloudCornerLast->points[j].y - pointSel.y) *
								(laserCloudCornerLast->points[j].y - pointSel.y) +
								(laserCloudCornerLast->points[j].z - pointSel.z) *
								(laserCloudCornerLast->points[j].z - pointSel.z);

							if (int(laserCloudCornerLast->points[j].intensity) % 100 < closestPointScan) {
								if (pointSqDis < minPointSqDis2) {
									minPointSqDis2 = pointSqDis;
									minPointInd2 = j;
								}
							}
						}
					}

					pointSearchCornerInd1[i] = closestPointInd;
					pointSearchCornerInd2[i] = minPointInd2;
				}

				if (pointSearchCornerInd2[i] >= 0) {
					tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
					tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

					float x0 = pointSel.x;
					float y0 = pointSel.y;
					float z0 = pointSel.z;
					float x1 = tripod1.x;
					float y1 = tripod1.y;
					float z1 = tripod1.z;
					float x2 = tripod2.x;
					float y2 = tripod2.y;
					float z2 = tripod2.z;

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

					float s = 1;
					if (iterCount >= 5) {
						s = 1 - 1.8 * fabs(ld2);
					}

					coeff.x = s * la;
					coeff.y = s * lb;
					coeff.z = s * lc;
					coeff.intensity = s * ld2;

					if (s > 0.1 && ld2 != 0) {
						laserCloudOri->push_back(cornerPointsSharp->points[i]);
						coeffSel->push_back(coeff);
					}
				}
			}

			for (int i = 0; i < surfPointsFlatNum; i++) {
				TransformToStart(&surfPointsFlat->points[i], &pointSel);
				//	pointSel = surfPointsFlat->points[i];

				if (iterCount % 5 == 0) {
					kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

					int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
					if (pointSearchSqDis[0] < 25) {
						closestPointInd = pointSearchInd[0];
						int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity) % 100;

						float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
				//		for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
						for (int j = closestPointInd + 1; j < laserCloudSurfLastNum; j++) {
							if (int(laserCloudSurfLast->points[j].intensity) % 100 > closestPointScan + 2.5) {
								break;
							}

							pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
								(laserCloudSurfLast->points[j].x - pointSel.x) +
								(laserCloudSurfLast->points[j].y - pointSel.y) *
								(laserCloudSurfLast->points[j].y - pointSel.y) +
								(laserCloudSurfLast->points[j].z - pointSel.z) *
								(laserCloudSurfLast->points[j].z - pointSel.z);

							if (int(laserCloudSurfLast->points[j].intensity) % 100 <= closestPointScan) {
								if (pointSqDis < minPointSqDis2) {
									minPointSqDis2 = pointSqDis;
									minPointInd2 = j;
								}
							}
							else {
								if (pointSqDis < minPointSqDis3) {
									minPointSqDis3 = pointSqDis;
									minPointInd3 = j;
								}
							}
						}
						for (int j = closestPointInd - 1; j >= 0; j--) {
							if (int(laserCloudSurfLast->points[j].intensity) % 100 < closestPointScan - 2.5) {
								break;
							}

							pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
								(laserCloudSurfLast->points[j].x - pointSel.x) +
								(laserCloudSurfLast->points[j].y - pointSel.y) *
								(laserCloudSurfLast->points[j].y - pointSel.y) +
								(laserCloudSurfLast->points[j].z - pointSel.z) *
								(laserCloudSurfLast->points[j].z - pointSel.z);

							if (int(laserCloudSurfLast->points[j].intensity) % 100 >= closestPointScan) {
								if (pointSqDis < minPointSqDis2) {
									minPointSqDis2 = pointSqDis;
									minPointInd2 = j;
								}
							}
							else {
								if (pointSqDis < minPointSqDis3) {
									minPointSqDis3 = pointSqDis;
									minPointInd3 = j;
								}
							}
						}
					}

					pointSearchSurfInd1[i] = closestPointInd;
					pointSearchSurfInd2[i] = minPointInd2;
					pointSearchSurfInd3[i] = minPointInd3;
				}

				if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
					tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
					tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
					tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

					float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
						- (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
					float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
						- (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
					float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
						- (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
					float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

					float ps = sqrt(pa * pa + pb * pb + pc * pc);
					pa /= ps;
					pb /= ps;
					pc /= ps;
					pd /= ps;

					float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

					pointProj = pointSel;
					pointProj.x -= pa * pd2;
					pointProj.y -= pb * pd2;
					pointProj.z -= pc * pd2;

					float s = 1;
					if (iterCount >= 5) {
						s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
							+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));
					}

					coeff.x = s * pa;
					coeff.y = s * pb;
					coeff.z = s * pc;
					coeff.intensity = s * pd2;

					if (s > 0.1 && pd2 != 0) {
						laserCloudOri->push_back(surfPointsFlat->points[i]);
						coeffSel->push_back(coeff);
					}
				}
			}

			int pointSelNum = laserCloudOri->points.size();
			if (pointSelNum < 10) {
				continue;
			}

			cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
			cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
			cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
			cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
			cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
			cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
			for (int i = 0; i < pointSelNum; i++) {
				pointOri = laserCloudOri->points[i];
				coeff = coeffSel->points[i];

				float s = 1; //10 * (pointOri.intensity - int(pointOri.intensity));

				float srx = sin(s * transform[0]);
				float crx = cos(s * transform[0]);
				float sry = sin(s * transform[1]);
				float cry = cos(s * transform[1]);
				float srz = sin(s * transform[2]);
				float crz = cos(s * transform[2]);
				float tx = s * transform[3];
				float ty = s * transform[4];
				float tz = s * transform[5];

				float arx = (-s * crx*sry*srz*pointOri.x + s * crx*crz*sry*pointOri.y + s * srx*sry*pointOri.z
					+ s * tx*crx*sry*srz - s * ty*crx*crz*sry - s * tz*srx*sry) * coeff.x
					+ (s*srx*srz*pointOri.x - s * crz*srx*pointOri.y + s * crx*pointOri.z
						+ s * ty*crz*srx - s * tz*crx - s * tx*srx*srz) * coeff.y
					+ (s*crx*cry*srz*pointOri.x - s * crx*cry*crz*pointOri.y - s * cry*srx*pointOri.z
						+ s * tz*cry*srx + s * ty*crx*cry*crz - s * tx*crx*cry*srz) * coeff.z;
				float ary = ((-s * crz*sry - s * cry*srx*srz)*pointOri.x
					+ (s*cry*crz*srx - s * sry*srz)*pointOri.y - s * crx*cry*pointOri.z
					+ tx * (s*crz*sry + s * cry*srx*srz) + ty * (s*sry*srz - s * cry*crz*srx)
					+ s * tz*crx*cry) * coeff.x
					+ ((s*cry*crz - s * srx*sry*srz)*pointOri.x
						+ (s*cry*srz + s * crz*srx*sry)*pointOri.y - s * crx*sry*pointOri.z
						+ s * tz*crx*sry - ty * (s*cry*srz + s * crz*srx*sry)
						- tx * (s*cry*crz - s * srx*sry*srz)) * coeff.z;

				float arz = ((-s * cry*srz - s * crz*srx*sry)*pointOri.x + (s*cry*crz - s * srx*sry*srz)*pointOri.y
					+ tx * (s*cry*srz + s * crz*srx*sry) - ty * (s*cry*crz - s * srx*sry*srz)) * coeff.x
					+ (-s * crx*crz*pointOri.x - s * crx*srz*pointOri.y
						+ s * ty*crx*srz + s * tx*crx*crz) * coeff.y
					+ ((s*cry*crz*srx - s * sry*srz)*pointOri.x + (s*crz*sry + s * cry*srx*srz)*pointOri.y
						+ tx * (s*sry*srz - s * cry*crz*srx) - ty * (s*crz*sry + s * cry*srx*srz)) * coeff.z;

				float atx = -s * (cry*crz - srx * sry*srz) * coeff.x + s * crx*srz * coeff.y
					- s * (crz*sry + cry * srx*srz) * coeff.z;

				float aty = -s * (cry*srz + crz * srx*sry) * coeff.x - s * crx*crz * coeff.y
					- s * (sry*srz - cry * crz*srx) * coeff.z;

				float atz = s * crx*sry * coeff.x - s * srx * coeff.y - s * crx*cry * coeff.z;

				float d2 = coeff.intensity;

				matA.at<float>(i, 0) = arx;
				matA.at<float>(i, 1) = ary;
				matA.at<float>(i, 2) = arz;
				matA.at<float>(i, 3) = atx;
				matA.at<float>(i, 4) = aty;
				matA.at<float>(i, 5) = atz;
				matB.at<float>(i, 0) = -0.05 * d2;

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
				float eignThre[6] = { 10, 10, 10, 10, 10, 10 };
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
			if (isDegenerate) {
				cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
				matX.copyTo(matX2);
				matX = matP * matX2;

				//ROS_INFO ("laser odometry degenerate");

			}

			transform[0] += matX.at<float>(0, 0);
			transform[1] += matX.at<float>(1, 0);
			transform[2] += matX.at<float>(2, 0);
			transform[3] += matX.at<float>(3, 0);
			transform[4] += matX.at<float>(4, 0);
			transform[5] += matX.at<float>(5, 0);

			for (int i = 0; i < 6; i++)
			{
				if (isnan(transform[i]))
					transform[i] = 0;
			}

			float deltaR = sqrt(matX.at<float>(0, 0) * 180 / CV_PI * matX.at<float>(0, 0) * 180 / CV_PI
				+ matX.at<float>(1, 0) * 180 / CV_PI * matX.at<float>(1, 0) * 180 / CV_PI
				+ matX.at<float>(2, 0) * 180 / CV_PI * matX.at<float>(2, 0) * 180 / CV_PI);
			float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
				+ matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
				+ matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);

			if (deltaR < 0.1 && deltaT < 0.1) {
				break;

			}

			//ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
		}
	}
	for (int k = 0; k < 6; k++)
	{
		odoMessage.transform[k] = transform[k];
	}
//	std::cout << "TransformAft    " << transform[0] << " " << transform[1] << " " << transform[2] << std::endl;
//	std::cout << "                " << transform[3] << " " << transform[4] << " " << transform[5] << std::endl;
//	std::cout << "TransformSumBef:" << transformSum[0] << " " << transformSum[1] << " " << transformSum[2] << std::endl;
//	std::cout << "                " << transformSum[3] << " " << transformSum[4] << " " << transformSum[5] << std::endl;
	float rx, ry, rz, tx, ty, tz;
	AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
		-transform[0], -transform[1] * 1.05, -transform[2], rx, ry, rz);

	float x1 = cos(rz) * (transform[3])
		- sin(rz) * (transform[4]);
	float y1 = sin(rz) * (transform[3])
		+ cos(rz) * (transform[4]);
	float z1 = transform[5] * 1.05;

	float x2 = x1;
	float y2 = cos(rx) * y1 - sin(rx) * z1;
	float z2 = sin(rx) * y1 + cos(rx) * z1;

	tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
	ty = transformSum[4] - y2;
	tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);


	transformSum[0] = rx;
	transformSum[1] = ry;
	transformSum[2] = rz;
	transformSum[3] = tx;
	transformSum[4] = ty;
	transformSum[5] = tz;
//	std::cout << "TransformSumAft:" << transformSum[0] << " " << transformSum[1] << " " << transformSum[2] << std::endl;
//	std::cout << "                " << transformSum[3] << " " << transformSum[4] << " " << transformSum[5] << std::endl;
	//memcpy(TransFormSum, transformSum, sizeof(transformSum));
	for (int k = 0; k < 6; k++)
	{
		odoMessage.transformSum[k] = transformSum[k];
	}
	////////////////转换到世界坐标系结束
	int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();//上一团点云的if值没更新，没进去if，在这更新上一团点云
	for (int i = 0; i < cornerPointsLessSharpNum; i++) {
		TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
	}

	int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
	for (int i = 0; i < surfPointsLessFlatNum; i++) {
		TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
	}
	frameCount++;
	if (frameCount >= skipFrameNum + 1)
	{
		int laserCloudFullResNum = laserCloudFullRes->points.size();
		for (int i = 0; i < laserCloudFullResNum; i++) {
			TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
		}//将上一时刻的所有点云()s
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
	cornerPointsLessSharp = laserCloudCornerLast;
	laserCloudCornerLast = laserCloudTemp;

	laserCloudTemp = surfPointsLessFlat;
	surfPointsLessFlat = laserCloudSurfLast;
	laserCloudSurfLast = laserCloudTemp;

	laserCloudCornerLastNum = laserCloudCornerLast->points.size();
	laserCloudSurfLastNum = laserCloudSurfLast->points.size();
	if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
		kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
		kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
	}
	odoMessage.laser_cloud_corner_last = laserCloudCornerLast;
	odoMessage.laser_cloud_surf_last = laserCloudSurfLast;
	odoMessage.velodyne_cloud_3 = laserCloudFullRes;

	return odoMessage;
}
