#include"scanRegistration.h"

int scanRegistration::getRingForAngle(const float & angle)
{
	float factor = (32 - 1) / (10.67f - (-30.67f));
	return int(((angle * 180 / M_PI) - (-30.67f))*factor + 0.5);
}

int scanRegistration::CAfilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZRGB>());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, 1)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, -1)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, 1)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, -1)));
	/*range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, 0.5)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, -0.5)));
	*/
	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(cloud);
	condrem.filter(*cloud_filtered);

	return 1;

}

int scanRegistration::laserHandler(pcl::PointCloud<pcl::PointXYZRGB>::Ptr velodyne_points)
{
	laserCloudIn->clear();
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*velodyne_points, *velodyne_points, indices);
	for (int k = 0; k < velodyne_points->points.size(); k++)
	{
		if (abs(velodyne_points->points[k].x) == 0 && abs(velodyne_points->points[k].y) == 0 && abs(velodyne_points->points[k].z) == 0)
		{
			continue;
		}
		laserCloudIn->push_back(velodyne_points->points[k]);
	}
	//CAfilter(velodyne_points, laserCloudIn);
	return 1;
}

int scanRegistration::divideBeam(pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudIn)
{
	int cloudSize = laserCloudIn->points.size();
	float startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
	float endOri = -atan2(laserCloudIn->points[cloudSize - 1].y,
		laserCloudIn->points[cloudSize - 1].x) + 2 * float(M_PI);
	if (endOri - startOri > 3 * M_PI) {
		endOri -= 2 * M_PI;
	}
	else if (endOri - startOri < M_PI) {
		endOri += 2 * M_PI;
	}
	bool halfPassed = false;
	int count = cloudSize;
	pcl::PointXYZI point;
	pcl::PointXYZI pointTemp;
	std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans(N_SCANS);

	for (int i = 0; i < cloudSize; i++) {
		//ROSתJiZhangʱ�������GJM2�����һ��
		point.x = -laserCloudIn->points[i].x;
		point.y = laserCloudIn->points[i].z;
		point.z = laserCloudIn->points[i].y;

		if (!pcl_isfinite(point.x) ||
			!pcl_isfinite(point.y) ||
			!pcl_isfinite(point.z)) {
			count--;
			continue;
		}
		if (point.x*point.x + point.y*point.y + point.z*point.z < 0.0001) {
			count--;
			continue;
		}
		//float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z));
		//int scanID = getRingForAngle(angle);
		//if (scanID > (N_SCANS - 1) || scanID < 0) {
		//	count--; // ��16��������ӵ��޳�
		//	continue;
		//}
		int scanID= -(*reinterpret_cast<uint8_t*>(&laserCloudIn->points[i].g)) + N_SCANS -1;//+39 40�ߣ�int scanID= -(*reinterpret_cast<uint8_t*>(&laserCloudIn->points[i].g)) + N_SCANS-1

		float ori = -atan2(point.x, point.z);
		if (!halfPassed) {
			if (ori < startOri - M_PI / 2) {
				ori += 2 * M_PI;
			}
			else if (ori > startOri + M_PI * 3 / 2) {
				ori -= 2 * M_PI;
			}

			if (ori - startOri > M_PI) {
				halfPassed = true;
			}
		}
		else {
			ori += 2 * M_PI;

			if (ori < endOri - M_PI * 3 / 2) {
				ori += 2 * M_PI;
			}
			else if (ori > endOri + M_PI / 2) {
				ori -= 2 * M_PI;
			}
		}
		float relTime = (ori - startOri) / (endOri - startOri);
		point.intensity = *reinterpret_cast<uint8_t*>(&laserCloudIn->points[i].r) * 100 + scanID + scanPeriod * relTime;
		laserCloudScans[scanID].push_back(point);
	}
	cloudSize = count;
	laserCloud->clear();
	for (int i = 0; i < N_SCANS; i++) {
		*laserCloud += laserCloudScans[i];
	}
	return 1;
}

int scanRegistration::extractFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud)
{
	std::vector<int> scanStartInd(N_SCANS, 0);
	std::vector<int> scanEndInd(N_SCANS, 0);
	int scanCount = -1;
	/*������������������������������������������  ��¼���Ƶ����� & ��¼ÿһ�������������ʼ����ֹ ������������������������������������������������*/
	for (int i = 5; i < laserCloud->points.size() - 5; i++) {
		// �����еļ����һ��һ������ڸõ�ǰ��5����(10��)��ƫ���ΪcloudCurvature�������ݵ�����
		float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
			+ laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
			+ laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
			+ laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
			+ laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
			+ laserCloud->points[i + 5].x;
		float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
			+ laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
			+ laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
			+ laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
			+ laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
			+ laserCloud->points[i + 5].y;
		float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
			+ laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
			+ laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
			+ laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
			+ laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
			+ laserCloud->points[i + 5].z;
		cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
		cloudSortInd[i] = i;
		cloudNeighborPicked[i] = 0;
		cloudLabel[i] = 0;

		if (int(laserCloud->points[i].intensity) % 100 != scanCount) {
			scanCount = int(laserCloud->points[i].intensity) % 100;

			// ��¼ÿһ����ʼ�����ֹ���λ�á���[4]&[end-4]��Ϊʲô��ô�洢�أ���Ϊ������Ҫ���������ʼ/��ֹ�������������ʣ��������ʵĹ������Ѿ�ȥ����ǰ5����ͺ�5����
			if (scanCount > 0 && scanCount < N_SCANS) {
				scanStartInd[scanCount] = i + 5;
				scanEndInd[scanCount - 1] = i - 5;

			}
		}
	}
	scanStartInd[0] = 5;
	scanEndInd.back() = laserCloud->points.size() - 5;

	for (int i = 5; i < laserCloud->points.size() - 6; i++) {
		float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
		float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
		float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
		float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

		if (diff > 0.1) {

			float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
				laserCloud->points[i].y * laserCloud->points[i].y +
				laserCloud->points[i].z * laserCloud->points[i].z);

			float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
				laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
				laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

			/*�� ������ĵ�(b)������������н�С��ĳ��ֵbʱ���н�С�Ϳ��ܴ����ڵ���������һ����ٽ�6������Ϊ���ɱ��Ϊ������ĵ� ��*/
			/*�� Ȼ�󹹽���һ�����������εĵ����������ݵ������������ʣ��ж�X[i]������X[i+1]�ļн�С��5.732��(threshold=0.1) ��*/
			/*�� depth1>depth2 X[i+1]���������Զ����ǲ�������depth1<depth2 X[i]���������Զ����ǲ����� ��*/
			if (depth1 > depth2) {
				diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
				diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
				diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
					cloudNeighborPicked[i - 5] = 1;
					cloudNeighborPicked[i - 4] = 1;
					cloudNeighborPicked[i - 3] = 1;
					cloudNeighborPicked[i - 2] = 1;
					cloudNeighborPicked[i - 1] = 1;
					cloudNeighborPicked[i] = 1;
				}
			}
			else {
				diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
				diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
				diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
					cloudNeighborPicked[i + 1] = 1;
					cloudNeighborPicked[i + 2] = 1;
					cloudNeighborPicked[i + 3] = 1;
					cloudNeighborPicked[i + 4] = 1;
					cloudNeighborPicked[i + 5] = 1;
					cloudNeighborPicked[i + 6] = 1;
				}
			}
		}

		/*�� ������ĵ�(a)�������ĳ�㼰�����ľ���ƽ������ĳ��ֵa��˵����������һ�����룩 ������*/
		/*�� ��ĳ�㵽��ǰ������ľ��������c���ĸõ���ȣ���õ��ж�Ϊ���ɱ��������ĵ� ��������������*/
		/*���������ԽС������Խ�󣬼����ⷢ�䷽����Ͷ�䵽��ƽ��Խ����ˮƽ�� ������������������������������*/

		float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
		float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
		float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
		float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

		float dis = laserCloud->points[i].x * laserCloud->points[i].x
			+ laserCloud->points[i].y * laserCloud->points[i].y
			+ laserCloud->points[i].z * laserCloud->points[i].z;

		if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
			cloudNeighborPicked[i] = 1;
		}
	}

	/*������������������������������������������  ��¼���Ƶ������ǵ� ������������������������������������������������*/
	/*pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp;
	pcl::PointCloud<pcl::PointXYZI> cornerPointsLessSharp;
	pcl::PointCloud<pcl::PointXYZI> surfPointsFlat;
	pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlat;*/
	cornerPointsSharp->clear();
	cornerPointsLessSharp->clear();
	surfPointsFlat->clear();
	surfPointsLessFlat->clear();

	for (int i = 0; i < N_SCANS; i++) {
		/*���� ����ÿһ�㼤���(��16��)����ÿ������ֳ�6�ݣ���ʼλ��Ϊsp����ֹλ��Ϊep��������������*/
		/*���� ������ѭ���������Ƕ�cloudCurvature��С�����������cloudSortedInd�������������� ��������*/
		pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
		for (int j = 0; j < 6; j++) {
			int sp = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / 6;
			int ep = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / 6 - 1;

			for (int k = sp + 1; k <= ep; k++) {
				for (int l = k; l >= sp + 1; l--) {
					if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
						int temp = cloudSortInd[l - 1];
						cloudSortInd[l - 1] = cloudSortInd[l];
						cloudSortInd[l] = temp;
					}
				}
			}
			/*���� ɸѡ�����ǵ� Corner: label=2; LessCorner: label=1 ��������*/
			int largestPickedNum = 0;
			for (int k = ep; k >= sp; k--) {
				int ind = cloudSortInd[k];
				if (cloudNeighborPicked[ind] == 0 &&
					cloudCurvature[ind] > 0.1) {

					largestPickedNum++;
					if (largestPickedNum <= 2) {
						cloudLabel[ind] = 2;
						cornerPointsSharp->push_back(laserCloud->points[ind]);
						cornerPointsLessSharp->push_back(laserCloud->points[ind]);
					}
					else if (largestPickedNum <= 20) {
						cloudLabel[ind] = 1;
						cornerPointsLessSharp->push_back(laserCloud->points[ind]);
					}
					else {
						break;
					}

					// ���������ʵ�󣬽��õ��ǣ����������ʵ㸽����ǰ��5�����ǲ���ѡȡΪ������
					cloudNeighborPicked[ind] = 1;
					for (int l = 1; l <= 5; l++) {
						float diffX = laserCloud->points[ind + l].x
							- laserCloud->points[ind + l - 1].x;
						float diffY = laserCloud->points[ind + l].y
							- laserCloud->points[ind + l - 1].y;
						float diffZ = laserCloud->points[ind + l].z
							- laserCloud->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}
					for (int l = -1; l >= -5; l--) {
						float diffX = laserCloud->points[ind + l].x
							- laserCloud->points[ind + l + 1].x;
						float diffY = laserCloud->points[ind + l].y
							- laserCloud->points[ind + l + 1].y;
						float diffZ = laserCloud->points[ind + l].z
							- laserCloud->points[ind + l + 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}
				}
			}


			/*���� ɸѡ����ƽ��� Flat: label=-1 ��ͨ���Flat�㽵�����γ�LessFlat: label=0 ��������*/
			int smallestPickedNum = 0;
			for (int k = sp; k <= ep; k++) {
				int ind = cloudSortInd[k];
				if (cloudNeighborPicked[ind] == 0 &&
					cloudCurvature[ind] < 0.1) {

					cloudLabel[ind] = -1;
					surfPointsFlat->push_back(laserCloud->points[ind]);

					smallestPickedNum++;
					if (smallestPickedNum >= 4) {
						break;
					}

					cloudNeighborPicked[ind] = 1;
					for (int l = 1; l <= 5; l++) {
						float diffX = laserCloud->points[ind + l].x
							- laserCloud->points[ind + l - 1].x;
						float diffY = laserCloud->points[ind + l].y
							- laserCloud->points[ind + l - 1].y;
						float diffZ = laserCloud->points[ind + l].z
							- laserCloud->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}
					for (int l = -1; l >= -5; l--) {
						float diffX = laserCloud->points[ind + l].x
							- laserCloud->points[ind + l + 1].x;
						float diffY = laserCloud->points[ind + l].y
							- laserCloud->points[ind + l + 1].y;
						float diffZ = laserCloud->points[ind + l].z
							- laserCloud->points[ind + l + 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}
				}
			}

			// surfPointsLessFlatΪ���������flat�㣬����ǰ����̫��label=0�ĵ�
			for (int k = sp; k <= ep; k++) {
				if (cloudLabel[k] <= 0) {
					surfPointsLessFlatScan->push_back(laserCloud->points[k]);
				}
			}
		}

		pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
		downSizeFilter.setInputCloud(surfPointsLessFlatScan);
		downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
		downSizeFilter.filter(surfPointsLessFlatScanDS);

		for (int i = 0; i < surfPointsLessFlatScanDS.size(); i++)
		{
			surfPointsLessFlat->push_back(surfPointsLessFlatScanDS.points[i]);
		}
	}
	return 1;
}

registrationMessage scanRegistration::registrationProcess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr velodyne_points)
{
	laserHandler(velodyne_points);
	divideBeam(laserCloudIn);
	extractFeatures(laserCloud);
	
	registrationMessage regMessage;
	regMessage.laser_cloud_flat = surfPointsFlat;
	regMessage.laser_cloud_less_flat = surfPointsLessFlat;
	regMessage.laser_cloud_sharp = cornerPointsSharp;
	regMessage.laser_cloud_less_sharp = cornerPointsLessSharp;
	regMessage.velodyne_cloud_2 = laserCloud;
	return regMessage;
}
