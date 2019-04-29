#include<iostream>
#include "scanRegistration.h"
#include "laserOdometry.h"
#include "laserMapping.h"

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);

	scanRegistration scanregistration;
	laserOdometry laserodometry;
	laserMapping lasermapping;

	registrationMessage regismessage;
	odometryMessage odomessage;
	mappingMessage mapmessage;

	for (int i = 1; i <= 530; i++)
	{
		pcl::io::loadPCDFile("./20180430/"+std::to_string(i)+".pcd", *cloud_in);
		int64 TScan = cv::getTickCount();
		regismessage = scanregistration.registrationProcess(cloud_in);
		TScan = cv::getTickCount() - TScan;
		std::cout << "！！！！！！！！！！ ScanRegistration's time ！！！！！！！！！:" << TScan * 1000 / cv::getTickFrequency() << "ms" << std::endl;
		int64 TOdometry = cv::getTickCount();
		odomessage = laserodometry.odometryProcess(regismessage);
		TOdometry = cv::getTickCount() - TOdometry;
		std::cout << "！！！！！！！！！！  LaserOdometry's time   ！！！！！！！！！:" << TOdometry * 1000 / cv::getTickFrequency() << "ms" << std::endl;

		std::string filename = "./20180430 OOD/";
		laserodometry.save2txt(odomessage.transform, i,filename);
		filename = "./20180430ODOTrans/";
		laserodometry.save2pcd(odomessage.velodyne_cloud_3, i,filename);
		//耽匯屐憧匯肝
		int64 TMapping = cv::getTickCount();
		mapmessage = lasermapping.mappingProcess(odomessage);
		TMapping = cv::getTickCount() - TMapping;
		std::cout << "！！！！！！！！！！   LaserMapping's time   ！！！！！！！！！:" << TMapping * 1000 / cv::getTickFrequency() << "ms" << std::endl;
		
		filename = "./20180430 top MAP/";
		lasermapping.save2pcd(mapmessage.velodyne_cloud_registered, i,filename);
		filename = "./20180430 top MAPTrans/";
		lasermapping.save2txt(mapmessage.transformAftMapped, i,filename);
		//耽侯曾屐憧匯肝
		//if (i % 3 == 2)
		//{
		//	int64 TMapping = cv::getTickCount();
		//	mapmessage = lasermapping.mappingProcess(odomessage);
		//	TMapping = cv::getTickCount() - TMapping;
		//	std::cout << "！！！！！！！！！！   LaserMapping's time   ！！！！！！！！！:" << TMapping * 1000 / cv::getTickFrequency() << "ms" << std::endl;
		//	lasermapping.save2pcd(mapmessage.velodyne_cloud_registered, i);
		//}
		
	}
	system("pause");
	return 1;
}