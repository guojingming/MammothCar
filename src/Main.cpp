#include "Mammoth.h"

using namespace mammoth::layer;
using Eigen::MatrixXd;

void ryh_test();
void gjm_test();


int main(int argc, char ** argv) {
	
	ryh_test();
	//gjm_test();


	return 0;
}

void gjm_test() {
	//MultipleLidarViewer::showVelodyne16and32Points(0,1);
	//�µ�pcd��ȡ����
	//PointViewer::get_instance()->init_point_viewer();
	/*PCDFILE f;
	PcdUtil::read_pcd_file("C:\\DataSpace\\map\\0406-1.pcd", &f);
	PointViewer::get_instance()->set_point_cloud(f);
	system("pause");*/
	//��¼����
	//PointViewer::get_instance()->init_point_viewer();
	//DataGatherLayer::get_instance()->start_grab("E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\gps_data", "E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\pcd_data", "E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\imu_data", "E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\ori_imu_data");
	
	//�����㷨
	PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	
	//PcdUtil::read_pcd_file("D:\\t3.pcd", cloud);
	PcdUtil::read_pcd_file("D:\\2.pcd", cloud);
	for (int i = 0; i < cloud->size(); i++) {
		
			(*cloud)[i].g = 255;
			(*cloud)[i].r = 255;
			(*cloud)[i].b = 255;

	}


	//DimensionReductionCluster::start_clusting(cloud);
	//printf("%d\n", cloud->size());
	PointViewer::get_instance()->set_point_cloud(cloud);
	system("pause");
	/*
	pcl::fromPCLPointCloud2();*/
	//PCDתXYZ
	//PcdUtil::trans_pcd_to_xyz("D:/map.pcd","D:/map.xyz");
	

	//SLAM
	//JluSlamLayer::get_instance()->start_slam("E:\\DataSpace\\LidarDataSpace\\new_lidar_20180226-1\\gps_data","E:\\DataSpace\\LidarDataSpace\\new_lidar_20180226-1\\pcd_data");
} 

void ryh_test() {
	PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_dev_handle(2);
	PcapTransformLayer::get_instance()->get_current_frame_CE30D(device, cloud, 0);
	PointViewer::get_instance()->set_point_cloud(cloud);
	system("pause");
}