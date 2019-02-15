#include "VisualizationLayer.h"

using namespace mammoth::layer;

int MultipleLidarViewer::lidar_count = 2;
int * MultipleLidarViewer::finish_signals = new int[lidar_count];
int * MultipleLidarViewer::grabbing_signals = new int[lidar_count];
//std::vector<pcl::PointCloud<PointType>::Ptr> MultipleLidarViewer::point_clouds;
pcl::PointCloud<PointType>::Ptr MultipleLidarViewer::vlp16_cloud_ptr = nullptr;
pcl::PointCloud<PointType>::Ptr MultipleLidarViewer::hdl32_cloud_ptr = nullptr;

int lidar_vlp16() {
	pcap_t * velodyne16_lidar = PcapTransformLayer::get_instance()->get_pcap_dev_handle(5);
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 0) = -0.0026426241;
	transform(0, 1) = -0.033511896;
	transform(0, 2) = 0.99944311;
	transform(0, 3) = 0.0028372451;
	transform(1, 0) = 0.99478978;
	transform(1, 1) = -0.10200293;
	transform(1, 2) = -0.00079018838;
	transform(1, 3) = -0.021259857;
	transform(2, 0) = 0.10197344;
	transform(2, 1) = 0.99422783;
	transform(2, 2) = 0.033599857;
	transform(2, 3) = 0.076272339;
	transform(3, 0) = 0;
	transform(3, 1) = 0;
	transform(3, 2) = 0;
	transform(3, 3) = 1;
	pcl::PointCloud<PointType>::Ptr vlp16_cloud(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr trans_vlp16_cloud(new pcl::PointCloud<PointType>());

	while (true) {
		if (MultipleLidarViewer::grabbing_signals[0] == 0) {
			continue;
		}
		PcapTransformLayer::get_instance()->get_current_frame(velodyne16_lidar, vlp16_cloud, 1);
		pcl::transformPointCloud(*vlp16_cloud, *trans_vlp16_cloud, transform);
		//finish
		MultipleLidarViewer::vlp16_cloud_ptr = trans_vlp16_cloud;
		//printf("vlp16: %d ", trans_vlp16_cloud->size());
		MultipleLidarViewer::finish_signals[0] = 1;
		MultipleLidarViewer::grabbing_signals[0] = 0;
	}
}

int lidar_hdl32() {
	pcap_t * velodyne32_lidar = PcapTransformLayer::get_instance()->get_pcap_dev_handle(1);
	pcl::PointCloud<PointType>::Ptr hdl32_cloud(new pcl::PointCloud<PointType>());
	while (true) {
		if (MultipleLidarViewer::grabbing_signals[1] == 0) {
			continue;
		}
		PcapTransformLayer::get_instance()->get_current_frame(velodyne32_lidar, hdl32_cloud, 0);
		//finish
		MultipleLidarViewer::hdl32_cloud_ptr = hdl32_cloud;
		printf("thread hdl32: %d\n", hdl32_cloud->size());
		MultipleLidarViewer::finish_signals[1] = 1;
		MultipleLidarViewer::grabbing_signals[1] = 0;
	}
	
}

void MultipleLidarViewer::showVelodyne16and32Points(int ethernet_num1, int ethernet_num2){
    PointViewer::get_instance()->init_point_viewer();
	memset(finish_signals, 0, 2);
	memset(grabbing_signals, 1, 2);
    pcl::PointCloud<PointType>::Ptr combined_cloud(new pcl::PointCloud<PointType>());
	std::thread t1(lidar_vlp16);
	std::thread t2(lidar_hdl32);

    while(true){
		bool all_finished = true;
		for (int i = 0; i < lidar_count; i++) {
			if (finish_signals[i] == 1) {
				continue;
			} else {
				all_finished = false;
				break;
			}
		}
		if (all_finished == false) {
			continue;
		}
		for (int i = 0; i < lidar_count; i++) {
			//ֹͣ�ɼ� �չ� ������vector
			grabbing_signals[i] = 0; 
		}
		//�ϲ�
		printf("hdl:%d\n", hdl32_cloud_ptr->size());
		PcdTransformLayer::get_instance()->combine(vlp16_cloud_ptr, hdl32_cloud_ptr, combined_cloud);
		
		//��ʾ
		PointViewer::get_instance()->set_point_cloud(combined_cloud);
		for (int i = 0; i < lidar_count; i++) {
			//��һ�ֲɼ�
			finish_signals[i] = 0;
			grabbing_signals[i] = 1;
			vlp16_cloud_ptr = nullptr;
			hdl32_cloud_ptr = nullptr;
		}
    }
}
