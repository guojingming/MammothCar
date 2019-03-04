#include "mammoth.h"

using namespace mammoth::io;
using namespace mammoth::algorithm;

int lidar_count = 2;
int * finish_signals = new int[lidar_count];
int * grabbing_signals = new int[lidar_count];

pcl::PointCloud<PointType>::Ptr vlp16_cloud_ptr = nullptr;
pcl::PointCloud<PointType>::Ptr hdl32_cloud_ptr = nullptr;

int lidar_vlp16() {
	pcap_t * velodyne16_lidar = PcapProcesser::get_instance()->get_pcap_dev_handle(5);
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
		if (grabbing_signals[0] == 0) {
			continue;
		}
		PcapProcesser::get_instance()->get_current_frame(velodyne16_lidar, vlp16_cloud, 1);
		pcl::transformPointCloud(*vlp16_cloud, *trans_vlp16_cloud, transform);
		//finish
		vlp16_cloud_ptr = trans_vlp16_cloud;
		//printf("vlp16: %d ", trans_vlp16_cloud->size());
		finish_signals[0] = 1;
		grabbing_signals[0] = 0;
	}
}

int lidar_hdl32() {
	pcap_t * velodyne32_lidar = PcapProcesser::get_instance()->get_pcap_dev_handle(1);
	pcl::PointCloud<PointType>::Ptr hdl32_cloud(new pcl::PointCloud<PointType>());
	while (true) {
		if (grabbing_signals[1] == 0) {
			continue;
		}
		PcapProcesser::get_instance()->get_current_frame(velodyne32_lidar, hdl32_cloud, 0);
		//finish
		hdl32_cloud_ptr = hdl32_cloud;
		printf("thread hdl32: %d\n", hdl32_cloud->size());
		finish_signals[1] = 1;
		grabbing_signals[1] = 0;
	}
	
}

int main(){
    PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	clock_t start_time = 0, end_time = 0;
	for (int i = 0; i < 10000; ++i) {
		start_time = clock();
		PcdUtil::read_pcd_file("E:/xbw/obj_dec_data/"+ to_string(i) +".pcd", cloud);
	//	PcdUtil::read_pcd_file("03.pcd", cloud);

		//clustering.cpp
		DimensionReductionCluster::start_clusting(cloud);
		if (cloud->size() != 0)
			PointViewer::get_instance()->set_point_cloud(cloud);
		end_time = clock();
		Sleep(50);
		//cout << to_string(end_time - start_time) + "ms" << endl;
		//cout << to_string(i) + ".pcd" << endl;
	}
	system("pause");



	/*memset(finish_signals, 0, 2);
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
			grabbing_signals[i] = 0; 
		}
		printf("hdl:%d\n", hdl32_cloud_ptr->size());
		PcdProcesser::get_instance()->combine(vlp16_cloud_ptr, hdl32_cloud_ptr, combined_cloud);
		PointViewer::get_instance()->set_point_cloud(combined_cloud);
		for (int i = 0; i < lidar_count; i++) {
			finish_signals[i] = 0;
			grabbing_signals[i] = 1;
			vlp16_cloud_ptr = nullptr;
			hdl32_cloud_ptr = nullptr;
		}
    }*/
    return 0;
}
    