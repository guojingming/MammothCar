#include "VisualizationLayer.h"

using namespace mammoth::layer;

// void GyroscopeVisualizer::init(int argc, char ** argv, const char * title) {
// 	Initialize(argc, argv, title);
// }

// size_t GyroscopeVisualizer::add_object(const char * path) {
// 	return AddObject(path);
// }

// void GyroscopeVisualizer::rotate(size_t obj_id, float yaw, float pitch, float roll) {
// 	SetRotate(obj_id, yaw, roll, pitch);
// }

// void GyroscopeVisualizer::translate(size_t obj_id, float x, float y, float z) {
// 	SetTranslate(obj_id, x, y, z);
// }


void display(pcl::PointCloud<PointType>::Ptr cloud_16_transformed, pcl::PointCloud<PointType>::Ptr cloud_32) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("trans"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointType>(cloud_32, "cloud_32");
	pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(cloud_16_transformed, 0, 255, 0);
	viewer->addPointCloud<PointType>(cloud_16_transformed, single_color, "cloud_16_transformed");
	viewer->addCoordinateSystem(1.0);
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
}


void MultipleLidarViewer::showVelodyne16and32Points(int ethernet_num1, int ethernet_num2){
    PointViewer::get_instance()->init_point_viewer();
    pcl::PointCloud<PointType>::Ptr vlp16_cloud(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr trans_vlp16_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr hdl32_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr combined_cloud(new pcl::PointCloud<PointType>());
    pcap_t * velodyne16_lidar = PcapTransformLayer::get_instance()->get_pcap_dev_handle(5);
    pcap_t * velodyne32_lidar = PcapTransformLayer::get_instance()->get_pcap_dev_handle(1);
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
    while(true){
        PcapTransformLayer::get_instance()->get_current_frame(velodyne16_lidar, vlp16_cloud, 1);
		
        PcapTransformLayer::get_instance()->get_current_frame(velodyne32_lidar, hdl32_cloud, 0);
        
		pcl::transformPointCloud(*vlp16_cloud, *trans_vlp16_cloud, transform);
		PcdTransformLayer::get_instance()->combine(trans_vlp16_cloud, hdl32_cloud, combined_cloud);
		
		PointViewer::get_instance()->set_point_cloud(combined_cloud);
    }
}
