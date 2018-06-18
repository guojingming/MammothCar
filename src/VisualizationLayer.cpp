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

void MultipleLidarViewer::showVelodyne16and32Points(int ethernet_num1, int ethernet_num2){
    PointViewer::get_instance()->init_point_viewer();
    pcl::PointCloud<PointType>::Ptr vlp16_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr hdl32_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr combined_cloud(new pcl::PointCloud<PointType>());
    pcap_t * velodyne16_lidar = PcapTransformLayer::get_instance()->get_pcap_dev_handle(0);
    pcap_t * velodyne32_lidar = PcapTransformLayer::get_instance()->get_pcap_dev_handle(1);
    
    while(true){
        vlp16_cloud->clear();
        hdl32_cloud->clear();
        combined_cloud->clear();
        PcapTransformLayer::get_instance()->get_current_frame(velodyne16_lidar, vlp16_cloud, 0);
        PcapTransformLayer::get_instance()->get_current_frame(velodyne32_lidar, hdl32_cloud, 1);
        PcdTransformLayer::get_instance()->rotation(vlp16_cloud, 0, 0, 0);
        PcdTransformLayer::get_instance()->translation(vlp16_cloud, 0, 0, 0);
        PcdTransformLayer::get_instance()->combine(vlp16_cloud, hdl32_cloud, combined_cloud);
        PointViewer::get_instance()->set_point_cloud(combined_cloud);
    }
}