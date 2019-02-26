#include "pcdprocesser.h"

using namespace mammoth::util;
using namespace mammoth::io;

PcdProcesser * PcdProcesser::layer = nullptr;

PcdProcesser::PcdProcesser(){

}

PcdProcesser::~PcdProcesser(){

}

PcdProcesser * PcdProcesser::get_instance(){
	if(layer == nullptr){
		layer = new PcdProcesser();
	}
	return layer;
}

void PcdProcesser::rotation(pcl::PointCloud<PointType>::Ptr & cloud, float x, float y, float z){
	float t_x = x * 3.1415926 / 180;
	float t_y = y * 3.1415926 / 180;
	float t_z = z * 3.1415926 / 180;
	for (int i = 0; i < cloud->size(); i++) {
		float o_x = (*cloud)[i].x;
		float o_y = (*cloud)[i].y;
		float o_z = (*cloud)[i].z;
		
		(*cloud)[i].y = o_y * cos(t_x) - o_z * sin(t_x);
		(*cloud)[i].z = o_y * sin(t_x) + o_z * cos(t_x);
		o_y = (*cloud)[i].y;
		o_z = (*cloud)[i].z;


		(*cloud)[i].x = o_x * cos(t_y) - o_z * sin(t_y);
		(*cloud)[i].z = o_x * sin(t_y) + o_z * cos(t_y);
		o_x = (*cloud)[i].x;
		o_z = (*cloud)[i].z;

		(*cloud)[i].x = o_x * cos(t_z) - o_y * sin(t_z);
		(*cloud)[i].y = o_x * sin(t_z) + o_y * cos(t_z);
		o_x = (*cloud)[i].x;
		o_y = (*cloud)[i].y;
	}
}

void PcdProcesser::translation(pcl::PointCloud<PointType>::Ptr & cloud, float x, float y, float z){
	for (int i = 0; i<cloud->size(); i++) {
		(*cloud)[i].x += x;
		(*cloud)[i].y += y;
		(*cloud)[i].z += z;
	}
}

void PcdProcesser::combine(pcl::PointCloud<PointType>::Ptr & cloud1, pcl::PointCloud<PointType>::Ptr & cloud2, pcl::PointCloud<PointType>::Ptr & combine_cloud){
	combine_cloud->clear();
	for(int i = 0;i<cloud1->size();i++){
		combine_cloud->push_back((*cloud1)[i]);
	}
	for(int i = 0;i<cloud2->size();i++){
		combine_cloud->push_back((*cloud2)[i]);
	}
}

void PcdProcesser::pcd_to_bin(std::string pcd_path, string bin_path) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcd_path, *cloud);
	std::fstream output(bin_path.c_str(), std::ios::out | std::ios::binary);
	PointViewer::get_instance()->set_point_cloud(cloud);
	float data[4] = { 50 };
	for (int i = 0; i < cloud->size(); i++) {
		data[0] = (*cloud)[i].x;
		data[1] = (*cloud)[i].y;
		data[2] = (*cloud)[i].z;
		output.write((char *)data, 4 * sizeof(float));
	}
	output.close();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PcdProcesser::bin_to_pcd(std::string &in_file) {
	std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
	if (!input.good()) {
		std::cerr << "Could not read file: " << in_file << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);
	for (int i = 0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *)&point.x, 3 * sizeof(float));
		input.read((char *)&point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
	return points;
}