#pragma once

#include "unionconfig.h"
#include "preprocess.h"
#include "pcdutil.h"
#include <pcl/common/transforms.h>
#include<pcl/visualization/pcl_visualizer.h>

namespace mammoth {
	namespace layer {
		class GyroscopeVisualizer {
		public:
			// static void init(int argc, char ** argv, const char * title);
			// static size_t add_object(const char * path); 
			// static void rotate(size_t obj_id, float yaw, float pitch, float roll);
			// static void translate(size_t obj_id, float x, float y, float z);
		};

		class MultipleLidarViewer{
		public:
			static void showVelodyne16and32Points(int ethernet_num1, int ethernet_num2);
			static int * finish_signals;
			static int * grabbing_signals;
			static int lidar_count;
			//static std::vector<pcl::PointCloud<PointType>::Ptr> point_clouds;
			static pcl::PointCloud<PointType>::Ptr vlp16_cloud_ptr;
			static pcl::PointCloud<PointType>::Ptr hdl32_cloud_ptr;
		private:
			//int lidar_vlp16();
			//int lidar_hdl32();
		};
	}

}