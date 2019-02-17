#pragma once

#include "unionconfig.h"

#include "pcapprocesser.h"
#include "gnssprocesser.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include "stdutil.h"

namespace mammoth {
	namespace io {
		class MultiDataGather {
		public:
			~MultiDataGather();
			static MultiDataGather * get_instance();
			void start_grab(const std::string& gps_folder_path, const std::string& pcd_folder_path, const std::string& imu_folder_path, const std::string& ori_imu_folder_path);
		private:
			static MultiDataGather * layer;
			MultiDataGather();
			static std::string gps_folder_path;
			static std::string pcd_folder_path;
			static std::string imu_folder_path;
			static std::string ori_imu_folder_path;
			static int gps_count;
			static int pcd_count;
			static int imu_count;
			static void gps_thread();
			static void pcd_thread();
			static void imu_thread();
		};
	}
}