#pragma once

#include "header.h"
#include "pcapprocesser.h"
#include "gnssprocesser.h"

#define START_PCD_COUNT 0

namespace mammoth {
	class MultiDataGraber {
	public:
		~MultiDataGraber();
		static MultiDataGraber * get_instance();
		void start_grab(const std::string& pcd_folder_path, const std::string& gps_folder_path, const std::string& camera_folder_path, const std::string& imu_folder_path);
	private:
		static MultiDataGraber * layer;
		MultiDataGraber();
		static std::string gps_folder_path;
		static std::string pcd_folder_path;
		static std::string imu_folder_path;
		static std::string camera_folder_path;
		static int gps_count;
		static int pcd_count;
		static int imu_count;
		static int pic_count;
		static void gps_thread();
		static void pcd_thread();
		static void imu_thread();
		static void pic_thread();
	};
}