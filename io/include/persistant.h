#pragma once

#include "unionconfig.h"

#include "preprocess.h"
#include "stdutil.h"

namespace mammoth {
	namespace layer {
		class MysqlPersistantLayer {
		public:
			static void connect();
			static int excute_sql(std::string sql);
			static void disconnect();
		private:

		};

		class DataGatherLayer {
		public:
			~DataGatherLayer();
			static DataGatherLayer * get_instance();
			void start_grab(const std::string& gps_folder_path, const std::string& pcd_folder_path, const std::string& imu_folder_path, const std::string& ori_imu_folder_path);
		private:
			static DataGatherLayer * layer;
			DataGatherLayer();
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