#pragma once

#include "algorithm.h"
#include "gnssprocesser.h"

namespace mammoth {
	namespace algorithm {
		class JluSlamLayer {
		public:
			~JluSlamLayer();
			static JluSlamLayer * get_instance();
			void start_slam(const std::string& gps_folder_path, const std::string& pcd_folder_path, int start_number = 0);
		private:
			void JluSlamLayer::DoTransform(float theta1, float theta2, float trans_x, float trans_y, void* pData, size_t count);
			static JluSlamLayer * layer;
			JluSlamLayer();
		};
	}
}