#pragma once

#include "unionconfig.h"
#include "pcdutil.h"
#include "stdutil.h"
#include "gnssprocesser.h"

namespace mammoth {
	class MappingProcesser {
	public:
		~MappingProcesser();
		static MappingProcesser * get_instance();
		void start_slam(const std::string& gps_folder_path, const std::string& pcd_folder_path, int start_number = 0);
	private:
		void MappingProcesser::DoTransform(float theta1, float theta2, float trans_x, float trans_y, void* pData, size_t count);
		static MappingProcesser * layer;
		MappingProcesser();
	};
}