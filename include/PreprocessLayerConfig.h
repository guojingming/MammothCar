#pragma once

#include "UnionConfig.h"

#include "PcdUtilLayerConfig.h"

#ifdef WIN32
#include "pcap.h"
#include <windows.h>
#include <omp.h>
#include <direct.h>  
#include <io.h>
#endif

#include "StdUtilLayer.h"
#include "InputLayer.h"
#include "PcdUtilLayer.h"

namespace mammoth {
	namespace config {
		class PreprocessLayerConfig {
		public:
			static double * hdl32_vertical_angles;
			static uint8_t * hdl32_vertical_ids;
			static double * vlp16_vertical_angles;
			static uint8_t * vlp16_vertical_ids;
			static double * pandar40P_vertical_angles;
			static double * pandar40P_horizontal_angles;
			static double *banewakeCE30D_vertical_angles;
			// static double * rslidar16_vertical_angles;
			// static double * rslidar16_horizontal_angles;
		};

		//enum PRESET_CONFIGS {VLP16, HDL32E, PANDAR40P};
	}
}