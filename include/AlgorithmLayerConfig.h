#pragma once

#include "GNSSFilter.h"

#include <opencv2/highgui/highgui.hpp>

#include "UnionConfig.h"

#include "PcdUtilLayer.h"
#include "StdUtilLayer.h"
#include "PreprocessLayer.h"
#include "VisualizationLayer.h"
#include "Eigen/Dense"


#define _Key(x,y) ((x & 0x0000FFFF)|(y<<16))

#define TRACING_SOURCE_FILE

namespace mammoth {
	namespace config {
		class AlgorithmLayerConfig {
		public:
			
		};
	}
}