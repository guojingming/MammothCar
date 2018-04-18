#pragma once

#include "UnionConfig.h"

#include "PcdUtilLayer.h"
#include "StdUtilLayer.h"
#include "PreprocessLayer.h"
#include "Eigen/Dense"  

#include <opencv2\highgui\highgui.hpp>

#define _Key(x,y) ((x & 0x0000FFFF)|(y<<16))

#define TRACING_SOURCE_FILE

namespace mammoth {
	namespace config {
		class AlgorithmLayerConfig {
		public:
			
		};
	}
}