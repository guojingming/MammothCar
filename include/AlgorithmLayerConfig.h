#pragma once

#include "UnionConfig.h"

#include "PcdUtilLayer.h"
#include "StdUtilLayer.h"
#include "PreprocessLayer.h"
#include "Eigen/Dense"  

#define _Key(x,y) ((x & 0x0000FFFF)|(y<<16))

namespace mammoth {
	namespace config {
		class AlgorithmLayerConfig {
		public:
			
		};
	}
}