#pragma once

#include "VisualizationLayerConfig.h"

namespace mammoth {
	namespace layer {
		class GyroscopeVisualizer {
		public:
			static void init(int argc, char ** argv, const char * title);
			static size_t add_object(const char * path); 
			static void rotate(size_t obj_id, float yaw, float pitch, float roll);
			static void translate(size_t obj_id, float x, float y, float z);
		};
	}
}