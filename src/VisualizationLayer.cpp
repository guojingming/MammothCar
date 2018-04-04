#include "VisualizationLayer.h"

using namespace mammoth::layer;

// TODO : Add visualize support
void GyroscopeVisualizer::init(int argc, char ** argv, const char * title) {
	//Initialize(argc, argv, title);
}

size_t GyroscopeVisualizer::add_object(const char * path) {
	//return AddObject(path);
	return 0;
}

void GyroscopeVisualizer::rotate(size_t obj_id, float yaw, float pitch, float roll) {
	//SetRotate(obj_id, yaw, roll, pitch);
}

void GyroscopeVisualizer::translate(size_t obj_id, float x, float y, float z) {
	//SetTranslate(obj_id, x, y, z);
}