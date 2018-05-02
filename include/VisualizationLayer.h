#pragma once

#include "VisualizationLayerConfig.h"

using namespace mammoth::config;

namespace mammoth {
	namespace layer {

		struct MyBox{
			MyPoint2D point1;
			MyPoint2D point2;
			MyPoint2D point3;
			MyPoint2D point4;
		};

		class MammothViewer{
		public:
			virtual void wait_rendering(int millseconds) = 0;
			virtual void clear_window() = 0;
			virtual void draw_rectangle(MyBox& box, MyPoint3D rgb, int line_width = 2, bool filled = false) = 0;
			virtual void draw_circle(MyPoint2D center, int radius, MyPoint3D rgb,  int line_width = 2, bool filled = false) = 0;
		};

		class MammothViewerManager{
		public:
			virtual MammothViewer* create_viewer() = 0;
			virtual MammothViewer* create_viewer(const char * name, MyPoint2D start_location, MyPoint2D end_location) = 0;
		};

		class OpencvViewer : public MammothViewer{
		public:
			OpencvViewer(const char * name, MyPoint2D start_location, MyPoint2D window_size);
			virtual void wait_rendering(int millseconds);
			virtual void clear_window();
			virtual void draw_rectangle(MyBox& box, MyPoint3D rgb, int line_width = 2, bool filled = false);
			virtual void draw_circle(MyPoint2D center, int radius, MyPoint3D rgb,  int line_width = 2, bool filled = false);
		private:
			const char * window_name;
			cv::Mat* p_window;
		};

		class OpencvViewerManager : public MammothViewerManager{
		public:
			static OpencvViewerManager* get_instance();
			virtual MammothViewer* create_viewer();
			virtual MammothViewer* create_viewer(const char * name, MyPoint2D start_location, MyPoint2D window_size);
		private:
			OpencvViewerManager(){};
			std::map<std::string, MammothViewer*> viewers;
		};
	}
}