#pragma once

#include "header.h"

#define CHECK_DATA_ASSOSIATION_PRECISION_ON
//#define CHECK_DATA_ASSOSIATION_PRECISION_OFF

namespace mammoth {
	class CheckAssosiationPrecision {
	//public:
		//void static check(std::map<int, ObjectState>& FrameState, std::set<int>& OnlineObjects, std::vector<std::vector<float>>& detectedObjs);
	private:
		bool static has_window_opened;
		void static DrawRecWithId(cv::Mat& img, int id, int center_x, int center_y, float px1, float py1, float px2, float py2, cv::Scalar color);
		void static DrawNewObj(cv::Mat& img, int id, int center_x, int center_y, float px1, float py1, float px2, float py2);
		void static DrawTrackedObj(cv::Mat& img, int id, int center_x, int center_y, float px1, float py1, float px2, float py2);
		void static DrawDetectedObj(cv::Mat& img, int id, int center_x, int center_y, float px1, float py1, float px2, float py2);
		void static InitCheckingWindows();
		void static ShowImage(cv::Mat& preImage, cv::Mat& curImage);
	};
}