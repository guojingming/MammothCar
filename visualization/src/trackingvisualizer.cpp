#include "trackingvisualizer.h"

using namespace std;
using namespace cv;
using namespace mammoth;

bool CheckAssosiationPrecision::has_window_opened = false;

// void CheckAssosiationPrecision::check(std::map<int, ObjectState>& FrameState, std::set<int>& OnlineObjects, std::vector<std::vector<float>>& detectedObjs) {
// #ifdef CHECK_DATA_ASSOSIATION_PRECISION_ON
// 	//main
// 	static Mat* p_preImage, *p_curImage;
// 	static Mat* p_blackImage;
// 	if (!has_window_opened) {
// 		has_window_opened = true;
// 		p_blackImage = new Mat(800, 800, CV_8UC3, cvScalar(0, 0, 0));
// 		p_preImage = new Mat(800, 800, CV_8UC3, cvScalar(0, 0, 0));
// 		p_curImage = new Mat(800, 800, CV_8UC3, cvScalar(0, 0, 0));
// 		InitCheckingWindows();
// 	}

// 	int center_x_off = 400;
// 	int center_y_off = 400;
// 	int x_factor = -20;
// 	int y_factor = -20;

// 	p_blackImage->copyTo(*p_preImage);
// 	p_blackImage->copyTo(*p_curImage);

// 	printf("-----------------------------------\n");

// 	for (auto it = FrameState.begin(); it != FrameState.end(); it++) {
// 		ObjectState& state = (*it).second;
// 		printf("STATE %d %f %f\n", (*it).first, state.ObbBox.CenterX, state.ObbBox.CenterY);

// 		DrawTrackedObj(*p_preImage, (*it).first, center_x_off + x_factor * state.ObbBox.CenterX, center_y_off + y_factor * state.ObbBox.CenterY, x_factor * state.ObbBox.V1X, y_factor * state.ObbBox.V1Y, x_factor * state.ObbBox.V2X, y_factor * state.ObbBox.V2Y);
// 		//DrawTrackedObj(*p_curImage, (*it).first, center_x_off + x_factor * state.ObbBox.CenterX, center_y_off + y_factor * state.ObbBox.CenterY, x_factor * state.ObbBox.V1X, y_factor * state.ObbBox.V1Y, x_factor * state.ObbBox.V2X, y_factor * state.ObbBox.V2Y);
// 	}

// 	printf("-----------------------------------\n");

// 	for (int i = 0; i < detectedObjs.size(); i++) {
// 		std::vector<float>& obj = detectedObjs[i];
// 		int id = obj[9];

// 		printf("DOBJ %d %f %f\n", id, obj[0], obj[1]);


// 		if (OnlineObjects.find(id) == OnlineObjects.end()) {
// 			DrawNewObj(*p_curImage, id, center_x_off + x_factor * obj[0], center_y_off + y_factor * obj[1], x_factor * obj[5], y_factor *  obj[6], x_factor * obj[7], y_factor * obj[8]);
// 		}
// 		else {
// 			DrawDetectedObj(*p_curImage, id, center_x_off + x_factor * obj[0], center_y_off + y_factor * obj[1], x_factor * obj[5], y_factor * obj[6], x_factor * obj[7], y_factor * obj[8]);
// 		}
// 	}

// 	//printf("-----------------------------------\n");

// 	ShowImage(*p_preImage, *p_curImage);
// 	cv::waitKey(0);
// #endif
// }

void CheckAssosiationPrecision::DrawRecWithId(Mat& img, int id, int center_x, int center_y, float px1, float py1, float px2, float py2, Scalar color) {
	//draw rec
	float long_width = fabs(px1) + fabs(px2);
	float short_width = fabs(px1) - fabs(px2);
	float long_height = fabs(py2) + fabs(py1);
	float short_height = fabs(py2) - fabs(py1);

	float p1_x = center_x - long_width / 2;
	float p2_x = center_x + long_width / 2;
	float p1_y = center_y + short_height / 2;
	float p2_y = center_y - short_height / 2;
	float p3_x = center_x - short_width / 2;
	float p4_x = center_x + short_width / 2;
	float p3_y = center_y - long_height / 2;
	float p4_y = center_y + long_height / 2;

	line(img, Point(p1_x, p1_y), Point(p3_x, p3_y), color, 2, CV_AA);
	line(img, Point(p3_x, p3_y), Point(p2_x, p2_y), color, 2, CV_AA);
	line(img, Point(p2_x, p2_y), Point(p4_x, p4_y), color, 2, CV_AA);
	line(img, Point(p4_x, p4_y), Point(p1_x, p1_y), color, 2, CV_AA);

	//draw id
	char id_str[10] = { 0 };
	sprintf(id_str, "ID:%d", id);
	putText(img, id_str, Point(center_x, center_y), cv::FONT_HERSHEY_DUPLEX, 0.5, color, 1);
}

void CheckAssosiationPrecision::DrawNewObj(Mat& img, int id, int center_x, int center_y, float px1, float py1, float px2, float py2) {
	DrawRecWithId(img, id, center_x, center_y, px1, py1, px2, py2, Scalar(0, 0, 255));
}

void CheckAssosiationPrecision::DrawTrackedObj(Mat& img, int id, int center_x, int center_y, float px1, float py1, float px2, float py2) {
	DrawRecWithId(img, id, center_x, center_y, px1, py1, px2, py2, Scalar(0, 255, 0));
}

void CheckAssosiationPrecision::DrawDetectedObj(Mat& img, int id, int center_x, int center_y, float px1, float py1, float px2, float py2) {
	DrawRecWithId(img, id, center_x, center_y, px1, py1, px2, py2, Scalar(255, 0, 0));
}

void CheckAssosiationPrecision::InitCheckingWindows() {
	cvNamedWindow("Pre-Window", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Cur-Window", CV_WINDOW_AUTOSIZE);
}

void CheckAssosiationPrecision::ShowImage(Mat& preImage, Mat& curImage) {
	imshow("Pre-Window", preImage);
	imshow("Cur-Window", curImage);
}
