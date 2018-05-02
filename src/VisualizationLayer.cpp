#include "VisualizationLayer.h"

using namespace mammoth::layer;
using namespace cv;

OpencvViewer::OpencvViewer(const char * name, MyPoint2D start_location, MyPoint2D window_size) : window_name(name){
	Mat* p_picture = new Mat(window_size.y, window_size.x, CV_8UC3,Scalar(0, 0, 0));
	cvNamedWindow(name, 0);
	cvResizeWindow(name, window_size.x, window_size.y);
	cvMoveWindow(name, start_location.x, start_location.y);
	imshow(name, *p_picture);
	p_window = p_picture;
}

void OpencvViewer::clear_window(){
	uchar* pxvec = (*p_window).ptr<uchar>(0);
    int i, j;
    for (i = 0; i < (*p_window).rows; i++)
    {
        pxvec = (*p_window).ptr<uchar>(i);
        for (j = 0; j < (*p_window).cols*(*p_window).channels(); j++)
        {
            pxvec[j] = 0;
        }
    }
}

void OpencvViewer::draw_rectangle(MyBox& box, MyPoint3D rgb, int line_width, bool filled){
	IplImage tmp(*p_window);
	CvPoint points[1][4] = {CvPoint(box.point1.x, box.point1.y),
					CvPoint(box.point2.x, box.point2.y),
					CvPoint(box.point3.x, box.point3.y),
					CvPoint(box.point4.x, box.point4.y)};
	CvPoint* ppt[1] = {points[0]};
	int npt[] = {4};

	if(filled){
		cvFillPoly((CvArr*)&tmp, ppt, npt, 1, Scalar(rgb.z, rgb.y, rgb.x));

	}else{
		cvLine((CvArr*)&tmp, points[0][0], points[0][1], Scalar(rgb.z, rgb.y, rgb.x), line_width);
		cvLine((CvArr*)&tmp, points[0][1], points[0][2], Scalar(rgb.z, rgb.y, rgb.x), line_width);
		cvLine((CvArr*)&tmp, points[0][2], points[0][3], Scalar(rgb.z, rgb.y, rgb.x), line_width);
		cvLine((CvArr*)&tmp, points[0][3], points[0][0], Scalar(rgb.z, rgb.y, rgb.x), line_width);
	}

	//cvRectangle((CvArr*)&tmp, Point(box.point1.x, box.point1.y), Point(box.point3.x, box.point3.y), Scalar(rgb.z, rgb.y, rgb.x));
	imshow(window_name, *p_window);
}

void OpencvViewer::draw_circle(MyPoint2D center, int radius, MyPoint3D rgb,  int line_width, bool filled){
	IplImage tmp(*p_window);
	cvDrawCircle((CvArr*)&tmp, Point(center.x, center.y), radius, Scalar(rgb.z, rgb.y, rgb.x));
	imshow(window_name, *p_window);
}

void OpencvViewer::wait_rendering(int millseconds){
	cvWaitKey(millseconds);
}


OpencvViewerManager* OpencvViewerManager::get_instance(){
	static OpencvViewerManager manager;
	return &manager;
}

MammothViewer* OpencvViewerManager::create_viewer(){
	MammothViewer* p_viewer = new OpencvViewer("window1", MyPoint2D(0, 0), MyPoint2D(600, 480));
	return p_viewer;
}

MammothViewer* OpencvViewerManager::create_viewer(const char * name, MyPoint2D start_location, MyPoint2D window_size){
	MammothViewer* p_viewer = new OpencvViewer(name, start_location, window_size);
	std::string name_str = name;
	//viewers[name_str] = p_viewer;
	return p_viewer;
}

