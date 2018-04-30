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
        //三通道数据都在第一行依次排列，按照BGR顺序
        //依次赋值为1
        for (j = 0; j < (*p_window).cols*(*p_window).channels(); j++)
        {
            pxvec[j] = 0;
        }
    }
}

void OpencvViewer::draw_rectangle(MyBox& box, MyPoint3D rgb){
	
}

void OpencvViewer::draw_circle(MyPoint2D center, int radius, MyPoint3D rgb){
	IplImage tmp(*p_window);
	cvDrawCircle((CvArr*)&tmp, Point(center.x, center.y), radius, Scalar(rgb.z, rgb.y, rgb.x));
	imshow(window_name, *p_window);
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

