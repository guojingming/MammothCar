#include "mammoth.h"
#include "app.h"
#include "multidatagather.h"
 
using namespace mammoth::io;

//#1 PCD folder path can not be ""
//#2 if gnss/camera/imu path is "", the data of gnss/camera/imu will note be collected

// E:\\DataSpace\\multidata2019\\pic
// E:\\DataSpace\\multidata2019\\gnss
// E:\\DataSpace\\multidata2019\\imu

void MultidataGraberApp::run(){
	//pcd gnss pic imu
	MultiDataGather::get_instance()->start_grab("E:\\DataSpace\\multidata2019\\pcd_create", "E:\\DataSpace\\multidata2019\\gps_create", "E:\\DataSpace\\multidata2019\\pic_create", "E:\\DataSpace\\multidata2019\\imu");
}