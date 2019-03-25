#include "mammoth.h"
#include "multidatagather.h"
 
using namespace mammoth::io;

// E:\\DataSpace\\multidata2019\\pic
// E:\\DataSpace\\multidata2019\\gps
// E:\\DataSpace\\multidata2019\\imu

int main() {
	//pcd gps pic ori_imu imu
	MultiDataGather::get_instance()->start_grab("E:\\DataSpace\\multidata2019\\pcd_create", "E:\\DataSpace\\multidata2019\\gps_create", "E:\\DataSpace\\multidata2019\\pic_create", "");
	return 0; 
}   