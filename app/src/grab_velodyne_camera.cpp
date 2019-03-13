#include "mammoth.h"
#include "multidatagather.h"

using namespace mammoth::io;

int main() {
	MultiDataGather::get_instance()->start_grab("", "E:\\DataSpace\\multidata2019\\pcd", "", "", "E:\\DataSpace\\multidata2019\\pic");
	return 0;
}