#include "multidatagather.h"

#include <time.h>

using namespace mammoth::util;
using namespace mammoth::io;


unsigned long long GetCurrentTimeMsec() {
#ifdef _WIN32
	struct timeval tv;
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tv.tv_sec = clock;
	tv.tv_usec = wtm.wMilliseconds * 1000;
	return ((unsigned long long)tv.tv_sec * 1000 + (unsigned long long)tv.tv_usec / 1000);
#else
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((unsigned long long)tv.tv_sec * 1000 + (unsigned long long)tv.tv_usec / 1000);
#endif
}

MultiDataGather * MultiDataGather::layer = nullptr;

MultiDataGather::MultiDataGather() {

}

MultiDataGather::~MultiDataGather() {
	if (layer != nullptr) {
		delete layer;
	}
}

MultiDataGather* MultiDataGather::get_instance() {
	if (layer == nullptr) {
		layer = new MultiDataGather();
	}
	return layer;
}

std::string MultiDataGather::gps_folder_path = "";
std::string MultiDataGather::pcd_folder_path = "";
std::string MultiDataGather::imu_folder_path = "";
std::string MultiDataGather::ori_imu_folder_path = "";
std::string MultiDataGather::camera_folder_path = "";

int MultiDataGather::gps_count = 0;
int MultiDataGather::pcd_count = 0; 
int MultiDataGather::imu_count = 0;
int MultiDataGather::pic_count = 0;

void MultiDataGather::start_grab(const std::string& gps_folder_path, const std::string& pcd_folder_path, const std::string& imu_folder_path, const std::string& ori_imu_folder_path, const std::string& camera_folder_path) {
	this->gps_folder_path = gps_folder_path;
	this->pcd_folder_path = pcd_folder_path;
	this->imu_folder_path = imu_folder_path;
	this->ori_imu_folder_path = ori_imu_folder_path;
	this->camera_folder_path = camera_folder_path;

	std::thread pcd_t(pcd_thread);
	std::thread pic_t(pic_thread);
	std::thread gps_t(gps_thread);
	std::thread imu_t(imu_thread);
	pcd_t.join();
	pic_t.join();
	gps_t.join();
	imu_t.join();

}

void MultiDataGather::gps_thread() {
	if (gps_folder_path.compare("") != 0) {
		gps_count = 0;
		char gps_path[200];
		char gps_content[100];
		UdpAttitude tcpAttitudeSolver;
		int last_pcd_count = -1;
		FileUtil * current_file = nullptr;
		while (true) {
			//memset(gps_path, 0, 200);
			//memset(gps_content, 0, 100);
			if (tcpAttitudeSolver.Capture()) {
				/*printf("%.8f %.8f %.3f %.4f %.4f %.3f\n",
				tcpAttitudeSolver.m_package.m_longitude,
				tcpAttitudeSolver.m_package.m_latitude,
				tcpAttitudeSolver.m_package.m_elevation,
				tcpAttitudeSolver.m_package.m_yaw,
				tcpAttitudeSolver.m_package.m_pitch,
				tcpAttitudeSolver.m_package.m_roll);*/
				sprintf(gps_content, "%d#%.8f %.8f %.3f %.4f %.4f %.3f",
					gps_count,
					tcpAttitudeSolver.m_package.m_latitude,
					tcpAttitudeSolver.m_package.m_longitude,
					tcpAttitudeSolver.m_package.m_elevation,
					tcpAttitudeSolver.m_package.m_yaw,
					tcpAttitudeSolver.m_package.m_pitch,
					tcpAttitudeSolver.m_package.m_roll);
				if (last_pcd_count != pcd_count) {
					sprintf(gps_path, "%s\\%d_gps.txt", gps_folder_path.c_str(), pcd_count);
					FileUtil file(gps_path, 1);
					current_file = &file;
					current_file->write_line(gps_content);
				} else {
					current_file->write_line(gps_content);
				}
				gps_count++;
			}
		}
	}
}



void MultiDataGather::pic_thread() {
	if (camera_folder_path.compare("") != 0) {
		pic_count = 0;
		char pic_path[200];
		long long pic_start, pic_end;
		auto camera = cv::VideoCapture(0);
		camera.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
		camera.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
		camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
		//cv::namedWindow("window1");
		while (true) {
			pic_start = GetCurrentTimeMsec();
			cv::Mat frame;
			camera >> frame;
			if (pic_count == pcd_count) {
				continue;
			}
			memset(pic_path, 0, 200);
			sprintf(pic_path, "%s\\%d_pic.jpg", camera_folder_path.c_str(), pic_count);
			//imshow("window1", frame);
			imwrite(pic_path, frame);
			pic_end = GetCurrentTimeMsec();
			cv::waitKey(1);	//延时30
			printf("pic %d\n", pic_end - pic_start);
			pic_count++;
		}
	}
}

void MultiDataGather::pcd_thread() {
	if (pcd_folder_path.compare("") != 0) {
		pcd_count = 0;
		char pcd_path[200];
		long long pcd_start, pcd_end;
		pcap_t* handle = PcapProcesser::get_instance()->get_pcap_dev_handle(1);
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
		while (true) {
			pcd_start = GetCurrentTimeMsec();
			memset(pcd_path, 0, 200);
			sprintf(pcd_path, "%s\\%d_pcd.pcd", pcd_folder_path.c_str(), pcd_count);
		
			PcapProcesser::get_instance()->get_current_frame_withnum(handle, cloud, 0, pcd_count);
			PcdUtil::save_pcd_file(pcd_path, cloud);
			pcd_end = GetCurrentTimeMsec();
			printf("pcd %d\n", pcd_end - pcd_start);
			pcd_count++;
		}
	}
}

void MultiDataGather::imu_thread() {
	if (imu_folder_path.compare("")!=0){
		imu_count = 0;
		int last_pcd_count = -1;
		char imu_path[200];
		char imu_content[100];
		char ori_imu_path[200];
		SyncCom sync_com = SerialUtil::openSync("COM5", 115200);
		char buffer[512];
		ImuSolver imuSolver;
		std::vector<unsigned char> splits;
		splits.push_back('*');
		splits.push_back('\r');
		splits.push_back('\n');
		DataBuffer<unsigned char> dataBuffer(splits);
		std::vector<std::vector<unsigned char> > segments;

		FileUtil * current_imu_file = nullptr;
		std::ofstream * current_ori_imu_file = nullptr;
		while (1) {
			int readByte = sync_com.Read(buffer, 512);
			if (readByte > 0) {
				dataBuffer.Charge((unsigned char *)buffer, readByte);
				dataBuffer.Discharge(segments);
				for (std::vector<std::vector<unsigned char> >::iterator it = segments.begin(); it != segments.end(); it++) {
					std::string seg(it->begin(), it->end());
					imuSolver.Solve(*it);
					memset(imu_content, 0, 100);
					sprintf(imu_content, "%d#%d#%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
						imu_count,
						gps_count,
						imuSolver.m_imuPackage.m_acclX,
						imuSolver.m_imuPackage.m_acclY,
						imuSolver.m_imuPackage.m_acclZ,
						imuSolver.m_imuPackage.m_gyroX,
						imuSolver.m_imuPackage.m_gyroY,
						imuSolver.m_imuPackage.m_gyroZ,
						imuSolver.m_imuPackage.m_deltaAngleX,
						imuSolver.m_imuPackage.m_deltaAngleY,
						imuSolver.m_imuPackage.m_deltaAngleZ,
						imuSolver.m_imuPackage.m_deltaVelX,
						imuSolver.m_imuPackage.m_deltaVelY,
						imuSolver.m_imuPackage.m_deltaVelZ,
						imuSolver.m_imuPackage.m_maginX,
						imuSolver.m_imuPackage.m_maginY,
						imuSolver.m_imuPackage.m_maginZ);
					//printf("%s\n", imu_content);
					if (last_pcd_count != pcd_count) {
						last_pcd_count = pcd_count;
						sprintf(imu_path, "%s\\%d_imu.txt", imu_folder_path.c_str(), pcd_count);

						sprintf(ori_imu_path, "%s\\%d_imu.txt", ori_imu_folder_path.c_str(), pcd_count);
						//FileUtil file(imu_path, 2);
						if (current_imu_file != nullptr) {
							delete current_imu_file;
							current_imu_file = nullptr;
						}
						current_imu_file = new FileUtil(imu_path, 2);
						current_imu_file->write_line(imu_content);

						if (current_ori_imu_file != nullptr) {
							current_ori_imu_file->close();
							delete current_ori_imu_file;
							current_ori_imu_file = nullptr;
						}
						current_ori_imu_file = new std::ofstream;
						current_ori_imu_file->open(ori_imu_path, std::ios::out | std::ios::binary);

						current_ori_imu_file->write((char *)&imu_count, sizeof(imu_count));
						current_ori_imu_file->write((char *)&gps_count, sizeof(gps_count));
						current_ori_imu_file->write(seg.c_str(), 54);
					} else {
						current_imu_file->write_line(imu_content);

						current_ori_imu_file->write((char *)&imu_count, sizeof(imu_count));
						current_ori_imu_file->write((char *)&gps_count, sizeof(gps_count));
						current_ori_imu_file->write(seg.c_str(), 54);
					}
					imu_count++;
				}
			}
		}
	}
}