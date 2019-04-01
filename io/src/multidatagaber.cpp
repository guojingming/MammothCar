#include "multidatagraber.h"

#include <time.h>

using namespace mammoth;


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

MultiDataGraber * MultiDataGraber::layer = nullptr;

MultiDataGraber::MultiDataGraber() {

}

MultiDataGraber::~MultiDataGraber() {
	if (layer != nullptr) {
		delete layer;
	}
}

MultiDataGraber* MultiDataGraber::get_instance() {
	if (layer == nullptr) {
		layer = new MultiDataGraber();
	}
	return layer;
}

std::string MultiDataGraber::gps_folder_path = "";
std::string MultiDataGraber::pcd_folder_path = "";
std::string MultiDataGraber::imu_folder_path = "";
std::string MultiDataGraber::camera_folder_path = "";

int MultiDataGraber::gps_count = 0;
int MultiDataGraber::pcd_count = 0; 
int MultiDataGraber::imu_count = 0;
int MultiDataGraber::pic_count = 0;

void MultiDataGraber::start_grab(const std::string& pcd_folder_path, const std::string& gps_folder_path, const std::string& camera_folder_path, const std::string& imu_folder_path) {
	this->gps_folder_path = gps_folder_path;
	this->pcd_folder_path = pcd_folder_path;
	this->imu_folder_path = imu_folder_path;
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

void MultiDataGraber::gps_thread() {
	if (gps_folder_path.compare("") != 0) {
		gps_count = -1;
		int gps_item_count = 0;
		char gps_path[200];
		char gps_content[100];
		UdpAttitude tcpAttitudeSolver;
		int last_pcd_count = -1;
		FileUtil file;
		while (true) {
			//memset(gps_path, 0, 200);
			//memset(gps_content, 0, 100);
			if (tcpAttitudeSolver.NoSyncCapture()) {
				/*printf("%.8f %.8f %.3f %.4f %.4f %.3f\n",
				tcpAttitudeSolver.m_package.m_longitude,
				tcpAttitudeSolver.m_package.m_latitude,
				tcpAttitudeSolver.m_package.m_elevation,
				tcpAttitudeSolver.m_package.m_yaw, 
				tcpAttitudeSolver.m_package.m_pitch,
				tcpAttitudeSolver.m_package.m_roll);*/
				sprintf(gps_content, "%d#%.2f %.8f %.8f %.3f %.4f",
					gps_item_count,
					tcpAttitudeSolver.m_package.m_avrtime,
					tcpAttitudeSolver.m_package.m_latitude,
					tcpAttitudeSolver.m_package.m_longitude,
					tcpAttitudeSolver.m_package.m_elevation,
					tcpAttitudeSolver.m_package.m_yaw
					//tcpAttitudeSolver.m_package.m_pitch,
					//tcpAttitudeSolver.m_package.m_roll
					);
				if (gps_count < pcd_count) {
					gps_count = pcd_count;
					sprintf(gps_path, "%s\\%d_gps.txt", gps_folder_path.c_str(), gps_count);
					
					file.reload_file(gps_path, 1);
					file.write_line(gps_content);
				} else {
					file.write_line(gps_content);
				}
				gps_item_count++;
			}
		}
	}
}



void MultiDataGraber::pic_thread() {
	if (camera_folder_path.compare("") != 0) {
		pic_count = 0;
		char pic_path[200];
		long long pic_start, pic_end;
		auto camera = cv::VideoCapture(0);
		camera.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
		camera.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
		camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
		cv::namedWindow("window1");
		while (true) {
			pic_start = GetCurrentTimeMsec();
			
			if (pic_count >= pcd_count) {
				continue;
			}
			cv::Mat frame;

			camera >> frame;
			memset(pic_path, 0, 200);
			sprintf(pic_path, "%s\\%d_pic.jpg", camera_folder_path.c_str(), pic_count);
			imshow("window1", frame);
			imwrite(pic_path, frame);
			pic_end = GetCurrentTimeMsec();
			cv::waitKey(1);	//延时30
			//printf("pic %d\n", pic_end - pic_start);
			pic_count++;
		}
	}
}

void MultiDataGraber::pcd_thread() {
	if (pcd_folder_path.compare("") != 0) {
		PointViewer::get_instance()->init_point_viewer();
		pcd_count = START_PCD_COUNT;
		char pcd_path[200];
		long long pcd_start, pcd_end;
		pcap_t* handle = PcapProcesser::get_instance()->get_pcap_dev_handle(1);
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
		while (true) {
			pcd_start = GetCurrentTimeMsec();
			memset(pcd_path, 0, 200);
			sprintf(pcd_path, "%s\\%d_pcd.pcd", pcd_folder_path.c_str(), pcd_count);
		
			PcapProcesser::get_instance()->get_current_frame(handle, cloud, 0);
			if (pcd_count - pic_count >= 1 && camera_folder_path.compare("") != 0) {
				continue;
			}
			if (pcd_count - gps_count >= 1 && gps_folder_path.compare("") != 0) {
				continue;
			}
			if (pcd_count - imu_count >= 1 && imu_folder_path.compare("") != 0) {
				continue;
			}
			PointViewer::get_instance()->set_point_cloud(cloud);
			PcdUtil::save_pcd_file(pcd_path, cloud);
			pcd_end = GetCurrentTimeMsec();
			//printf("pcd %d\n", pcd_end - pcd_start);
			printf("pcd: %d pic: %d gps: %d imu: %d\n", pcd_count, pic_count, gps_count, imu_count);
			pcd_count++;
		}
	}
}

void MultiDataGraber::imu_thread() {
	if (imu_folder_path.compare("")!=0){
		imu_count = -1;
		int imu_item_count = 0;
		int last_pcd_count = -1;
		char imu_path[200];
		char imu_content[100];
		char ori_imu_path[200];
		SyncCom sync_com = SerialUtil::openSync("COM4", 115200);
		char buffer[512];
		ImuSolver imuSolver;
		std::vector<unsigned char> splits;
		splits.push_back('\r');
		splits.push_back('\n');
		DataBuffer<unsigned char> dataBuffer(splits);
		std::vector<std::vector<unsigned char> > segments;
		FileUtil imu_file;
		std::ofstream * current_ori_imu_file = nullptr;
		while (1) {
			int readByte = sync_com.Read(buffer, 512);
			if (readByte > 0) {
				dataBuffer.Charge((unsigned char *)buffer, readByte);
				dataBuffer.Discharge(segments);
				for (std::vector<std::vector<unsigned char> >::iterator it = segments.begin(); it != segments.end(); it++) {
					double data[6];
					memcpy(data, &(*it)[0], 48);
					memset(imu_content, 0, 100);
					sprintf(imu_content, "%d#%.8f %.8f %.8f %.8f %.8f %.8f",
						imu_item_count,
						data[0],
						data[1],
						data[2],
						data[3],
						data[4],
						data[5]);
					//printf("%s\n", imu_content);
					if (imu_count < pcd_count) {
						imu_count = pcd_count;
						sprintf(imu_path, "%s\\%d_imu.txt", imu_folder_path.c_str(), pcd_count);

						imu_file.reload_file(imu_path, 1);
						imu_file.write_line(imu_content);
					} else {
						imu_file.write_line(imu_content);
					}
					imu_item_count++;
				}
			}
		}
	}
}