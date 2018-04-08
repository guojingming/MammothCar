#include "PersistantLayer.h"

using namespace mammoth::layer;
using namespace mammoth::config;

void MysqlPersistantLayer::connect() {

	//sql::mysql::MySQL_Driver *driver;
	//sql::Connection *con;
	//sql::Statement *state;
	//sql::ResultSet *result;
	//// 初始化驱动  
	//driver = sql::mysql::get_mysql_driver_instance();
	//// 建立链接  
	////con = driver->connect("tcp://192.168.0.243:3306", "root", "1234");
	//con = driver->connect(PersistantLayerConfig::mysql_address.c_str(), PersistantLayerConfig::mysql_user.c_str(), PersistantLayerConfig::mysql_password.c_str());
	//state = con->createStatement();
	//std::string change_database_cmd = "use " + PersistantLayerConfig::mysql_database;
	//state->execute("use lidar");
	////state->execute(change_database_cmd);
	//// 查询  
	//result = state->executeQuery("select * from test");
	//// 输出查询  
	//while (result->next()) {
	//	int id = result->getInt("test_id");
	//	std::string name = result->getString("test_value");
	//	std::cout << id << " : " << name << std::endl;
	//}
	//delete state;
	//delete con;

}

int MysqlPersistantLayer::excute_sql(std::string sql) {

	return 0;
}

void MysqlPersistantLayer::disconnect() {

}

DataGatherLayer * DataGatherLayer::layer = nullptr;

DataGatherLayer::DataGatherLayer() {

}

DataGatherLayer::~DataGatherLayer() {
	if (layer != nullptr) {
		delete layer;
	}
}

DataGatherLayer* DataGatherLayer::get_instance() {
	if (layer == nullptr) {
		layer = new DataGatherLayer();
	}
	return layer;
}

std::string DataGatherLayer::gps_folder_path = "";
std::string DataGatherLayer::pcd_folder_path = "";
std::string DataGatherLayer::imu_folder_path = "";
int DataGatherLayer::gps_count = 0;
int DataGatherLayer::pcd_count = 0; 
int DataGatherLayer::imu_count = 0;

void DataGatherLayer::start_grab(const std::string& gps_folder_path, const std::string& pcd_folder_path, const std::string& imu_folder_path) {
	this->gps_folder_path = gps_folder_path;
	this->pcd_folder_path = pcd_folder_path;
	this->imu_folder_path = imu_folder_path;
	std::thread gps_t(gps_thread);
	std::thread pcd_t(pcd_thread);
	//std::thread imu_t(imu_thread);
	system("pause");
}

void DataGatherLayer::gps_thread() {
	gps_count = 0;
	char gps_path[200];
	char gps_content[100];
	UdpAttitudeLayer tcpAttitudeSolver;
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
				tcpAttitudeSolver.m_package.m_longitude,
				tcpAttitudeSolver.m_package.m_latitude,
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

void DataGatherLayer::pcd_thread() {
	pcd_count = 0;
	char pcd_path[200];
	while (true) {
		memset(pcd_path, 0, 200);
		HPCD file;
		sprintf(pcd_path, "%s\\%d_pcd.pcd", pcd_folder_path.c_str(), pcd_count);
		PcapTransformLayer::get_instance()->get_current_frame(pcd_path, file);
		
		pcd_count++;
	}
}

void DataGatherLayer::imu_thread() {
	imu_count = 0;
	int last_pcd_count = -1;
	char imu_path[200]; 
	char imu_content[100];

	char ori_imu_path[200];

	std::string ori_imu_folder_path = "C:\\DataSpace\\LidarDataSpace\\lidar_20180322\\ori_imu_data";


	SyncCom sync_com = SerialUtil::openSync("COM5", 115200);
	char buffer[512];
	ImuSolver imuSolver;
	std::vector<unsigned char> splits;
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
				sprintf(imu_content, "%d#%d#%f %f %f %f %f %f",
					imu_count,
					gps_count,
					imuSolver.m_imuPackage.m_acclX,
					imuSolver.m_imuPackage.m_acclY,
					imuSolver.m_imuPackage.m_acclZ,
					imuSolver.m_imuPackage.m_gyroX,
					imuSolver.m_imuPackage.m_gyroY,
					imuSolver.m_imuPackage.m_gyroZ);
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
					current_ori_imu_file->write(seg.c_str(), 48);
				} else {
					current_imu_file->write_line(imu_content);

					current_ori_imu_file->write((char *)&imu_count, sizeof(imu_count));
					current_ori_imu_file->write((char *)&gps_count, sizeof(gps_count));
					current_ori_imu_file->write(seg.c_str(), 48);
				}
				imu_count++;
			}
		}
	}
	printf("Receive data finished!\n");
}