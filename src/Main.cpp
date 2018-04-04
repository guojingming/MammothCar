#include "Mammoth.h"

using namespace mammoth::layer;
using Eigen::MatrixXd;

void cz_test();
void gjm_test();
void byz_test();

void temp_test(){
	std::vector<pcl::PointCloud<PointType>::Ptr> vec;
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	PcdUtil::read_pcd_file("E:\\DataSpace\\transpcd\\4.pcd", cloud);
	for (int i = 0; i < (*cloud).size(); i++) {
		(*cloud)[i].z *= 6;
	}
	PcdUtil::save_pcd_file("E:\\DataSpace\\transpcd\\pro_4.pcd", cloud);
}

int main(int argc, char ** argv) {
	temp_test();
	//cz_test();
	//gjm_test();
	//byz_test();

	return 0;
}

void cz_test() {
	int frame_count = 500;
	std::vector<pcl::PointCloud<PointType>::Ptr> vec;
	for (int i = 0; i < frame_count; i++) {
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
		vec.push_back(cloud);
	}  
	PcapTransformLayer::get_instance()->trans_pcap_to_pcd("E:\\DataSpace\\mark_ori_data_20180325\\9.pcap", vec, 0);
	char temp_path[100];
	for (int i = 0; i < frame_count; i++) {
		if (vec[i]->size() == 0) {
			break; 
		}
		memset(temp_path, 0, 100);
		sprintf(temp_path, "E:\\DataSpace\\transpcd9\\%d.pcd", i);
		PcdUtil::save_pcd_file(temp_path, vec[i]);
	}
	system("pause");
}

void test_extract_hall() {
	PointViewer::get_instance()->init_point_viewer();
	std::vector<pcl::PointCloud<PointType>::Ptr> clouds;
	
	char temp[100];
	int start_count = 600;
	int count = 1;
	int pcd_count = count + start_count;

	float xoz_rotation_angle = 1.8 * PI / 180;
	//1618
	for (int j = start_count; j < pcd_count; j++) {
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
		memset(temp, 0, 100);
		sprintf(temp, "E:\\DataSpace\\transpcd\\%d.pcd", j);
		PcdUtil::read_pcd_file(temp, cloud);
		pcl::PointCloud<PointType>::Ptr filted_cloud(new pcl::PointCloud<PointType>());
		for (int i = 0; i < cloud->size(); i++) {
			//XOZË³Ê±ÕëÐý×ª
			float x0 = (*cloud)[i].x;
			float z0 = (*cloud)[i].z;
			(*cloud)[i].x = x0 * cos(xoz_rotation_angle) + z0 * sin(xoz_rotation_angle);
			(*cloud)[i].z = -1 * x0 * sin(xoz_rotation_angle) + z0 * cos(xoz_rotation_angle);
			
			//¸ÄÑÕÉ«
			(*cloud)[i].r = 255;
			(*cloud)[i].b = 255;
			(*cloud)[i].g = 255;
			
			if (!(((*cloud)[i].y * (*cloud)[i].y + (*cloud)[i].x * (*cloud)[i].x <= 4) 
				//||((*cloud)[i].y >= -0.5 && (*cloud)[i].y <= 1 && (*cloud)[i].z >= -1.17 && (*cloud)[i].z <= -1.05)
				)) {
				(*filted_cloud).push_back((*cloud)[i]);
			}
		}

		//¹ýÂË
		//pcl::PassThrough<PointType> passThrough;
		//passThrough.setInputCloud(filted_cloud);
		//passThrough.setFilterLimitsNegative(true);
		//passThrough.setFilterFieldName("z");
		//passThrough.setFilterLimits(-1.2, -0.95);
		//passThrough.filter(*filted_cloud);

		/*passThrough.setInputCloud(filted_cloud);
		passThrough.setFilterLimitsNegative(false);
		passThrough.setFilterFieldName("x");
		passThrough.setFilterLimits(-50, 0);
		passThrough.filter(*filted_cloud);
*/


		for (int i = 0; i < (*filted_cloud).size(); i++) {
			(*filted_cloud)[i].z *= 2;
		}


		PointViewer::get_instance()->set_point_cloud(filted_cloud);

		PcdUtil::save_pcd_file("E:\\test.pcd", filted_cloud);
		//PcdUtil::trans_pcd_to_xyz("E:\\test.pcd", "E:\\test.xyz");

	}

	/*for (int j = start_count; j < pcd_count; j++) {
		PointViewer::get_instance()->set_point_cloud(clouds[j - start_count]);
	}*/
	system("pause");

}

void gjm_test() {
	test_extract_hall(); 

	//ÐÂµÄpcd¶ÁÈ¡º¯Êý
	/*PointViewer::get_instance()->init_point_viewer();
	PCDFILE f;
	PcdUtil::read_pcd_file("E:\\DataSpace\\LidarDataSpace\\new_lidar_20180226-1\\pcd_data\\0_17_pcd.pcd", &f);
	PointViewer::get_instance()->set_point_cloud (f);
	*/

	//¼ÇÂ¼Êý¾Ý
	DataGatherLayer::get_instance()->start_grab("E:\\DataSpace\\LidarDataSpace\\test\\gps_data","E:\\DataSpace\\LidarDataSpace\\test\\pcd_data","E:\\DataSpace\\LidarDataSpace\\test\\imu_data");

	//ï¿½ï¿½ï¿½ï¿½ï¿½ã·¨
	/*PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	PcdUtil::read_pcd_file("D:\\TTT.pcd", cloud);
	DimensionReductionCluster::start_clusting(cloud);
	printf("%d\n", cloud->size());
	PointViewer::get_instance()->set_point_cloud(cloud);
	system("pause");*/
	
	//PCD×ªXYZ
	//PcdUtil::trans_pcd_to_xyz("D:/map.pcd","D:/map.xyz");
	


	//SLAM
	JluSlamLayer::get_instance()->start_slam("E:\\DataSpace\\LidarDataSpace\\new_lidar_20180206\\gps_data","E:\\DataSpace\\LidarDataSpace\\new_lidar_20180206\\pcd_data");

	//SLAM
	//std::string gps_folder_path("E:\\DataSpace\\LidarDataSpace\\lidar_20180206\\gps_data");
	//std::string pcd_folder_path("E:\\DataSpace\\LidarDataSpace\\lidar_20180206\\pcd_data");
	std::string gps_folder_path("/home/jlurobot/YK-01a/lidar_data_20180226-1/gps_data");
	std::string pcd_folder_path("/home/jlurobot/YK-01a/lidar_data_20180226-1/pcd_data");
	JluSlamLayer::get_instance()->start_slam(gps_folder_path, pcd_folder_path);
	
	//IMUï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½
	/*FileUtil file("D:\\imudata.txt", 0);
	std::string str = "a";
	ImuLocalization l;
	float s[3] = { 0, 0, 0 };
	float v[3] = { 0, 0, 0 };
	float delta_v[3] = { 0, 0, 0 };
	while (str != "") {
		str = file.read_line();
		if (str.find_first_of("Vell") == 0) {
			str = str.substr(str.find_first_of("[") + 1);
			str = str.substr(0, str.find_first_of("]"));
			std::string delta_vx_str = str.substr(0, str.find_first_of(","));
			str = str.substr(str.find_first_of(",") + 1);
			std::string delta_vy_str = str.substr(0, str.find_first_of(","));
			str = str.substr(str.find_first_of(",") + 1);
			std::string delta_vz_str = str;
			delta_v[0] = atof(delta_vx_str.c_str());
			delta_v[1] = atof(delta_vy_str.c_str());
			delta_v[2] = atof(delta_vz_str.c_str());
			l.calculate_distance(s, v, delta_v);
			printf("%f %f %f\n", s[0], s[1], s[2]);
		}
	}*/

	//ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½
	/*PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	PcdUtil::read_pcd_file("D:\\new_map.pcd", cloud);
	PointViewer::get_instance()->set_point_cloud(cloud);
	system("pause");*/

} 

void byz_test() {
	SyncCom sync_com = SerialUtil::openSync("COM4", 460800);
	
	char buffer[512];
	ImuSolver imuSolver;
	std::vector<unsigned char> splits;
	splits.push_back('\r');
	splits.push_back('\n');
	DataBuffer<unsigned char> dataBuffer(splits);
	std::vector<std::vector<unsigned char> > segments;
	
	while (1) {
		//memset(buffer, 0, 512);
		int readByte = sync_com.Read(buffer, 512);
		//std::cout << readByte;
		// printf("readlength=%d\n",ReadByte);
		// Buffer[ReadByte]='\0';
		// SplitString(Buffer, ",")
		if (readByte > 0) {
			// std::cout << readByte;
			dataBuffer.Charge((unsigned char *)buffer, readByte);
			dataBuffer.Discharge(segments);
			//std::cout << segments.size() << std::endl;
			for (std::vector<std::vector<unsigned char> >::iterator it = segments.begin(); it != segments.end(); it++) {
				std::string seg(it->begin(), it->end());
				imuSolver.Solve(*it);
				imuSolver.m_imuPackage.Output();
				// for (std::vector<unsigned char>::iterator i = it->begin(); i != it->end(); i++)
				// {
				//   printf("0x%02x ", *i);
				// }
				// std::cout << std::endl;
				// std::cout << seg << std::endl;
				//std::cout << it->size() << std::endl;
			}
		}
		// printf("%s",buffer);
		//usleep(50000);
		// else printf("Read data failure times=%d\n",j);
	}
	printf("Receive data finished!\n");
}

