/*Mat test_image = Mat::zeros(p_map->map_width, p_map->map_height, CV_8UC3);
for (int i = 0; i < p_map->map_height; i++) {
for (int j = 0; j < p_map->map_width; j++) {
std::vector<PointType> & temp_vec = p_map->get_grid(i, j);
test_image.at<cv::Vec3b>(i, j)[2] += temp_vec.size() * 6>255 ? 255 : temp_vec.size() * 6;
}
}
imshow("window1", test_image);
cv::waitKey(0);*/

//旧版本的点云叠加
//void JluSlamLayer::start_slam(const std::string& gps_folder_path, const std::string& pcd_folder_path, int start_number) {
//	PointViewer::get_instance()->init_point_viewer();
//	std::vector<std::string> file_names;
//	std::string folder_path = pcd_folder_path + "\\*.pcd";
//	FileUtil::get_all_files(folder_path, file_names);
//	//FileUtil trace_file("D:\\trace.txt", 2);
//	std::string * p_paths = new std::string[file_names.size()];
//	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<PointType>::Ptr pre_scene(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<PointType>::Ptr pre_pre_scene(new pcl::PointCloud<PointType>());
//	/*for (int i = 0; i < file_names.size(); i++) {
//	std::string number_str = file_names[i].substr(0, file_names[i].find_first_of("_"));
//	int number = atoi(number_str.c_str());
//	p_paths[number - start_number] = file_names[i];
//	}*/
//	for (int i = 0; i < file_names.size(); i++) {
//		std::string number_str = file_names[i].substr(0, file_names[i].find_first_of('_'));
//		std::string file_path = file_names[i];
//		//读文件
//		std::string read_pcd_path = pcd_folder_path + "\\" + file_path;
//		printf("%s\n", file_path.c_str());
//		char temp[100];
//		sprintf(temp, "%d_gps.txt", atoi(number_str.c_str()));
//
//		std::string gps_path = temp;
//		std::string read_gps_path = gps_folder_path + "\\" + gps_path;
//		int a = 0;
//
//		pcl::PointCloud<PointType>::Ptr point_cloud_piece(new pcl::PointCloud<PointType>());
//		pcl::io::loadPCDFile(read_pcd_path.c_str(), *point_cloud_piece);
//
//		//过滤一次
//		/*pcl::PassThrough<PointType> passThrough;
//		passThrough.setInputCloud(point_cloud_piece);
//		passThrough.setFilterLimitsNegative(false);
//		passThrough.setFilterFieldName("x");
//		passThrough.setFilterLimits(-50, 50);
//		passThrough.filter(*point_cloud_piece);
//		passThrough.setFilterFieldName("y");
//		passThrough.setFilterLimits(-50, 50);
//		passThrough.filter(*point_cloud_piece);*/
//
//		FileUtil gps_file(read_gps_path.c_str(), 0);
//		//
//		//旧版本
//		//std::string record = gps_file.read_line();
//		//新版本
//		std::string record;
//		std::string next;
//		do {
//			record = next;
//			next = gps_file.read_line();
//		} while (next.length() > 0);
//		record = record.substr(record.find_first_of("#") + 1);
//		//
//		std::string longitude_str = record.substr(0, record.find_first_of(" "));
//		record = record.substr(record.find_first_of(" ") + 1);
//		std::string latitude_str = record.substr(0, record.find_first_of(" "));
//		record = record.substr(record.find_first_of(" ") + 1);
//		record.substr(0, record.find_first_of(" "));
//		record = record.substr(record.find_first_of(" ") + 1);
//		std::string yaw_str = record.substr(0, record.find_first_of(" "));
//		GPGGA_Data gga_data;
//		gga_data.lon = atof(longitude_str.c_str());
//		gga_data.lat = atof(latitude_str.c_str());
//		PTNLAVR_Data ptnlavr_data;
//		ptnlavr_data.yaw = atof(yaw_str.c_str());
//
//
//
//		/*//旧版本 读原始报文
//		std::string gga_record = gps_file.read_line();
//		std::string hdt_record = gps_file.read_line();
//		GPGGA_Data gga_data = GnssTransformLayer::get_instance()->decodeGPGGA(gga_record);
//		PTNLAVR_Data ptnlavr_data = GnssTransformLayer::get_instance()->decodePTNLAVR(hdt_record);
//		//*/
//
//		//计算位置向量
//		double lat0 = 4349.13958348;
//		double lon0 = 12516.60912408;
//		Vec2d location_vec = GnssTransformLayer::get_instance()->get_distance1(gga_data.lat, gga_data.lon, lat0, lon0);
//
//		//printf("%f %f\n", location_vec.x, location_vec.y);
//
//		Vec2d trans_vec;
//
//
//		trans_vec.x = -1 * location_vec.x;
//		trans_vec.y = -1 * location_vec.y;
//		//计算车的方向
//		//获取到的方向角是正北逆时针角  由于GPS在车尾 方向是车头方向的左侧 所以把GPS角度-90度为车的正北逆时针方向角
//		float car_angle = (360 - ptnlavr_data.yaw) + 180;
//		//以正北为正方向 所有点云顺时针旋转 -车的正北逆时针方向角
//		//顺时针旋转公式
//		//x1 = x0 * cos(-α) + y0 * sin(-α)
//		//y1 = -x0 * sin(-α) + y0 * cos(-α
//		//float xoy_rotation_angle = -1 * (360 - car_angle) * PI / 180;
//		float xoy_fix_rotation_angle = 90 * PI / 180;
//		float xoy_rotation_angle = car_angle * PI / 180;
//
//		//printf("%f\n", car_angle);
//
//		float xoz_rotation_angle = -73 * PI / 180;
//		//更新pre_scene,pre_pre_scene
//		pre_pre_scene->clear();
//		for (int j = 0; j < pre_scene->size(); j++) {
//			pre_pre_scene->push_back((*pre_scene)[j]);
//		}
//		pre_scene->clear();
//		PointType point;
//		point.x = 0;
//		point.y = 0;
//		point.z = 0;
//		point.rgba = 0;
//		point_cloud_piece->push_back(point);
//		for (int j = 0; j < point_cloud_piece->size(); j++) {
//			pre_scene->push_back((*point_cloud_piece)[j]);
//			float x0;
//			float y0;
//			float z0;
//			//XOZ旋转
//			x0 = (*point_cloud_piece)[j].x;
//			z0 = (*point_cloud_piece)[j].z;
//			(*point_cloud_piece)[j].x = x0 * cos(xoz_rotation_angle) + z0 * sin(xoz_rotation_angle);
//			(*point_cloud_piece)[j].z = -1 * x0 * sin(xoz_rotation_angle) + z0 * cos(xoz_rotation_angle);
//
//
//			//修正误差旋转
//			/*x0 = (*point_cloud_piece)[j].x;
//			y0 = (*point_cloud_piece)[j].y;
//			(*point_cloud_piece)[j].x = x0 * cos(xoy_fix_rotation_angle) + y0 * sin(xoy_fix_rotation_angle);
//			(*point_cloud_piece)[j].y = -1 * x0 * sin(xoy_fix_rotation_angle) + y0 * cos(xoy_fix_rotation_angle);*/
//
//
//			//XOY旋转 
//			x0 = (*point_cloud_piece)[j].x;
//			y0 = (*point_cloud_piece)[j].y;
//			(*point_cloud_piece)[j].x = x0 * cos(xoy_rotation_angle) + y0 * sin(xoy_rotation_angle);
//			(*point_cloud_piece)[j].y = -1 * x0 * sin(xoy_rotation_angle) + y0 * cos(xoy_rotation_angle);
//
//			//翻折
//			(*point_cloud_piece)[j].y = -1 * (*point_cloud_piece)[j].y;
//
//
//			//平移
//			(*point_cloud_piece)[j].x += trans_vec.y;
//			(*point_cloud_piece)[j].y += -1 * trans_vec.x;
//
//			//翻折
//			(*point_cloud_piece)[j].y = -1 * (*point_cloud_piece)[j].y;
//
//			//坐标变换
//			//(*point_cloud_piece)[j].y = -1 * (*point_cloud_piece)[j].y;
//
//			/*if ((*point_cloud_piece)[j].z < -2.40) {
//			(*point_cloud_piece)[j].g = 125;
//			(*point_cloud_piece)[j].b = 0;
//			}*/
//			//printf("%f %f %f\n", (*point_cloud_piece)[j].x, (*point_cloud_piece)[j].y, (*point_cloud_piece)[j].z);
//			//叠加
//			scene->push_back((*point_cloud_piece)[j]);
//		}
//
//		//车的轨迹点
//		point = (*point_cloud_piece)[point_cloud_piece->size() - 1];
//		char trace_data[100];
//		memset(trace_data, 0, 100);
//		sprintf(trace_data, "%f %f %f %f", car_angle, point.x, point.y, point.z);
//
//		//trace_file.write_line(trace_data);
//
//		/*char save_path[100] = { 0 };
//		sprintf(save_path, "E:/DataSpace/LidarDataSpace/lidar_20171108/pro_pcds1/%d.pcd", i);
//		pcl::io::savePCDFileBinary(save_path, *scene);
//		scene->clear();*/
//
//		//printf("%d\n", scene->size());
//		//if (i % 40 == 0) {
//		//PointViewer::get_instance()->set_point_cloud(scene);
//		//system("pause");
//		//printf("序号:%d 方向:%f 位置: x%f y:%f\n", i, hdt_data.yaw, -1 * trans_vec.x, -1 * trans_vec.y);
//		//}
//		/*if (i == 3000) {
//		break;
//		}*/
//	}
//	PcdUtil::save_pcd_file("D:\\new_map.pcd", scene);
//	system("pause");
//}

//转化数据
//int last_gps = 0;
////先读PCD文件序号
//for (int i = 0; i < 4989; i++) {
//	char temp[100];
//	sprintf(temp, "C:/DataSpace/LidarDataSpace/lidar_20180226-1/pcd_data/%d_*.pcd", i);
//	std::string folder_path = temp;
//	std::vector<std::string> pcd_files;
//	FileUtil::get_all_files(folder_path, pcd_files);
//	std::string file_name = pcd_files[0];
//	file_name = file_name.substr(file_name.find_first_of("_") + 1);
//	file_name = file_name.substr(0, file_name.find_first_of("_"));
//	int current_gps = atoi(file_name.c_str());
//	//合并gps数据
//	//1#1231.1231231,1231231,12312312
//	//2#1233.2323532,1231231,43534345
//	memset(temp, 0, 100);
//	sprintf(temp, "C:/DataSpace/LidarDataSpace/new_lidar_20180226-1/gps_data/%d_gps.txt", i);
//	FileUtil combined_gps_file(temp, 2);
//	memset(temp, 0, 100);
//	sprintf(temp, "C:/DataSpace/LidarDataSpace/new_lidar_20180226-1/imu_data/%d_imu.txt", i);
//	FileUtil combined_imu_file(temp, 2);
//	memset(temp, 0, 100);
//	sprintf(temp, "C:/DataSpace/LidarDataSpace/new_lidar_20180226-1/ori_imu_data/%d_imu.txt", i);
//	std::ofstream combined_ori_imu_file(temp, std::ios::binary | std::ios::out);
//	for (int j = last_gps; j <= current_gps; j++) {
//		//合并gps数据
//		memset(temp, 0, 100);
//		sprintf(temp, "C:/DataSpace/LidarDataSpace/lidar_20180226-1/gps_data/%d_gps.txt", j);
//		folder_path = temp;
//		std::vector<std::string> gps_files;
//		FileUtil::get_all_files(folder_path, gps_files);
//		std::string gps_file_name = "C:/DataSpace/LidarDataSpace/lidar_20180226-1/gps_data/" + gps_files[0];
//		FileUtil gps_file(gps_file_name.c_str(), 0);
//		std::string content = gps_file.read_line();
//		memset(temp, 0, 100);
//		sprintf(temp, "%d#", j);
//		content = temp + content;
//		combined_gps_file.write_line(content);
//		//合并imu数据
//		memset(temp, 0, 100);
//		sprintf(temp, "C:/DataSpace/LidarDataSpace/lidar_20180226-1/imu_data/*_%d_imu.txt", j);
//		std::string imu_file_name = temp;
//		std::vector<std::string> imu_files;
//		FileUtil::get_all_files(imu_file_name, imu_files);
//		for (int k = 0; k < imu_files.size(); k++) {
//			FileUtil imu_file(("C:/DataSpace/LidarDataSpace/lidar_20180226-1/imu_data/" + imu_files[k]).c_str(), 0);
//			std::string imu_number_str = imu_files[k].substr(0, imu_files[k].find_first_of("_"));
//			int imu_number = atoi(imu_number_str.c_str());
//			imu_number_str = imu_files[k].substr(imu_files[k].find_first_of("_") + 1);
//			int gps_number = atoi(imu_number_str.substr(0, imu_number_str.find_first_of("_")).c_str());
//			memset(temp, 0, 100);
//			sprintf(temp, "%d#%d#", imu_number, gps_number);
//			content = temp + imu_file.read_line();
//			combined_imu_file.write_line(content);
//		}
//		//合并原始imu数据
//		memset(temp, 0, 100);
//		sprintf(temp, "C:/DataSpace/LidarDataSpace/lidar_20180226-1/ori_imu_data/*_%d_imu.txt", j);
//		std::string ori_imu_file_name = temp;
//		std::vector<std::string> ori_imu_files;
//		FileUtil::get_all_files(ori_imu_file_name, ori_imu_files);
//		for (int k = 0; k < ori_imu_files.size(); k++) {
//			std::ifstream ifs("C:/DataSpace/LidarDataSpace/lidar_20180226-1/ori_imu_data/" + ori_imu_files[k], std::ios::in | std::ios::binary);
//			std::string ori_imu_number_str = ori_imu_files[k].substr(0, ori_imu_files[k].find_first_of("_"));
//			int imu_number = atoi(ori_imu_number_str.c_str());
//			ori_imu_number_str = ori_imu_files[k].substr(ori_imu_files[k].find_first_of("_") + 1);
//			int gps_number = atoi(ori_imu_number_str.substr(0, ori_imu_number_str.find_first_of("_")).c_str());
//			combined_ori_imu_file.write((char *)(&imu_number), sizeof(imu_number));
//			combined_ori_imu_file.write((char *)(&gps_number), sizeof(gps_number));
//			char buffer[48];
//			ifs.read(buffer, 48);
//			combined_ori_imu_file.write(buffer, 48);
//		}
//	}
//	combined_ori_imu_file.close();
//	last_gps = current_gps + 1;
//}

//void test0() {
	//JluSlamLayer::get_instance()->start_slam("E:\\DataSpace\\LidarDataSpace\\lidar_20180101\\pro-gps-data", "E:\\DataSpace\\LidarDataSpace\\lidar_20180101\\pro-lidar-data");
	//JluSlamLayer::get_instance()->start_slam("E:\\DataSpace\\LidarDataSpace\\lidar_20180101\\bak_gps", "E:\\DataSpace\\LidarDataSpace\\lidar_20180101\\bak_lidar");
	//GnssTransformLayer::get_instance()->pre_gps_process("E:\\DataSpace\\LidarDataSpace\\lidar_20180101\\gps-data\\","E:\\DataSpace\\LidarDataSpace\\lidar_20180101\\pro-gps-data\\",0,70);
	/*PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	char * buffer = new char[100];
	for (int i = 0; i < 70; i++) {
	memset(buffer, 0, 100);
	sprintf(buffer, "D:\\Download\\%d.pcap", i + 1);
	std::string path = buffer;
	PcapTransformLayer::get_instance()->trans_pcap_to_pcd(path, 0, cloud);
	PointViewer::get_instance()->set_point_cloud(cloud);
	memset(buffer, 0, 100);
	sprintf(buffer, "E:\\DataSpace\\LidarDataSpace\\lidar_20180101\\pro-lidar-data\\%d.pcd", i + 1);
	path = buffer;
	PcdUtil::save_pcd_file(path, cloud);
	}*/
	//system("pause");
//}

//void test2() {
//	float * buffer = new float[3];
//	memset(buffer, 0, sizeof(float) * 3);
//	GyroscopeSerialInput::startSync("COM10", 9600, buffer, 3);
//	char * char_buffer = new char[50];
//	UdpAttitudeLayer udpAttitudeLayer;
//	float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
//	float millsecond = 0;
//	int packet_count = 0;
//	size_t obj1 = GyroscopeVisualizer::add_object("G.obj");
//	size_t obj2 = GyroscopeVisualizer::add_object("G.obj");
//	float yaw_diff = 0;
//	float pitch_diff = 0;
//	float roll_diff = 0;
//	bool flag = false;
//	FileUtil file("D:/log.txt",2);
//	while (true) {
//		udpAttitudeLayer.GetAttitude(yaw, pitch, roll);
//		udpAttitudeLayer.GetAttitude(millsecond);
//		memset(char_buffer, 0, sizeof(char) * 50);
//		int count = GyroscopeSerialInput::readSync(char_buffer, 50);
//		/*static std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
//		std::chrono::system_clock::time_point T = std::chrono::system_clock::now();
//		if (std::chrono::duration_cast<std::chrono::milliseconds>(T - now).count() > 1000) {
//			now = T;
//		}*/
//		GyroscopeSerialInput::read_gyroscope(char_buffer, count);
//		//printf("TIME: %f\n", millsecond);
//		//char temp_buffer[100];
//		if (packet_count >= 100) { 
//			if (flag == false) {
//				flag = true;
//				yaw_diff /= packet_count;
//				pitch_diff /= packet_count;
//				roll_diff /= packet_count;
//			}
//			float gyro_yaw = -1 * buffer[2] + yaw_diff;
//			float gyro_pitch = -1 * buffer[0] + pitch_diff;
//			float gyro_roll = buffer[1] + roll_diff;
//			/*printf("%f %f %f %f ", millsecond, gyro_yaw, gyro_pitch, gyro_roll);
//			printf("%f %f %f\n", yaw, pitch, roll);
//			memset(temp_buffer, 0, 100);
//			sprintf(temp_buffer, "%f %f %f %f %f %f %f", millsecond, gyro_yaw, gyro_pitch, gyro_roll, yaw, pitch, roll);
//			std::string str = temp_buffer;
//			file.write_line(str);
//			*/
//			GyroscopeVisualizer::translate(obj1, 0, 10, 0);
//			GyroscopeVisualizer::rotate(obj1, gyro_yaw * 3.1415926 / 180, gyro_pitch * 3.1415926 / 180, gyro_roll * 3.1415926 / 180);
//			GyroscopeVisualizer::rotate(obj2, yaw * 3.1415926 / 180, pitch * 3.1415926 / 180, roll * 3.1415926 / 180);
//		} else {
//			yaw_diff += (yaw + buffer[2]);
//			pitch_diff += (pitch + buffer[0]);
//			roll_diff += (roll - buffer[1]);
//			packet_count++;
//		}
//		//printf("\r IMU: %f %f %f ", buffer[2], buffer[0], buffer[1]);
//		//printf("GPS : %f %f %f", yaw, pitch, roll);
//		//Sleep(50);
//	}
//}
//
//void test1() {
//	float * buffer = new float[3];
//	memset(buffer, 0, sizeof(float) * 3);
//	GyroscopeSerialInput::startSync("COM8", 9600, buffer, 3);
//	char * char_buffer = new char[20];
//	auto func1 = [=](char* char_buffer) {
//		size_t obj1 = GyroscopeVisualizer::add_object("G.obj");
//		GyroscopeVisualizer::translate(obj1, 0,10, 0);
//		while (true) {
//			//system("cls");
//			memset(char_buffer, 0, sizeof(char) * 20);
//			int count = GyroscopeSerialInput::readSync(char_buffer, 20);
//			//printf("%s", char_buffer);
//			GyroscopeSerialInput::read_gyroscope(char_buffer, count);
//			
//			if (GyroscopeSerialInput::has_angle_data) {
//				//printf("IMU: %f %f %f \n", buffer[2], buffer[0], buffer[1]);
//
//				float yaw = buffer[2];
//				//float yaw = 180 - buffer[2];
//
//				GyroscopeVisualizer::rotate(obj1, yaw * 3.1415926 / 180, buffer[0] * 3.1415926 / 180, buffer[1] * 3.1415926 / 180);
//
//				GyroscopeSerialInput::has_angle_data = false;
//			}
//		}
//	};
//
//	std::thread thread1(func1, char_buffer);
//
//
//	float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
//	float millsecond = 0;
//	//bool flag = false;
//	auto func2 = [=](float& yaw, float& pitch, float& roll, float& millsecond) {
//		size_t obj2 = GyroscopeVisualizer::add_object("G.obj");
//		UdpAttitudeLayer udpAttitudeLayer;
//		int count = 0;
//		while (true) {
//			//system("cls");
//			udpAttitudeLayer.GetAttitude(yaw, pitch, roll);
//			udpAttitudeLayer.GetAttitude(millsecond);
//
//			GyroscopeVisualizer::rotate(obj2, yaw * 3.1415926 / 180, pitch * 3.1415926 / 180, roll * 3.1415926 / 180);
//			//tcpAttitudeLayer.GetAttitude(yaw, pitch, roll);
//			//if (count >= 100) {
//			/*if (flag == false) {
//			flag = true;
//			yaw_diff /= count;
//			pitch_diff /= count;
//			roll_diff /= count;
//			}*/
//			//printf("TIME: %f\n", millsecond);
//			//printf("yaw_diff: %f pitch_diff: %f roll_diff: %f\n", yaw_diff, pitch_diff, roll_diff);
//			//printf("IMU : %f %f %f\n", 180 - data[2] + yaw_diff, data[0] + pitch_diff, -1 * data[1] + roll_diff);
//			//printf("GPS : %f %f %f\n", yaw, pitch, roll);
//			//} else {
//			/*yaw_diff += (yaw - 180 + data[2]);
//			pitch_diff += (pitch - data[0]);
//			roll_diff += (roll + data[1]);*/
//			count++;
//			//}
//		}
//	};
//
//	std::thread thread2(func2, yaw, pitch, roll, millsecond);
//
//	/*while (true) {
//		printf("IMU: %f %f %f \n", buffer[0], buffer[1], buffer[2]);
//		printf("GPS : %f %f %f\n", yaw, pitch, roll);
//		Sleep(50);
//	}
//*/
//	thread1.join();
//	thread2.join();
//
//	/*
//
//	y
//	|
//	|
//	|
//	|
//	z----------x
//
//	1. pitch   和GPS一样  imu.pitch
//	2. roll    方向相反   -  imu.roll
//	3. yaw     方向相反   180  -  imu.yaw
//
//	*/
//	system("pause");
//}

