#include "Mammoth.h"
#include "SerialLibrary.h"
#include <crtdbg.h>
#include <WinSock2.h>
#pragma comment(lib,"ws2_32.lib")

using namespace std;
using namespace mammoth::layer;
using Eigen::MatrixXd;

struct Point3D {
	float x;
	float y;
	float z;
	PointType * ptr;
};

struct Point2D {
	float x;
	float y;
};

struct Parameters {
	//雷达安装高度
	float lidar_altitude;
	//纵坡
	//切面的x坐标
	float line_x;
	//距离切面多近的点会参与直线拟合
	float distance_threshold1;
	//从y的哪个位置开始分段、拟合
	float start_y;
	//y分段每段的长度
	float segment_y_step;
	//y的分段数量
	int segment_count;
	//横坡
	//两条条线x坐标
	float line_x1;
	float line_x2;
	float line_x3;
	//离x线多近的点
	float distance_threshold2;
};

struct LongitudinalResult {
	float * kbr_results[4];
	float * altitudes;
};

struct CrossResult {
	float kbr_results[3][4];
	float altitudes[3];
};

void LineFitLeastSquares1(std::vector<std::vector<Point2D>> points_vec, LongitudinalResult& result);
void LineFitLeastSquares2(std::vector<std::vector<Point2D>>& points, CrossResult& result);
void calculate_longitudinal_angle(Point3D* points_3d, int point_count, Parameters & param, LongitudinalResult& result);
void calculate_cross_angle(Point3D* points_3d, int point_count, Parameters & param, CrossResult& result);

float GetProb2(const float* long_slope_c, const float * sizes, int count);

void temp_test();
void temp_test1();
void ryh_test();
void gjm_test();

void serial_test();

int main(int argc, char ** argv) {
	//temp_test1();
	temp_test();
	//serial_test();

	//temp_test2();
	return 0;
}

VOID _Ck(const char* dir, vector<string>& cv) {
	WIN32_FIND_DATAA WF;
	CHAR FindDir[260];
	strcpy_s(FindDir, dir);
	strcat_s(FindDir, "\\*");
	HANDLE hFind = FindFirstFileA(FindDir, &WF);
	do {
		if (hFind == INVALID_HANDLE_VALUE)
			break;
		if (!(WF.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
			const char* ptr = WF.cFileName + strlen(WF.cFileName);
			cv.push_back(WF.cFileName);
		}
	} while (FindNextFileA(hFind, &WF));
	FindClose(hFind);
}

void temp_test1() {
	PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	//pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_file_data("D:\\0180717\\5555.pcap");
	pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_dev_handle(1);
	long long frame_count = 0;
	while (1) {
		PcapTransformLayer::get_instance()->get_current_frame(device, cloud, 0);
		ClimbingLayer::get_instance()->climbing_check(cloud);
		//PointViewer::get_instance()->print_camera_data();
		if (cloud->size() != 0) {                  
			PointViewer::get_instance()->set_point_cloud(cloud);
		}
	}
}

void temp_test() {
	//创建参数对象
	Parameters param;
	//纵坡
	param.line_x = 0;
	param.lidar_altitude = 2.64;
	param.distance_threshold1 = 1.0;//0.5   0.5
	param.start_y = 3;
	param.segment_y_step = 1.5;
	param.segment_count = 10;
	//横坡
	param.line_x1 = -1.5;
	param.line_x2 = 0;
	param.line_x3 = 1.5;
	param.distance_threshold2 = 2;//0.5    2
	//创建结果对象
	LongitudinalResult longitudinal_result;
	longitudinal_result.kbr_results[0] = new float[param.segment_count];
	longitudinal_result.kbr_results[1] = new float[param.segment_count];
	longitudinal_result.kbr_results[2] = new float[param.segment_count];
	longitudinal_result.kbr_results[3] = new float[param.segment_count];
	longitudinal_result.altitudes = new float[param.segment_count];
	CrossResult cross_result;
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	
	char name[50];
	//pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_file_data("D:\\sp2.pcap");
	pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_dev_handle(1);
	PointViewer::get_instance()->init_point_viewer();
	long long frame_count = 0;
	while (1) {
		for (int i = 0; i < param.segment_count; i++) {
			longitudinal_result.altitudes[i] = -1000;
		}

		//PcdUtil::read_pcd_file("D:/test_frame.pcd", cloud);
		PcapTransformLayer::get_instance()->get_current_frame(device, cloud, 0);

		pcl::PassThrough<PointType> passThrough;
		passThrough.setInputCloud(cloud);
		passThrough.setFilterLimitsNegative(false);
		passThrough.setFilterFieldName("z");
		passThrough.setFilterLimits(-10, 1.5); //100
		passThrough.filter(*cloud);

		if (frame_count % 2 != 1) {
			frame_count++;
			//continue;
		}
		Point3D * points_3d = new Point3D[cloud->size()];
		for (int i = 0; i < cloud->size(); i++) {
			points_3d[i].y = (*cloud)[i].y;
			points_3d[i].x = (*cloud)[i].x;
			points_3d[i].z = (*cloud)[i].z; 
			(*cloud)[i].r = 0;
			(*cloud)[i].g = 25;
			(*cloud)[i].b = 0;

			points_3d[i].ptr = &((*cloud)[i]);

			(*cloud)[i].y *= -1;
		}
		//计算
		calculate_longitudinal_angle(points_3d, cloud->size(), param, longitudinal_result);
		PointViewer::get_instance()->set_point_cloud(cloud);

		//calculate_cross_angle(points_3d, cloud->size(), param, cross_result);
		//PointViewer::get_instance()->set_point_cloud(cloud);
		//显示结果
		char temp[100];
		char flag_i = 0;
		for (int i = 0; i < param.segment_count; i++) {
			if (longitudinal_result.kbr_results[0][i] > 15) {
				if (longitudinal_result.altitudes[i] != -1000) {
					longitudinal_result.kbr_results[3][i];
					memset(temp, 0, 100);
					sprintf(temp, "SEG%d ANG %2.3f ALT %2.3f", i, longitudinal_result.kbr_results[3][i], longitudinal_result.altitudes[i]);
					PointViewer::get_instance()->set_text(i, temp, 0, 40 + ( i + 2) * 25, 0.3f, {1.0f, 1.0f, 1.0f, 5.0f});
					printf("第%d段 点数:%f  角度:%f  高度:%f  相关系数:%f\n", i, longitudinal_result.kbr_results[0][i], longitudinal_result.kbr_results[3][i], longitudinal_result.altitudes[i], longitudinal_result.kbr_results[2][i]);
				}
			} else {
				longitudinal_result.kbr_results[3][i] = NAN;
				memset(temp, 0, 100);
				PointViewer::get_instance()->set_text(i, temp, 0, 40 + (i + 2) * 25, 0.3f, { 1.0f, 1.0f, 1.0f, 5.0f });
			}
		}
		 
		
		float p = GetProb2(longitudinal_result.kbr_results[3], longitudinal_result.kbr_results[0], param.segment_count);
		printf("rate : %f\n", p);
		float rate = p;
		memset(temp, 0, 100);
		sprintf(temp, "PASSINGRATE %2.6f", rate);
		if (rate >= 0.9) {
			PointViewer::get_instance()->set_text(16, temp, 0, 40, 0.3f, { 0.0f, 1.0f, 0.0f, 5.0f });
		} else if (rate < 0.9 && rate >= 0.6) {
			PointViewer::get_instance()->set_text(16, temp, 0, 40, 0.3f, { 1.0f, 0.6f, 0.4f, 5.0f });
		} else if (rate < 0.6) {
			PointViewer::get_instance()->set_text(16, temp, 0, 40, 0.3f, { 1.0f, 0.0f, 0.0f, 5.0f });
		}
		
		//printf("通过几率:%f\n\n", rate);
		/*
		printf("\n横坡：\n");
		printf("角度:%f 高度:%f\n", cross_result.kbr_results[0][3], cross_result.altitudes[0]);
		printf("角度:%f 高度:%f\n", cross_result.kbr_results[1][3], cross_result.altitudes[1]);
		printf("角度:%f 高度:%f\n", cross_result.kbr_results[2][3], cross_result.altitudes[2]);*/
		delete points_3d;
		cloud->clear();
		frame_count++;

		//system("pause");
	}
}

void calculate_longitudinal_angle(Point3D* points_3d, int point_count, Parameters & param, LongitudinalResult& result) {
	std::vector<std::vector<Point2D>> points_vec;
	if (points_vec.size() == 0) {
		for (int i = 0; i < param.segment_count; i++) {
			std::vector<Point2D> points;
			points_vec.push_back(points);
		}
	}
	for (int i = 0; i < points_vec.size(); i++) {
		points_vec[i].clear();
	}

	//memset(result.altitudes, 0, param.segment_count * sizeof(float));

	//筛选
	for (int i = 0; i < point_count; i++) {
		Point3D& temp_point = points_3d[i];
		if (fabs(temp_point.x - param.line_x) <= param.distance_threshold1) {
			//找到相应数组中
			Point2D p;
			p.x = temp_point.y;
			p.y = temp_point.z + param.lidar_altitude;
			//printf("x:%f y:%f\n", p.x, p.y);
			float d_x = p.x - param.start_y;
			if (d_x < 0) {
				continue;
			}
			int segment_index = d_x / param.segment_y_step;
			if (segment_index >= param.segment_count) {
				continue;
			}
			//计算高度
			if (result.altitudes[segment_index] < p.y) {
				result.altitudes[segment_index] = p.y;
			}
			points_vec[segment_index].push_back(p);
			switch (segment_index) {
			case 0:temp_point.ptr->r = 255; temp_point.ptr->g = 0; temp_point.ptr->b = 0; break;
			case 1:temp_point.ptr->r = 0; temp_point.ptr->g = 255; temp_point.ptr->b = 192; break;
			case 2:temp_point.ptr->r = 0; temp_point.ptr->g = 128; temp_point.ptr->b = 192; break;
			case 3:temp_point.ptr->r = 192; temp_point.ptr->g = 128; temp_point.ptr->b = 0; break;
			case 4:temp_point.ptr->r = 192; temp_point.ptr->g = 0; temp_point.ptr->b = 192; break;
			case 5:temp_point.ptr->r = 255; temp_point.ptr->g = 255; temp_point.ptr->b = 255; break;
			default: temp_point.ptr->r = 255; temp_point.ptr->g = 0; temp_point.ptr->b = 0; break;
			}
			

			//if (segment_index != 0) {
			//	points_vec[segment_index - 1].push_back(p);
			//	/*if (result.altitudes[segment_index - 1] < p.y) {
			//		result.altitudes[segment_index - 1] = p.y;
			//	}*/
			//}
			//if (segment_index < param.segment_count - 1) {
			//	points_vec[segment_index + 1].push_back(p);
			//	/*if (result.altitudes[segment_index + 1] < p.y) {
			//		result.altitudes[segment_index + 1] = p.y;
			//	}*/
			//}
		}
	}

	LineFitLeastSquares1(points_vec, result);
}

void calculate_cross_angle(Point3D* points_3d, int point_count, Parameters & param, CrossResult& result) {
	std::vector<std::vector<Point2D>> points_vec;
	if (points_vec.size() == 0) {
		for (int i = 0; i < 3; i++) {
			std::vector<Point2D> points;
			points_vec.push_back(points);
		}
	}
	for (int i = 0; i < points_vec.size(); i++) {
		points_vec[i].clear();
	}

	//筛选
	for (int i = 0; i < point_count; i++) {
		Point3D& temp_point = points_3d[i];

		//找到相应数组中
		Point2D p;
		p.x = temp_point.y;
		p.y = temp_point.z;// +param.lidar_altitude;


		if (fabs(temp_point.y - param.line_x1) <= param.distance_threshold1
			&& fabs(temp_point.x - param.start_y) <= param.distance_threshold2
			) {
			//计算高度
			if (result.altitudes[0] < p.y) {
				result.altitudes[0] = p.y;
			}
			points_vec[0].push_back(p);
			(*temp_point.ptr).r = 255;
			(*temp_point.ptr).g = 0;
			(*temp_point.ptr).b = 0;
		} else if (fabs(temp_point.y - param.line_x2) <= param.distance_threshold1
			&& fabs(temp_point.x - param.start_y) <= param.distance_threshold2
			) {
			//计算高度
			if (result.altitudes[1] < p.y) {
				result.altitudes[1] = p.y;
			}
			points_vec[1].push_back(p);
			(*temp_point.ptr).r = 0;
			(*temp_point.ptr).g = 255;
			(*temp_point.ptr).b = 0;
		} else if (fabs(temp_point.y - param.line_x3) <= param.distance_threshold1
			&& fabs(temp_point.x - param.start_y) <= param.distance_threshold2
			) {
			//计算高度
			if (result.altitudes[2] < p.y) {
				result.altitudes[2] = p.y;
			}
			points_vec[2].push_back(p);
			(*temp_point.ptr).r = 0;
			(*temp_point.ptr).g = 0;
			(*temp_point.ptr).b = 255;
		}
	}

	LineFitLeastSquares2(points_vec, result);
}

void LineFitLeastSquares1(std::vector<std::vector<Point2D>> points_vec, LongitudinalResult& result) {
	
	for (int k = 0; k < points_vec.size(); k++) {
		float A = 0.0;
		float B = 0.0;
		float C = 0.0;
		float D = 0.0;
		float E = 0.0;
		float F = 0.0;
		result.kbr_results[0][k] = 0;
		result.kbr_results[1][k] = 0;
		result.kbr_results[2][k] = 0;
		result.kbr_results[3][k] = 0;
		std::vector<Point2D> points = points_vec[k];
		if (points.size() == 0) {
			continue;
		}
		int count = points.size();
		for (int i = 0; i<count; i++) {
			float x = points[i].x;
			float y = points[i].y;
			A += x * x;
			B += x;
			C += x * y;
			D += y;
		}
		// 计算斜率a和截距b
		float a, b, temp = 0;
		if (temp = (count*A - B*B))// 判断分母不为0
		{
			a = (count*C - B*D) / temp;
			b = (A*D - B*C) / temp;
		} else {
			a = 1;
			b = 0;
		}
		// 计算相关系数r
		float Xmean, Ymean;
		Xmean = B / count;
		Ymean = D / count;
		float tempSumXX = 0.0, tempSumYY = 0.0;
		for (int i = 0; i<count; i++) {
			float x = points[i].x;
			float y = points[i].y;
			tempSumXX += (x - Xmean) * (x - Xmean);
			tempSumYY += (y - Ymean) * (y - Ymean);
			E += (x - Xmean) * (y - Ymean);
		}
		F = sqrt(tempSumXX) * sqrt(tempSumYY);

		float r;
		r = E / F;
		result.kbr_results[0][k] = points.size();
		result.kbr_results[1][k] = b;
		result.kbr_results[2][k] = r*r;
		result.kbr_results[3][k] = atan(a) * 180 / 3.141592657;
	}
}

void LineFitLeastSquares2(std::vector<std::vector<Point2D>>& points_vec, CrossResult& result) {
	

	for (int i = 0; i < points_vec.size(); i++) {
		result.kbr_results[i][0] = NAN;
		result.kbr_results[i][1] = NAN;
		result.kbr_results[i][2] = NAN;
		result.kbr_results[i][3] = NAN;
		float A = 0.0;
		float B = 0.0;
		float C = 0.0;
		float D = 0.0;
		float E = 0.0;
		float F = 0.0;

		std::vector<Point2D> points = points_vec[i];
		int count = points.size();
		for (int j = 0; j<count; j++) {


			//-0.37618   3.8631
			//- 0.30825   3.8878
			//- 0.24023   3.9126
			//- 0.17212   3.9374
			//- 0.10393   3.9622
			//- 0.035638  3.987
			//0.032743   4.0119
			//0.10121    4.0368
			//0.16977    4.0618
			//0.23842    4.0868
			//0.30715    4.1118
			//0.37598    4.1368
			//0.44489    4.1619 

			float x = -points[j].x;
			float y = points[j].y;
			A += x * x;
			B += x;
			C += x * y;
			D += y;
		}
		// 计算斜率a和截距b
		float a, b, temp = 0;
		if (temp = (count*A - B*B))// 判断分母不为0
		{
			a = (count*C - B*D) / temp;
			b = (A*D - B*C) / temp;
		} else {
			a = 1;
			b = 0;
		}
		// 计算相关系数r
		float Xmean, Ymean;
		Xmean = B / count;
		Ymean = D / count;
		float tempSumXX = 0.0, tempSumYY = 0.0;
		for (int j = 0; j<count; j++) {
			float x = points[j].x;
			float y = points[j].y;
			tempSumXX += (x - Xmean) * (x - Xmean);
			tempSumYY += (y - Ymean) * (y - Ymean);
			E += (x - Xmean) * (y - Ymean);
		}
		F = sqrt(tempSumXX) * sqrt(tempSumYY);

		float r;
		r = E / F;
		result.kbr_results[i][0] = a;
		result.kbr_results[i][1] = b;
		result.kbr_results[i][2] = r*r;
		result.kbr_results[i][3] = atan(a) * 180 / 3.141592657;

	}

}

void gjm_test() {
	//MultipleLidarViewer::showVelodyne16and32Points(0,1);
	//�µ�pcd��ȡ����
	//PointViewer::get_instance()->init_point_viewer();
	/*PCDFILE f;
	PcdUtil::read_pcd_file("C:\\DataSpace\\map\\0406-1.pcd", &f);
	PointViewer::get_instance()->set_point_cloud(f);
	system("pause");*/
	//��¼����
	//PointViewer::get_instance()->init_point_viewer();
	//DataGatherLayer::get_instance()->start_grab("E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\gps_data", "E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\pcd_data", "E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\imu_data", "E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\ori_imu_data");

	//�����㷨
	PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

	//PcdUtil::read_pcd_file("D:\\t3.pcd", cloud);
	pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_file_data("D:\\0180717\\7.pcap");
	while (1) {
		PcapTransformLayer::get_instance()->get_current_frame(device, cloud, 0);
		PointViewer::get_instance()->set_point_cloud(cloud);
	}


	//DimensionReductionCluster::start_clusting(cloud);
	//printf("%d\n", cloud->size());

	system("pause");
	/*
	pcl::fromPCLPointCloud2();*/
	//PCDתXYZ
	//PcdUtil::trans_pcd_to_xyz("D:/map.pcd","D:/map.xyz");


	//SLAM
	//JluSlamLayer::get_instance()->start_slam("E:\\DataSpace\\LidarDataSpace\\new_lidar_20180226-1\\gps_data","E:\\DataSpace\\LidarDataSpace\\new_lidar_20180226-1\\pcd_data");
}

void ryh_test() {
	PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_dev_handle(4);
	WSADATA wsaData;
	WSAStartup(WINSOCK_VERSION, &wsaData);
	SOCKET sock;
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	char buffer[50] = "version";
	char dest_ip[] = "192.168.1.80";
	unsigned short dest_port = 2368;
	sockaddr_in RemoteAddr;
	RemoteAddr.sin_family = AF_INET;
	RemoteAddr.sin_port = htons(dest_port);
	RemoteAddr.sin_addr.S_un.S_addr = inet_addr(dest_ip);
	sendto(sock, buffer, 50, 0, (sockaddr*)&RemoteAddr, sizeof(RemoteAddr));
	strcpy_s(buffer, "getDistanceAndAmplitudeSorted");
	sendto(sock, buffer, 50, 0, (sockaddr*)&RemoteAddr, sizeof(RemoteAddr));
	cv::namedWindow("grid");
	cv::Mat img = cv::Mat(21, 118, CV_8UC3, cv::Scalar(0, 0, 0));
	while (1) {
		PcapTransformLayer::get_instance()->get_current_frame_CE30D(img, device, cloud, 0);
		PointViewer::get_instance()->set_point_cloud(cloud);
	}
	closesocket(sock);
	//	system("pause");
}

void serial_test() {
	SerialPortInfo Com1info;
	SyncCom com1;
	Com1info.name = "COM7";
	Com1info.baudRate = 115200;
	Com1info.parity = NOPARITY;
	Com1info.dataBits = 8;
	Com1info.stopBits = 1;
	com1.Connect(Com1info);
	char data[16];
	memset(data, 0, 16);

	data[1] = 0;
	while (true) {

		data[0] = 0xAA;
		data[1] ++;
		data[2] = 0x01;
		data[3] = 0x00;
		data[4] = 0x00;
		data[5] = 0x14;
		data[6] = 0x00;
		data[7] = 0x00;
		data[8] = 0x00;
		data[9] = 0xDD;
		data[10] = 0x00;
		data[13] = 0x00;
		data[14] = 0x00;
		for (int i = 0; i <= 13; i++) {
			data[14] += data[i];
		}
		data[15] = 0x55;
		com1.SendData(data, 16);
		printf("Send: ");
		for (int i = 0; i < 16; i++) {
			printf("%d ", data[i]);
		}
		printf("\n");
		Sleep(30);
	}
}


float phi(float x) {
	// constants
	float a1 = 0.254829592f;
	float a2 = -0.284496736f;
	float a3 = 1.421413741f;
	float a4 = -1.453152027f;
	float a5 = 1.061405429f;
	float p = 0.3275911f;

	// Save the sign of x
	int sign = 1;
	if (x < 0)
		sign = -1;
	x = fabsf(x) / sqrtf(2.0f);

	// A&S formula 7.1.26
	float t = 1.0f / (1.0f + p * x);
	float y = 1.0f - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*expf(-x * x);

	return 0.5*(1.0 + sign * y);
}

template<typename _Cb>
float GetMean(const float* values, int n, _Cb cb = [](float x) {return x; }) {
	float sum = 0.0f;
	for (int i = 0; i < n; i++) {
		sum += cb(values[i]);
	}
	return sum / n;
}

template<typename _Cb>
float GetSqStd(const float* values, float mean, int n, _Cb cb = [](float x) {return x; }) {
	float sum = 0.0f;
	for (int i = 0; i < n; i++) {
		float C = cb(values[i]);
		sum += (C - mean) * (C - mean);
	}
	return sum / n;
}

float GetProb2(const float* long_slope_c, const float * sizes, int count) {
	//const float long_friction = 0.6f;
	////Commons
	//const float i0 = 2.65f;
	//const float ig1 = 4.596f;
	//const float n = 0.9f;
	//const float Temax = 619.7f;
	//const float m = 1820.0f + 100.0f + 100.0f;
	//const float g = 9.8f;
	//const float r = 0.353f;
	//const float zero_threshold = 1.0f;
	//const float b = 1.675f;
	//const float hg = 0.59f;
	//int outmode = 0;
	//float theta_vali = 0.0f;
	//float theta1[15];
	//int nt = 0;
	//printf("\n\n");
	//for (int i = 0; i < 15; i++) {
	//	if (sizes[i] < 15) {
	//		continue;
	//	}
	//	printf("%f\n", long_slope_c[i]);
	//	if (!isnan(long_slope_c[i])) {
	//		outmode |= 0x01;
	//		if (fabsf(long_slope_c[i]) > zero_threshold) {
	//			outmode |= 0x02;
	//			theta1[nt++] = long_slope_c[i] / 180.0f*3.141592f;
	//		}
	//	}

	//}
	//switch (outmode) {
	//case 0:
	//	break;
	//case 2:
	//	//All NaN
	//	return NAN;
	//case 1:
	//	return 1.0f;
	//}

	//float SMEAN = GetMean(theta1, nt, sinf);
	//float SSTDSQ = GetSqStd(theta1, SMEAN, nt, sinf);
	//float CMEAN = GetMean(theta1, nt, cosf);
	//float CSTDSQ = GetSqStd(theta1, CMEAN, nt, cosf);

	//float P1, P2, P3;
	////Condition1
	//{
	//	float E = SMEAN * g;
	//	float D = SSTDSQ * g * g;
	//	float Fmax = Temax * ig1*i0*n / r;
	//	float X = (Fmax / m - E) / sqrtf(D);
	//	P1 = phi(X);
	//}

	////Condition2
	//{
	//	float c1 = long_friction * CMEAN - SMEAN;
	//	float c2 = (long_friction * long_friction) * CSTDSQ + SSTDSQ;
	//	float x = c1 / sqrtf(c2);
	//	// printf("%.2f //// ", c2);
	//	P2 = phi(x);
	//}

	////condition3
	//{
	//	float E = b * CMEAN - hg * SMEAN;
	//	float D = (b * b)*CSTDSQ + (hg * hg)*SSTDSQ;
	//	float X = E / sqrt(D);

	//	P3 = phi(X);
	//}
	////printf("%.2f,%.2f,%.2f ///// ", P1, P2, P3);
	//return P1 * P2 * P3;
	float weight1 = 0;
	float threshold1 = 5;
	float weight2 = 1;
	float threshold2 = 15;
	float weight3 = 2;
	float threshold3 = 30;
	float weight4 = 2.7;
	int real_count = 0;
	float sum = 0;
	float average = 0;
	float result = 0;
	for (int i = 0; i < count; i++) {

		float angle = long_slope_c[i];
		if (angle == NAN) {
			continue;
		} else if (sizes[i] < 15) {
			continue;
		} else {
			if (angle <= threshold1 && angle >= 0) {
				sum += weight1 * angle;
			} else if (angle >= threshold1 && angle < threshold2) {
				sum += weight2 * angle;
			} else if (angle >= threshold2 && angle < threshold3) {
				sum += weight3 * angle;
			} else if (angle >= threshold3) {
				sum += weight4 * angle;
			}
			real_count++;
		}
	}
	average = sum / real_count;
	printf("ave: %f\n", average);
	if (average >= 0 && average < threshold1) {
		result = 1;
	} else if (average >= threshold1 && average < threshold2) {
		result = 0.8 + (threshold2 - average) / (threshold2 - threshold1) / 5;
	} else if (average >= threshold2 && average < threshold3) {
		result = 0.8 - (average - threshold2) / (threshold3 - threshold2) / 2;
	} else {
		result = 0.3 - (average - threshold3) / 100;
	}
	if (result < 0) {
		result = 0;
	}

	return result;
}


