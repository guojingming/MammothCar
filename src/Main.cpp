#include "Mammoth.h"
#include "SerialLibrary.h"
#include <crtdbg.h>
#include <WinSock2.h>
#pragma comment(lib,"ws2_32.lib")

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

void temp_test();
void temp_test1();
void ryh_test();
void gjm_test();

void serial_test();

int main(int argc, char ** argv) {

	/*std::vector<pcl::PointCloud<PointType>::Ptr> vec;
	for (int i = 0; i < 200; i++) {
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

		vec.push_back(cloud);
	}
	PcapTransformLayer::get_instance()->trans_pcap_to_pcd("D:/0180717/3.pcap", vec, 1);
	char temp[100];
	for (int i = 0; i < vec.size(); i++) {
		memset(temp, 0, 100); 
		sprintf(temp, "D:/test/%d.pcd", i);
		PcdUtil::save_pcd_file(temp, vec[i], 0);
	}*/

	temp_test1();
	//temp_test();
	//serial_test();
	return 0;
}

void temp_test1() {
	PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_file_data("D:\\0180717\\5555.pcap");
	//pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_dev_handle(5);
	long long frame_count = 0;
	while (1) {
		PcapTransformLayer::get_instance()->get_current_frame(device, cloud, 0);
		ClimbingLayer::get_instance()->climbing_check(cloud);
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
	param.distance_threshold1 = 0.5;
	param.start_y = 4;
	param.segment_y_step = 4;
	param.segment_count = 16;
	//横坡
	param.line_x1 = -1.5;
	param.line_x2 = 0;
	param.line_x3 = 1.5;
	param.distance_threshold2 = 0.5;
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
	pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_file_data("D:\\0180717\\111.pcap");
	//pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_dev_handle(5);
	PointViewer::get_instance()->init_point_viewer();
	long long frame_count = 0;
	while (1) {
		for (int i = 0; i < param.segment_count; i++) {
			longitudinal_result.altitudes[i] = -1000;
		}

		PcapTransformLayer::get_instance()->get_current_frame(device, cloud, 0);

		pcl::PassThrough<PointType> passThrough;
		passThrough.setInputCloud(cloud);
		passThrough.setFilterLimitsNegative(false);
		passThrough.setFilterFieldName("z");
		passThrough.setFilterLimits(-10, 5); //100
		passThrough.filter(*cloud);


		//PcdUtil::read_pcd_file("D:/test/83.pcd", data_cloud);
		if (frame_count % 2 != 1) {
			frame_count++;
			//continue;
		}
		Point3D * points_3d = new Point3D[cloud->size()];
		for (int i = 0; i < cloud->size(); i++) {
			/*points_3d[i].x = -1 * (*cloud)[i].y;
			points_3d[i].y = (*cloud)[i].x;
			points_3d[i].z = (*cloud)[i].z;*/
			//(*cloud)[i].y *= -1;

			points_3d[i].y = (*cloud)[i].y;
			points_3d[i].x = (*cloud)[i].x;
			points_3d[i].z = (*cloud)[i].z; 
 
			points_3d[i].ptr = &((*cloud)[i]);

			(*cloud)[i].y *= -1;
			/*points_3d[i].x = (*cloud)[i].x * cos(3.141592657 / 2) - (*cloud)[i].y * sin(3.141592657 / 2);
			points_3d[i].y = (*cloud)[i].y * cos(3.141592657 / 2) + (*cloud)[i].x * sin(3.141592657 / 2);
			points_3d[i].z = (*cloud)[i].z;
			printf("x:%f y:%f z:%f\n", points_3d[i].x, points_3d[i].y, points_3d[i].z);*/
		}
		//计算
		calculate_longitudinal_angle(points_3d, cloud->size(), param, longitudinal_result);
		PointViewer::get_instance()->set_point_cloud(cloud);

		//calculate_cross_angle(points_3d, cloud->size(), param, cross_result);
		//显示结果
		for (int i = 0; i < param.segment_count; i++) {
			if (longitudinal_result.kbr_results[0][i] > 30) {
				if (longitudinal_result.altitudes[i] != -1000) {
					printf("第%d段 点数:%f  角度:%f  高度:%f  相关系数:%f\n", i, longitudinal_result.kbr_results[0][i], longitudinal_result.kbr_results[3][i] + 5, longitudinal_result.altitudes[i], longitudinal_result.kbr_results[2][i]);
				}
			}
		}		printf("\n");
		/*printf("\n横坡：\n");
		printf("角度:%f 高度:%f\n", cross_result.kbr_results[0][3], cross_result.altitudes[0]);
		printf("角度:%f 高度:%f\n", cross_result.kbr_results[1][3], cross_result.altitudes[1]);
		printf("角度:%f 高度:%f\n", cross_result.kbr_results[2][3], cross_result.altitudes[2]);*/
		//printf("\r %f %f %f %f %f %f", longitudinal_result.kbr_results[3][0], longitudinal_result.kbr_results[3][1], longitudinal_result.kbr_results[3][2], longitudinal_result.kbr_results[3][3], longitudinal_result.kbr_results[3][4], longitudinal_result.kbr_results[3][5]);
		delete points_3d;
		cloud->clear();
		frame_count++;
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
			case 1:temp_point.ptr->r = 0; temp_point.ptr->g = 255; temp_point.ptr->b = 0; break;
			case 2:temp_point.ptr->r = 0; temp_point.ptr->g = 128; temp_point.ptr->b = 192; break;
			case 3:temp_point.ptr->r = 192; temp_point.ptr->g = 128; temp_point.ptr->b = 0; break;
			case 4:temp_point.ptr->r = 192; temp_point.ptr->g = 0; temp_point.ptr->b = 192; break;
			case 5:temp_point.ptr->r = 255; temp_point.ptr->g = 255; temp_point.ptr->b = 255; break;
			}
			

			if (segment_index != 0) {
				points_vec[segment_index - 1].push_back(p);
				/*if (result.altitudes[segment_index - 1] < p.y) {
					result.altitudes[segment_index - 1] = p.y;
				}*/
			}
			if (segment_index < param.segment_count - 1) {
				points_vec[segment_index + 1].push_back(p);
				/*if (result.altitudes[segment_index + 1] < p.y) {
					result.altitudes[segment_index + 1] = p.y;
				}*/
			}
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
		p.x = temp_point.x;
		p.y = temp_point.z + param.lidar_altitude;

		if (fabs(temp_point.x - param.line_x1) <= param.distance_threshold1 && fabs(temp_point.y - param.start_y) <= param.distance_threshold2) {
			//计算高度
			if (result.altitudes[0] < p.y) {
				result.altitudes[0] = p.y;
			}
			points_vec[0].push_back(p);
		} else if (fabs(temp_point.x - param.line_x2) <= param.distance_threshold1 && fabs(temp_point.y - param.start_y) <= param.distance_threshold2) {
			//计算高度
			if (result.altitudes[1] < p.y) {
				result.altitudes[1] = p.y;
			}
			points_vec[1].push_back(p);
		} else if (fabs(temp_point.x - param.line_x3) <= param.distance_threshold1 && fabs(temp_point.y - param.start_y) <= param.distance_threshold2) {
			//计算高度
			if (result.altitudes[2] < p.y) {
				result.altitudes[2] = p.y;
			}
			points_vec[2].push_back(p);
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
	float A = 0.0;
	float B = 0.0;
	float C = 0.0;
	float D = 0.0;
	float E = 0.0;
	float F = 0.0;

	for (int i = 0; i < points_vec.size(); i++) {
		result.kbr_results[i][0] = NAN;
		result.kbr_results[i][1] = NAN;
		result.kbr_results[i][2] = NAN;
		result.kbr_results[i][3] = NAN;


		std::vector<Point2D> points = points_vec[i];
		int count = points.size();
		for (int j = 0; j<count; j++) {
			float x = points[j].x;
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


