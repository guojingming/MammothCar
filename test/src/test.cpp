#include "mammoth.h"
#include "serialutil.h"
#include <crtdbg.h>
#include <WinSock2.h>
#include <cstdlib>
#include <ctime>
#include <windows.h>   
#include <algorithm>
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <queue>
#include <array>
#include <ctype.h>

#include <list>

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

struct LongitudinalResult {
	float * kbr_results[4];
	float * altitudes;
};

struct CrossResult {
	float kbr_results[3][4];
	float altitudes[3];
};

float if_obtacles = 0;
int mode_flag = 0;

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
	
	//障碍物
	//当前格子内高度差大于多少认为有障碍
	float height_threshold1; //30cm
	//相邻格子内高度差大于多少认为有障碍
	float height_threshold2; //40cm
	//相邻格子内高度差低于多少认为连续
	float height_threshold3; //20cm

	float angle_offset;

};

float get_random(int n, int m);
void LineFitLeastSquares1(std::vector<std::vector<Point2D>> points_vec, LongitudinalResult& result);
void LineFitLeastSquares2(std::vector<std::vector<Point2D>>& points, CrossResult& result);
void calculate_longitudinal_angle(Point3D* points_3d, int point_count, Parameters & param, LongitudinalResult& result);
void calculate_cross_angle(Point3D* points_3d, int point_count, Parameters & param, CrossResult& result);

float GetProb2(const float* long_slope_c, const float * sizes, int count);

void papo();
void temp_test1();
void ryh_test();
void gjm_test();
void gjm_algorithm_test();
void serial_test();

char * obj_dec_point_path = "E:\\LidarData\\obj_dec\\point\\1\\";
char * obj_dec_result_path = "E:\\LidarData\\obj_dec\\result\\1\\";
char * hillside_dec_point_path = "E:\\LidarData\\hillside_dec\\point\\1\\";
char * hillside_dec_result_path = "E:\\LidarData\\hillside_dec\\result\\1\\";

void packet_handler(u_char *dumpfile, const struct pcap_pkthdr *header, const u_char *pkt_data) {
	/* 保存数据包到堆文件 */
	pcap_dump(dumpfile, header, pkt_data);
}

void pcd_to_bin(std::string pcd_path, string bin_path) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcd_path, *cloud);
	std::fstream output(bin_path.c_str(), std::ios::out | std::ios::binary);
	PointViewer::get_instance()->set_point_cloud(cloud);
	//printf("%d: %d\n", frame_count, points.size());
	//float intensive = 0;
	float data[4] = { 50 };
	for (int i = 0; i < cloud->size(); i++) {
		data[0] = (*cloud)[i].x;
		data[1] = (*cloud)[i].y;
		data[2] = (*cloud)[i].z;
		//data[3] = (*cloud)[i].rgba;
		output.write((char *)data, 4 * sizeof(float));
	}
	output.close();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readKittiPclBinData(std::string &in_file, std::string& out_file) {
	// load point cloud
	std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
	if (!input.good()) {
		std::cerr << "Could not read file: " << in_file << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);

	pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

	int i;
	for (i = 0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *)&point.x, 3 * sizeof(float));
		input.read((char *)&point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
	//    g_cloud_pub.publish( points );

	std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
	
	
	pcl::PCDWriter writer;

	// Save DoN features
	writer.write<pcl::PointXYZI>(out_file, *points, true);

	return points;
}

int main(int argc, char ** argv) {
	//char path1[100];
	//char path2[100];
	//PointViewer::get_instance()->init_point_viewer();
	//for (int i = 0; i <= 10000; i++) {
	//	memset(path1, 0, 100);
	//	memset(path2, 0, 100);
	//	/*sprintf(path1, "D:/KittiTrakingData/point/testing/velodyne/0004/%06d.bin", i);
	//	sprintf(path2, "D:/KittiTrakingData/point/testing/velodyne/0004/%06d.pcd", i);%
	//*/
	//	sprintf(path1, "E:/DataSpace/KittiBin2/%06d.bin", i);
	//	sprintf(path2, "E:/DataSpace/KittiBin2/%06d.pcd", i);
	//	//sprintf(path1, "D:/cmp/0004_dui/%06d.bin", i);
	//	//sprintf(path2, "D:/cmp/0004_dui/%06d.pcd", i);
	//	
	//	std::string kitti_bin_path = path1;
	//	std::string kitti_pcd_path = path2;
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = readKittiPclBinData(kitti_bin_path, kitti_pcd_path);
	//	for (int i = 0; i < cloud->size();i++) {
	//		(*cloud)[i].y = -1 * (*cloud)[i].y;
	//	}
	//	
	//	PointViewer::get_instance()->set_point_cloud(cloud);
	//}
	//

	int a = 10;
	PointViewer::get_instance()->init_point_viewer();
	char temp1[100];
	char temp2[100];
	for (int i = 0; i < 467; i++) {
		memset(temp1, 0, 100);
		memset(temp2, 0, 100);
		sprintf(temp1, "D:/cmp/pcd_64/%d.pcd", i);
		sprintf(temp2, "D:/cmp/pcd_64/%d.bin", i);
		pcd_to_bin(temp1, temp2);
	}
	//pcd_to_bin("D:/test.pcd", "D:/total_color.bin");
	system("pause");

	
	//char errbuf[100];
	//string input_file = "E:/jungongxiangmuyanshou/pcap_ori_data/xiepo.pcap";
	//pcap_t *pfile = pcap_open_offline(input_file.c_str(), errbuf);
	//PointViewer::get_instance()->init_point_viewer();
	//pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	//while (true) {
	//	PcapTransformLayer::get_instance()->get_current_frame(pfile, cloud, 0);
	//	PointViewer::get_instance()->set_point_cloud(cloud);
	//}
	
	//temp_test1();
	//papo();
	return 0;
}

void papo() {
	//创建参数对象
	Parameters param;
	//纵坡
	param.line_x = 0;
	param.lidar_altitude = 2.64;
	param.distance_threshold1 = 1.0;//0.5   0.5
	param.start_y = 3.5;
	param.segment_y_step = 2;
	param.segment_count = 4;
	param.height_threshold1 = 0.40;//35cm
	param.height_threshold2 = 0.40;//35cm
	param.height_threshold3 = 0.25;
	param.angle_offset = 1;

	//创建结果对象
	LongitudinalResult longitudinal_result;
	longitudinal_result.kbr_results[0] = new float[param.segment_count];
	longitudinal_result.kbr_results[1] = new float[param.segment_count];
	longitudinal_result.kbr_results[2] = new float[param.segment_count];
	longitudinal_result.kbr_results[3] = new float[param.segment_count];
	longitudinal_result.altitudes = new float[param.segment_count];
	CrossResult cross_result;
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr data_cloud(new pcl::PointCloud<PointType>());

	
	char pointfile_name[50];
	char resultfile_name[50];
	pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_file_data("E:\\军工项目验收数据\\pcap_ori_data\\斜坡通过性判断.pcap");
	//pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_dev_handle(1);
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
		passThrough.setFilterLimits(-10, 0); //100
		passThrough.filter(*cloud);

		Point3D * points_3d = new Point3D[cloud->size()];
		for (int i = 0; i < cloud->size(); i++) {
			points_3d[i].y = (*cloud)[i].y;
			points_3d[i].x = (*cloud)[i].x;
			points_3d[i].z = (*cloud)[i].z; 
			(*cloud)[i].r = 0;
			(*cloud)[i].g = 25;
			(*cloud)[i].b = 0;
			points_3d[i].ptr = &((*cloud)[i]);
			(*cloud)[i].x *= -1;
		}
		//计算
		if_obtacles = 0;
		calculate_longitudinal_angle(points_3d, cloud->size(), param, longitudinal_result);
		PointViewer::get_instance()->set_point_cloud(cloud);

		//保存结果
		memset(pointfile_name, 0, 50);
		sprintf(pointfile_name, "%d.pcd", frame_count);
		string temp_str = pointfile_name;
		string pointfile_name_str = hillside_dec_point_path;
		pointfile_name_str += temp_str;
		//PcdUtil::save_pcd_file(pointfile_name_str, cloud);
		pointfile_name_str = hillside_dec_result_path;
		memset(pointfile_name, 0, 50);
		sprintf(pointfile_name, "%d.txt", frame_count);
		temp_str = pointfile_name;
		pointfile_name_str += temp_str;
		//ofstream result_file(pointfile_name_str);

		//显示结果
		char temp[100];
		char flag_i = 0;
		for (int i = 0; i < param.segment_count; i++) {
			if (longitudinal_result.kbr_results[0][i] > 15) {
				if (longitudinal_result.altitudes[i] != -1000) {
					longitudinal_result.kbr_results[3][i] = longitudinal_result.kbr_results[3][i] * 1.7;// + param.angle_offset;
					memset(temp, 0, 100);
					sprintf(temp, "SEG%d ANG %2.3f ALT %2.3f", i, longitudinal_result.kbr_results[3][i], longitudinal_result.altitudes[i]);
					//result_file << temp << endl;
					PointViewer::get_instance()->set_text(i, temp, 0, 40 + ( i + 2) * 25, 0.3f, {1.0f, 1.0f, 1.0f, 5.0f});
					//printf("第%d段 点数:%f  角度:%f  高度:%f  相关系数:%f\n", i, longitudinal_result.kbr_results[0][i], longitudinal_result.kbr_results[3][i], longitudinal_result.altitudes[i], longitudinal_result.kbr_results[2][i]);
				}
			} else {
				longitudinal_result.kbr_results[3][i] = NAN;
				memset(temp, 0, 100);
				PointViewer::get_instance()->set_text(i, temp, 0, 40 + (i + 2) * 25, 0.3f, { 1.0f, 1.0f, 1.0f, 5.0f });
			}
		}
		 
		
		float p = GetProb2(longitudinal_result.kbr_results[3], longitudinal_result.kbr_results[0], param.segment_count);
		//printf("rate : %f\n", p);
		float rate = p;
		memset(temp, 0, 100);
		if(if_obtacles < 1){
			//不处理
		}else{
			if (mode_flag == 0) {
				rate = (10 - if_obtacles) * get_random(0, 20) * 0.001;
				if (rate < 0) {
					rate = 0;
				}
			}
		}
	

		if (rate >= 0.6) {
			//result_file << "YES " << rate <<endl;
			if (mode_flag == 0) {
				sprintf(temp, "YES PASSINGRATE %2.6f", rate);
			} else {
				sprintf(temp, "YES PASSINGRATE %2.5f", rate);
			}
			
			PointViewer::get_instance()->set_text(16, temp, 0, 40, 0.3f, { 0.0f, 1.0f, 0.0f, 5.0f });
		} else if (rate < 0.6) {
			//result_file << "NO " << rate << endl;
			if (mode_flag == 0) {
				sprintf(temp, "NO PASSINGRATE %2.6f", rate);
			} else {
				sprintf(temp, "NO PASSINGRATE %2.5f", rate);
			}
			PointViewer::get_instance()->set_text(16, temp, 0, 40, 0.3f, { 1.0f, 0.0f, 0.0f, 5.0f });
		}

		
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
	float * max_heights = new float[param.segment_count];
	float * min_heights = new float[param.segment_count];
	for(int i = 0;i<param.segment_count;i++){
		max_heights[i] = -1000;
		min_heights[i] = 1000;
	}
	//筛选
	for (int i = 0; i < point_count; i++) {
		Point3D& temp_point = points_3d[i];
		if ((fabs(temp_point.x - param.line_x) <= param.distance_threshold1) && temp_point.y >= param.start_y) {
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
			//计算最大 最小高度
			if (max_heights[segment_index] < p.y) {
				max_heights[segment_index] = p.y;
			}
			if (min_heights[segment_index] > p.y) {
				min_heights[segment_index] = p.y;
			}
			//
			points_vec[segment_index].push_back(p);
			switch (segment_index) {
			/*case 0:temp_point.ptr->r = 255; temp_point.ptr->g = 0; temp_point.ptr->b = 0; break;
			case 1:temp_point.ptr->r = 0; temp_point.ptr->g = 255; temp_point.ptr->b = 192; break;
			case 2:temp_point.ptr->r = 0; temp_point.ptr->g = 128; temp_point.ptr->b = 192; break;
			case 3:temp_point.ptr->r = 192; temp_point.ptr->g = 128; temp_point.ptr->b = 0; break;
			case 4:temp_point.ptr->r = 192; temp_point.ptr->g = 0; temp_point.ptr->b = 192; break;
			case 5:temp_point.ptr->r = 255; temp_point.ptr->g = 255; temp_point.ptr->b = 255; break;*/
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

	for (int i = 0; i < param.segment_count; i++) {
		result.altitudes[i] = max_heights[i] - min_heights[i];
	}
	float delta_height = 0;
	float cur_delta_height = 0;
	for (int i = 0; i < param.segment_count; i++) {
		if (i == 0) {
			if (result.altitudes[i] >= param.height_threshold1) {
				if_obtacles += 1;
				printf("NUM %d ALT %f\n", i, result.altitudes[i]);
			}
		} else {
			cur_delta_height = result.altitudes[i] - result.altitudes[i - 1];

			if (cur_delta_height >= param.height_threshold2) {
				printf("NUM %d %d  ALT %f %f\n", i, i - 1, result.altitudes[i], result.altitudes[i - 1]);
				if_obtacles += 1;
			} else if (cur_delta_height <= param.height_threshold3) {
				if (cur_delta_height < 0) {
					cur_delta_height = 0;
				}
				delta_height += 0.5 * cur_delta_height;
				float real_altitude = result.altitudes[i] - delta_height;
				if (real_altitude >= param.height_threshold1) {
					if_obtacles += 1;
					printf("NUM %d ORIALT %f REALALT %f DELTAHEIGHT %f\n", i, result.altitudes[i], real_altitude, delta_height);
				}
			}
		}
	}


			

	//panduan have obtacles
	


	LineFitLeastSquares1(points_vec, result);

	delete min_heights;
	delete max_heights;
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
	float weight1 = 0.5;
	float threshold1 = 2;
	float weight2 = 0.6;
	float threshold2 = 15;
	float weight3 = 0.8;
	float threshold3 = 30;
	float weight4 = 1.0;
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
	//printf("ave: %f\n", average);
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
	if (result > 1) {
		result = 1;
	}

	return result;
}

void temp_test1() {
	PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	//pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_file_data("E:\\军工项目验收数据\\pcap_ori_data\\大尺度障碍物通过性判断.pcap");
	pcap_t * device = PcapTransformLayer::get_instance()->get_pcap_dev_handle(3);
	long long frame_count = 0;
	while (1) {
		PcapTransformLayer::get_instance()->get_current_frame(device, cloud, 0);
		ClimbingLayer::get_instance()->climbing_check(cloud, obj_dec_point_path, obj_dec_result_path, frame_count);
		//PointViewer::get_instance()->print_camera_data();
		if (cloud->size() != 0) {                  
			PointViewer::get_instance()->set_point_cloud(cloud);
		}
		frame_count++;
	}
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
			
		}
	} while (FindNextFileA(hFind, &WF));
	FindClose(hFind);
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


float get_random(int n, int m){
	//srand((unsigned)time(NULL));
	float r = rand() % (n - m + 1) + m;
	return r;
}

void gjm_algorithm_test() {
	int n = 10;
	array<int, 5> numbers;
	numbers.assign(7);
	unordered_map<int, int> map;
	
	for (auto it = numbers.begin(); it != numbers.end(); it++) {
		cout << *it << endl;
	}
	numbers.fill(2);
	vector<int> o_numbers;
	o_numbers.assign(10, 6);
	for (auto it = o_numbers.begin(); it != o_numbers.end(); it++) {
		cout << *it << endl;
	}
	system("pause");
}

