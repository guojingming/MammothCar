#include "clustering.h"
#define ZONE_SIZE 10000//区域大小，单位cm
#define GRID_SIZE 10//栅格大小，单位cm
#define THRESHOLD 0.02//栅格z坐标方差阈值
#define GROUD_DEV -2//地面点z坐标阈值
using namespace mammoth::util;
using namespace mammoth::algorithm;

std::vector<uint32_t> DimensionReductionCluster::cube_handles;

float z_min = 1000;

void DimensionReductionCluster::start_clusting(pcl::PointCloud<PointType>::Ptr & cloud) {
	//filtering
	pcl::PassThrough<PointType> passThrough;
	passThrough.setInputCloud(cloud);
	passThrough.setFilterLimitsNegative(false);
	passThrough.setFilterFieldName("x");
	passThrough.setFilterLimits(-15, 15); //100
	passThrough.filter(*cloud);
	passThrough.setFilterFieldName("y");
	passThrough.setFilterLimits(-15, 15); //100
	passThrough.filter(*cloud);
	//pcl::io::savePCDFileBinary("test_pcd.pcd", *cloud);
	for (int i = 0; i < cloud->size(); ++i) {//简易滤除地面
		if (z_min >= cloud->points[i].z)
			z_min = cloud->points[i].z;
		/*if (z_max <= cloud->points[i].z)
			z_max = cloud->points[i].z;*/
	}
	//passThrough.setFilterFieldName("z");
	//passThrough.setFilterLimits(z_min + 0.6, z_max);
	//passThrough.filter(*cloud);

	PointViewer::get_instance()->remove_cubes(cube_handles);
	cloud = birdview_picture_grid(cloud);
}

pcl::PointCloud<PointType>::Ptr DimensionReductionCluster::birdview_picture_grid(pcl::PointCloud<PointType>::Ptr& cloud) {
	//TIME_FUNC; 
	//std::cout << " Start dimension reduction" << std::endl;
	int pic_size = ZONE_SIZE / GRID_SIZE;//原始值20000，代表栅格行列数 
	int rows = pic_size;
	int cols = pic_size;
	//int min_size = 0;
	//bool fill_pixel_flag = true;
	cv::Mat black_picture(rows, cols, CV_8UC1, cv::Scalar(0));
	srand((unsigned)time(NULL));
	std::map<int, std::vector<PointType>> map2Dto3D;
	int mc = cloud->size();
	#pragma omp parallel for
	for (int kk = 0; kk < 16; kk++) {//栅格填充
		for (int i = mc / 16 * kk; i < (kk == 15 ? mc : mc / 16 * (kk + 1)); i++) {
			PointType point = (*cloud)[i];
			int x = (int)(point.x * 100 / GRID_SIZE) + pic_size / 2;
			int y = (int)(point.y * 100 / GRID_SIZE) + pic_size / 2;
			map2Dto3D[_Key(x, y)].push_back(point);
		}
	}
	float z_average = 0;//一个栅格内z坐标的平均值
	float z_loss = 0;//一个栅格内z坐标的方差值
	for (std::map<int, std::vector<PointType>>::iterator it = map2Dto3D.begin(); it != map2Dto3D.end(); ++it) {
		for (int k = 0; k < it->second.size(); ++k)
			z_average += it->second[k].z;
		z_average /= it->second.size();
		for (int k = 0; k < it->second.size(); ++k)
			z_loss += pow(it->second[k].z - z_average, 2);
		z_loss /= it->second.size();
		/*cout << "loss:" << z_loss << endl;
		cout << "avg:" << z_average << endl;
		cout << "min:" << z_min << endl;*/
		if (z_loss <= THRESHOLD && z_average <= GROUD_DEV) {
			it = map2Dto3D.erase(it);//
			if (it == map2Dto3D.end())
				break;
		}

		else {
			black_picture.at<uchar>(it->first >> 16,it->first & 0xFFFF) = 125;//
		}
		z_average = 0;
		z_loss = 0;
	}
	//进度线 _Key
	//#pragma omp parallel for//将for循环分为16个线程并行执行
	/*for (int k = 0; k < 16; k++) {
		for (int i = mc / 16 * k; i < (k == 15 ? mc : mc / 16 * (k + 1)); i++) {
			PointType point = (*cloud)[i];
			int x = (int)(point.x * 100 / GRID_SIZE) + pic_size / 2;
			int y = (int)(point.y * 100 / GRID_SIZE) + pic_size / 2;
			black_picture.at<uchar>(y, x) = 125;
			map2Dto3D[_Key(x, y)].push_back(point);
		}
	}*/
	std::vector<std::vector<cv::Point>> filtered_contours;
	cz_region(black_picture, filtered_contours, 0, true, pic_size, pic_size);
	/*for (auto var = filtered_contours.begin(); var != filtered_contours.end();)
	{
		if (var->size() < 5) {
			var = filtered_contours.erase(var);
			if (var == filtered_contours.end())
				break;
		}
		else
			var++;
	}*/
	//TIME_FUNC;
	//std::cout << " Finish 2-D clusting" << std::endl << std::endl;
	//TIME_FUNC;
	//std::cout << " Start rasing dimension" << std::endl;
	//int combined_count = 0;
	pcl::PointCloud<PointType>::Ptr contours_cloud(new pcl::PointCloud<PointType>);
	(*contours_cloud).reserve(cloud->size());
	//#pragma omp parallel for
	for (int i = 0; i < filtered_contours.size(); i++) {
		std::vector<cv::Point> single_cluster = filtered_contours[i];
		/*if (single_cluster.size() == 0) {
			combined_count++;
			continue;
		}*/
		/*int r = (rand() % (255 - 0 + 1));
		int g = (rand() % (255 - 0 + 1));
		int b = (rand() % (255 - 0 + 1));*/
		float zMin = 10000;
		float zMax = -10000;
		/*float xMin = 10000;
		float xMax = -10000;
		float yMin = 10000;
		float yMax = -10000;*///未使用
		for (int j = 0; j < single_cluster.size(); j++) {//遍历该目标的每个cv::point
			cv::Point point = single_cluster[j];
			int x = point.x;
			int y = point.y;
			std::vector<PointType>& V = map2Dto3D[_Key(x, y)];
			size_t sz = V.size();
			for (int k = 0; k < sz; k++) {
				PointType point = V[k];
				/*point.r = r;
				point.g = g;
				point.b = b;*/
				if (zMin > point.z) {
					zMin = point.z;
				}
				if (zMax < point.z) {
					zMax = point.z;
				}
				/*if (yMin > point.y) {//未使用
				yMin = point.y;
				}
				if (yMax < point.y) {
				yMax = point.y;
				}
				if (xMin > point.x) {
				xMin = point.x;
				}
				if (xMax < point.x) {
				xMax = point.x;
				}*/
				(*contours_cloud).push_back(point);
			}
		}
		if (single_cluster.size() == 0)
			continue;
		
		cv::RotatedRect rec = cv::minAreaRect(single_cluster);
		cv::Point2f vertices[4];
		rec.points(vertices);

		//cout << "hmax:" << zMax << "hmin:" << zMin << endl;
		//cout << "length:"<<rec.size.height << endl;
		//cout << "width:" <<rec.size.width << endl;
		/*if (((zMax - zMin) <= 1.6 && (zMax - zMin) >= 1.4 && rec.size.height <= 43 && rec.size.height >= 38 && 
			rec.size.width <= 18 && rec.size.width >= 16) || ((zMax - zMin) <= 1.6 && (zMax - zMin) >= 1.4 && 
				rec.size.height <= 18 && rec.size.height >= 16 && rec.size.width <= 43 && rec.size.width >= 38) ||
					((zMax - zMin) <= 24 && (zMax - zMin) >= 12 && rec.size.height <= 5 && rec.size.height >= 0 &&
						rec.size.width <= 10 && rec.size.width >= 3) || ((zMax - zMin) <= 24 && (zMax - zMin) >= 12 && 
							rec.size.height <= 10 && rec.size.height >= 3 && rec.size.width <= 5 && rec.size.width >= 0)) {*/
		//if (zMin <= ) {
		//进度线，把车和人的点云筛出并计算特征

			//cout << "number" + to_string(i) << zMax << "a" << zMin << "a" << to_string(rec.size.height) << "a" << to_string(rec.size.width) << endl;
		PointType cube_points[8];
		int r = 0, g = 0, b = 0;
		//cout << "point " <<filtered_contours[i].size() << endl;
		//if ((rec.size.height <= 55 && rec.size.height >= 20 && rec.size.width <= 45 && rec.size.width >= 15 && filtered_contours[i].size() >= 5)||
		//	(rec.size.height <= 45 && rec.size.height >= 15 && rec.size.width <= 55 && rec.size.height >= 20 && filtered_contours[i].size() >= 5))//判定为车
		if ((sqrt(pow(rec.size.height, 2) + pow(rec.size.width, 2)) >= 25 && rec.size.height / rec.size.width <= 4 && rec.size.height / rec.size.width >= 0.3 && rec.size.height <= 55 && rec.size.height >= 10 && rec.size.width <= 45 && rec.size.width >= 5 && filtered_contours[i].size() >= 20) ||
			(sqrt(pow(rec.size.height, 2) + pow(rec.size.width, 2)) >= 25 && rec.size.width / rec.size.height <= 4 && rec.size.width / rec.size.height >= 0.3 && rec.size.height <= 45 && rec.size.height >= 5 && rec.size.width <= 55 && rec.size.height >= 10 && filtered_contours[i].size() >= 20))//判定为车
			r = 255;
		else if((zMax - zMin) <= 2.5 && (zMax - zMin) >= 1.2 && rec.size.height <= 7 && rec.size.height >= 0 &&
			rec.size.width <= 7 && rec.size.width >= 0 && filtered_contours[i].size() >= 5 && filtered_contours[i].size() <= 700 && 
			((zMax - zMin)*10/rec.size.height >= 3 || (zMax - zMin)*10/rec.size.width >= 3))//判定为人
			g = 255;
		else
			b = 255;
		for (int j = 0; j < 4; j++) {
			line(black_picture, vertices[j], vertices[(j + 1) % 4], cv::Scalar(255));
			cube_points[j].x = (vertices[j].x - pic_size / 2) * GRID_SIZE / 100;
			cube_points[j].y = (vertices[j].y - pic_size / 2) * GRID_SIZE / 100;
			cube_points[j].z = zMin;
			cube_points[j].r = r;
			cube_points[j].g = g;
			cube_points[j].b = b;
			//std::cout << "hell" << endl;
			//std::cout << "n" << to_string(j) << "x " << cube_points[j].x;
			//std::cout << "y " << cube_points[j].y << "z " << cube_points[j].z << std::endl;
			cube_points[j + 4].x = (vertices[j].x - pic_size / 2) * GRID_SIZE / 100;
			cube_points[j + 4].y = (vertices[j].y - pic_size / 2) * GRID_SIZE / 100;
			cube_points[j + 4].z = zMax;
			cube_points[j + 4].r = r;
			cube_points[j + 4].g = g;
			cube_points[j + 4].b = b;
			//std::cout << "n" << to_string(j + 4) << "x " << cube_points[j + 4].x;
			//std::cout << "y " << cube_points[j + 4].y << "z " << cube_points[j + 4].z << std::endl;
		}
		//cout << cube_points[0].x;
		if (b != 255) {
			uint32_t handle = PointViewer::get_instance()->add_cube(cube_points);
			cube_handles.push_back(handle);
		}
			
		//}
	}
	//printf("contours %d\n", contours_cloud->size());
	return contours_cloud;
}

int DimensionReductionCluster::cz_region(cv::Mat src, std::vector<std::vector<cv::Point>> &points, int filter_size, bool fill_flag, int pic_width, int pic_height) {
	cv::Mat tmp = src.clone();
	cv::Mat tmp1 = src.clone();
	cv::dilate(tmp, tmp, cv::Mat(cv::Size(3, 3), CV_8U));//opencv的点云轮廓膨胀，使目标鲁棒性更强
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);//从二值图像tmp(black_picture)中计算轮廓
	cv::Scalar filledcolor = cv::Scalar(255);
 	for (size_t i = 0; i < contours.size(); i++) {
		std::vector<cv::Point> points1;
		points1.reserve(contours[i].size());

		points.push_back(points1);
	}
	//std::cout << " [loop1 end]" << std::endl;
	int time_sum = 0;
	int loop_sum = 0;
	cv::Mat drawing(tmp.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat tplate3(tmp.size(), CV_8UC1, cv::Scalar(0)); //
	for (size_t i = 0; i < contours.size(); i++) {
		/*if (contours[i].size() < 50 || contours[i].size() > 150) {
			continue;
		}*/
		cv::RotatedRect rRect = cv::minAreaRect(contours[i]);
		cv::Rect brect = rRect.boundingRect();
		cv::drawContours(drawing, contours, i, filledcolor, CV_FILLED, 8);
		tmp1.copyTo(tplate3, drawing);
		const int i_ = (brect.x + 1) > 0 ? (brect.x + 1) : 0;
		const int j_ = (brect.y + 1) > 0 ? (brect.y + 1) : 0;
		const int ii_max = (brect.x + brect.width) < pic_width ? (brect.x + brect.width) : pic_width;
		const int jj_max = (brect.y + brect.height) < pic_height ? (brect.y + brect.height) : pic_height;
		std::vector<cv::Point>& temp_points = points[i];
		for (int jj = j_; jj < jj_max; jj++) { //rows
			uchar * data = tplate3.ptr<uchar>(jj);
			for (int ii = i_; ii < ii_max; ii++) {//cols
				if (data[ii] != 0) {
					temp_points.push_back(cv::Point(ii, jj));
				}
			}
		}
	}
	//cvNamedWindow("window");
	//cv::imshow("window", drawing);
	return 0;
}

