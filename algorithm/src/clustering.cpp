#include "clustering.h"

using namespace mammoth::util;
using namespace mammoth::algorithm;

std::vector<uint32_t> DimensionReductionCluster::cube_handles;

void DimensionReductionCluster::start_clusting(pcl::PointCloud<PointType>::Ptr & cloud) {
	//filtering
	pcl::PassThrough<PointType> passThrough;
	passThrough.setInputCloud(cloud);
	passThrough.setFilterLimitsNegative(false);
	passThrough.setFilterFieldName("x");
	passThrough.setFilterLimits(-1, 1); //100
	passThrough.filter(*cloud);
	passThrough.setFilterFieldName("y");
	passThrough.setFilterLimits(6, 14); //100
	passThrough.filter(*cloud);
	passThrough.setFilterFieldName("z");
	passThrough.setFilterLimits(-2.64, 30);
	passThrough.filter(*cloud);
	PointViewer::get_instance()->remove_cubes(cube_handles);
	cloud = birdview_picture_grid(cloud);
}

pcl::PointCloud<PointType>::Ptr DimensionReductionCluster::birdview_picture_grid(pcl::PointCloud<PointType>::Ptr& cloud) {
	//TIME_FUNC; 
	//std::cout << " Start dimension reduction" << std::endl;
	int grid_width = 20;//cm  20
	int pic_size = 20000 / grid_width;// 20000
	int rows = pic_size;
	int cols = pic_size;
	int min_size = 0;
	bool fill_pixel_flag = true;
	cv::Mat black_picture(rows, cols, CV_8UC1, cv::Scalar(0));
	srand((unsigned)time(NULL));
	std::map<int, std::vector<PointType>> map2Dto3D;
	int mc = cloud->size();
	//#pragma omp parallel for
	for (int k = 0; k < 16; k++) {
		for (int i = mc / 16 * k; i < (k == 15 ? mc : mc / 16 * (k + 1)); i++) {
			PointType point = (*cloud)[i];
			int x = (int)(point.x * 100 / grid_width) + pic_size / 2;
			int y = (int)(point.y * 100 / grid_width) + pic_size / 2;
			black_picture.at<uchar>(y, x) = 125;
			map2Dto3D[_Key(x, y)].push_back(point);
		}
	}
	std::vector<std::vector<cv::Point>> filtered_contours;
	cz_region(black_picture, filtered_contours, 0, true, 1000, 1000);
	//TIME_FUNC;
	//std::cout << " Finish 2-D clusting" << std::endl << std::endl;
	//TIME_FUNC;
	//std::cout << " Start rasing dimension" << std::endl;
	int combined_count = 0;
	pcl::PointCloud<PointType>::Ptr contours_cloud(new pcl::PointCloud<PointType>);
	(*contours_cloud).reserve(cloud->size());
	//#pragma omp parallel for

	for (int i = 0; i < filtered_contours.size(); i++) {
		std::vector<cv::Point> single_cluster = filtered_contours[i];
		if (single_cluster.size() == 0) {
			combined_count++;
			continue;
		}

		int r = (rand() % (255 - 0 + 1));
		int g = (rand() % (255 - 0 + 1));
		int b = (rand() % (255 - 0 + 1));
		float zMin = 10000;
		float zMax = -10000;
		float xMin = 10000;
		float xMax = -10000;
		float yMin = 10000;
		float yMax = -10000;
		for (int j = 0; j < single_cluster.size(); j++) {
			cv::Point point = single_cluster[j];
			int x = point.x;
			int y = point.y;
			std::vector<PointType>& V = map2Dto3D[_Key(x, y)];
			size_t sz = V.size();
			for (int k = 0; k < sz; k++) {
				PointType point = V[k];
				point.r = r;
				point.g = g;
				point.b = b;
				if (zMin > point.z) {
					zMin = point.z;
				}
				if (zMax < point.z) {
					zMax = point.z;
				}
				if (yMin > point.y) {
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
				}
				(*contours_cloud).push_back(point);
			}
		}
		cv::RotatedRect rec = cv::minAreaRect(single_cluster);
		cv::Point2f vertices[4];
		rec.points(vertices);
		PointType cube_points[8];
		for (int j = 0; j < 4; j++) {
			line(black_picture, vertices[j], vertices[(j + 1) % 4], cv::Scalar(255));
			cube_points[j].x = (vertices[j].x - pic_size / 2) * grid_width / 100;
			cube_points[j].y = (vertices[j].y - pic_size / 2) * grid_width / 100;
			cube_points[j].z = zMin;
			cube_points[j].r = 255;
			cube_points[j].g = 255;
			cube_points[j].b = 0;
			cube_points[j + 4].x = (vertices[j].x - pic_size / 2) * grid_width / 100;
			cube_points[j + 4].y = (vertices[j].y - pic_size / 2) * grid_width / 100;
			cube_points[j + 4].z = zMax;
			cube_points[j + 4].r = 255;
			cube_points[j + 4].g = 255;
			cube_points[j + 4].b = 0;
		}
		uint32_t handle = PointViewer::get_instance()->add_cube(cube_points);
		cube_handles.push_back(handle);
	}
	//printf("contours %d\n", contours_cloud->size());
	return contours_cloud;
}

int DimensionReductionCluster::cz_region(cv::Mat src, std::vector<std::vector<cv::Point>> &points, int filter_size, bool fill_flag, int pic_width, int pic_height) {
	cv::Mat tmp = src.clone();
	cv::Mat tmp1 = src.clone();
	cv::dilate(tmp, tmp, cv::Mat(cv::Size(2, 2), CV_8U));
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
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
	cv::Mat tplate3(tmp.size(), CV_8UC1, cv::Scalar(0)); //����һ�Ŵ洢ԭʼ���ͼ
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

