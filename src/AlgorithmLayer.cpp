#include "AlgorithmLayer.h"

using namespace mammoth::config;
using namespace mammoth::layer;
using Eigen::MatrixXd;

JluSlamLayer * JluSlamLayer::layer = nullptr;

JluSlamLayer::JluSlamLayer() {

}

JluSlamLayer::~JluSlamLayer() {
	if (layer != nullptr) {
		delete layer;
	}
}

JluSlamLayer * JluSlamLayer::get_instance() {
	if (layer == nullptr) {
		layer = new JluSlamLayer();
	}
	return layer;
}

void JluSlamLayer::start_slam(const std::string& gps_folder_path, const std::string& pcd_folder_path, int start_number) {
	std::vector<std::string> file_names;
	std::string folder_path = pcd_folder_path
#ifdef _WIN32
	+ "\\*.pcd"
#endif
	;
	FileUtil::get_all_files(folder_path, file_names);
	HPCD hpcd = PcdUtil::pcdOpen("E:\\DataSpace\\map\\0206\\0206-1.pcd");
	char temp[100];
	FileUtil trace_file("E:\\DataSpace\\trace\\0206\\trace_0206-1.txt", 2);
	for (int i = 0; i < file_names.size(); i++) {
		std::string number_str = file_names[i].substr(0, file_names[i].find_first_of('_'));
		std::string file_path = file_names[i];
		//ï¿½ï¿½ï¿½Ä¼ï¿½
		std::string read_pcd_path = pcd_folder_path + "/" + file_path;
		printf("%s\n", file_path.c_str());
		char temp[100];
		sprintf(temp, "%d_gps.txt", atoi(number_str.c_str()));

		std::string gps_path = temp;
		std::string read_gps_path = gps_folder_path + "/" + gps_path;
		int a = 0;

		PCDFILE pcd_file;
		PcdUtil::read_pcd_file(read_pcd_path, &pcd_file);
		int point_count = pcd_file.header.Points;
		FileUtil gps_file(read_gps_path.c_str(), 0);

		std::string record;
		std::string next;
		do {
			record = next;
			next = gps_file.read_line();
		} while (next.length() > 0);
		record = record.substr(record.find_first_of("#") + 1);
		//
		std::string longitude_str = record.substr(0, record.find_first_of(" "));
		record = record.substr(record.find_first_of(" ") + 1);
		std::string latitude_str = record.substr(0, record.find_first_of(" "));
		record = record.substr(record.find_first_of(" ") + 1);
		record.substr(0, record.find_first_of(" "));
		record = record.substr(record.find_first_of(" ") + 1);
		std::string yaw_str = record.substr(0, record.find_first_of(" "));
		GPGGA_Data gga_data;
		gga_data.lon = atof(longitude_str.c_str());
		gga_data.lat = atof(latitude_str.c_str());
		PTNLAVR_Data ptnlavr_data;
		ptnlavr_data.yaw = atof(yaw_str.c_str());
		double lat0 = 4349.13958348;
		double lon0 = 12516.60912408;
		Vec2d location_vec = GnssTransformLayer::get_instance()->get_distance1(gga_data.lat, gga_data.lon, lat0, lon0);
		Vec2d trans_vec;
		trans_vec.x = -1 * location_vec.x;
		trans_vec.y = -1 * location_vec.y;
		//ï¿½ï¿½ï¿½ã³µï¿½Ä·ï¿½ï¿½ï¿½
		//ï¿½ï¿½È¡ï¿½ï¿½ï¿½Ä·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½  ï¿½ï¿½ï¿½ï¿½GPSï¿½Ú³ï¿½Î² ï¿½ï¿½ï¿½ï¿½ï¿½Ç³ï¿½Í·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½Ô°ï¿½GPSï¿½Ç¶ï¿½-90ï¿½ï¿½Îªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ë·½ï¿½ï¿½ï¿?
		float car_angle = (360 - ptnlavr_data.yaw) + 180;

		float xoy_rotation_angle = car_angle * PI / 180;
		float xoz_rotation_angle = -73 * PI / 180;

		float theta1 = xoz_rotation_angle;
		float theta2 = xoy_rotation_angle;
		//¼ÆËã±ä»»¾ØÕó
		//Ô­Ê¼¾ØÕóÒ»¹²5¸ö T1~T5
		//T1 XOZÄæÊ±ÕëÐý×ª
		//T2 XOYÄæÊ±ÕëÐý×ª
		//T3 YÖá·­×ª
		//T4 XOYÆ½ÒÆ
		//T5 YÖá·­×ª
		//Êµ¼ÊÊ¹ÓÃ3¸ö¾ØÕó Q1~Q3
		//Q1ÎªÔ­Ê¼¾ØÕóÖÐµÄÇ°Èý¸ö¾ØÕóÏà³Ë
		//Q2ÎªÔ­Ê¼¾ØÕóT4
		//Q3ÎªÔ­Ê¼¾ØÕóT5
		//¼ÙÉèÊý¾Ý¾ØÕóÎªnÐÐ3ÁÐµÄÏòÁ¿D
		//Ôò±ä»»ºóµÄ½á¹ûÊÇ ((D * Q1) + Q2) * Q3
		float * data = (float *)pcd_file.pData;
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		DoTransform(theta1, theta2, trans_vec.x, trans_vec.y, pcd_file.pData, pcd_file.header.Points);
		float x = data[0];
		float y = data[1];
		float z = data[2];
		memset(temp, 0, 100);
		sprintf(temp, "%f %f %f %f", theta2, x, y, z);
		trace_file.write_line(temp);
		PcdUtil::pcdWrite(hpcd, (XYZRGBA *)pcd_file.pData, pcd_file.header.Points);
		PcdUtil::pcdRelease(&pcd_file);
	}
	PcdUtil::pcdClose(hpcd);
#ifdef _WIN32
	system("pause");
#endif
}

void JluSlamLayer::DoTransform(float theta1, float theta2, float trans_x, float trans_y, void* pData, size_t count) {
	float* fv = (float*)pData;
	register __m128 Q1_1 = _mm_set_ps(0.0f, -1 * sinf(theta1), cosf(theta1) * sinf(theta2), cosf(theta1) * cosf(theta2));
	register __m128 Q1_2 = _mm_set_ps(0, 0, -1 * cosf(theta2), sinf(theta2));
	register __m128 Q1_3 = _mm_set_ps(0.0f, cosf(theta1), sinf(theta1) * sinf(theta2), sinf(theta1) * cosf(theta2));
	register __m128 Q2_1 = _mm_set_ps(0.0f, 0.0f, -trans_x, trans_y);
	register __m128 Q3_1 = _mm_set_ps(1.0f, 1.0f, -1.0f, 1.0f);
	for (size_t j = 0; j < count; j++) {
		register __m128 V = _mm_load_ps(fv);
		register __m128 T = _mm_shuffle_ps(V, V, 0x00);//D1 * Q1
		register __m128 R = _mm_mul_ps(T, Q1_1);
		T = _mm_shuffle_ps(V, V, 0x55);
		T = _mm_mul_ps(T, Q1_2);
		R = _mm_add_ps(R, T);
		T = _mm_shuffle_ps(V, V, 0xAA);
		T = _mm_mul_ps(T, Q1_3);
		R = _mm_add_ps(R, T);
		R = _mm_add_ps(R, Q2_1); // (D1*Q1)+Q2
		R = _mm_mul_ps(R, Q3_1); // ((D1*Q1) + Q2) * Q3
		R = _mm_shuffle_ps(R, R, 0xE1);
		R.m128_f32[3] = V.m128_f32[3];
		_mm_store_ps(fv, R);
		fv += 4;
	}
}



std::vector<uint32_t> DimensionReductionCluster::cube_handles;

void DimensionReductionCluster::start_clusting(pcl::PointCloud<PointType>::Ptr & cloud) {
	//ï¿½Ë²ï¿½
	pcl::PassThrough<PointType> passThrough;
	passThrough.setInputCloud(cloud);
	passThrough.setFilterLimitsNegative(false);
	passThrough.setFilterFieldName("x");
	passThrough.setFilterLimits(-5, 5); //100
	passThrough.filter(*cloud);
	passThrough.setFilterFieldName("y");
	passThrough.setFilterLimits(-5, 5); //100
	passThrough.filter(*cloud);
	passThrough.setFilterFieldName("z");
	passThrough.setFilterLimits(-2.4, 30);
	passThrough.filter(*cloud);
	//É¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	PointViewer::get_instance()->remove_cubes(cube_handles);
	cloud = birdview_picture_grid(cloud);
	
}

pcl::PointCloud<PointType>::Ptr DimensionReductionCluster::birdview_picture_grid(pcl::PointCloud<PointType>::Ptr& cloud) {
	//ï¿½ï¿½Î¬Ó³ï¿½ï¿½
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
	//ï¿½ï¿½ï¿½ï¿½Ó³ï¿½ï¿½
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
	printf("contours %d\n", contours_cloud->size());
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
	cv::Mat tplate3(tmp.size(), CV_8UC1, cv::Scalar(0)); //ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Å´æ´¢Ô­Ê¼ï¿½ï¿½ï¿½Í?
	for (size_t i = 0; i < contours.size(); i++) {
		if (contours[i].size() < 50 || contours[i].size() > 150) {
			continue;
		}
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
	cvNamedWindow("window");
	cv::imshow("window", drawing);
	return 0;
}

ImuLocalization::ImuLocalization(float v0, float delta_t) {
	this->v0 = v0;
	this->delta_t = delta_t;
}

void ImuLocalization::calculate_distance(
	float * s,
	float * v,
	float * delta_v) {
	
	v[0] = v[0] + delta_v[0];
	s[0] += (v[0] - 0.5 * delta_v[0]) * delta_t;

	v[1] = v[1] + delta_v[1];
	s[1] += (v[1] - 0.5 * delta_v[1]) * delta_t;

	v[2] = v[2] + delta_v[2];
	s[2] += (v[2] - 0.5 * delta_v[2]) * delta_t;
}

