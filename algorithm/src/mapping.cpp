#include "mapping.h"

using namespace mammoth::util;
using namespace mammoth::io;
using namespace mammoth::algorithm;

MappingProcesser * MappingProcesser::layer = nullptr;

MappingProcesser::MappingProcesser() {

}

MappingProcesser::~MappingProcesser() {
	if (layer != nullptr) {
		delete layer;
	}
}

MappingProcesser * MappingProcesser::get_instance() {
	if (layer == nullptr) {
		layer = new MappingProcesser();
	}
	return layer;
}

void MappingProcesser::start_slam(const std::string& gps_folder_path, const std::string& pcd_folder_path, int start_number) {
	std::vector<std::string> file_names;
	std::string folder_path = pcd_folder_path + "\\*.pcd";
	FileUtil::get_all_files(folder_path, file_names);
	HPCD hpcd = PcdUtil::pcdOpen("E:\\DataSpace\\map\\0322-1-piece.pcd");
	char temp[100];
	FileUtil trace_file("E:\\DataSpace\\trace\\0406\\trace_0322-1-piece.txt", 2);
	for (int i = 0; i < 100; i++){
	//for (int i = 0; i < file_names.size(); i++) {
		std::string number_str = file_names[i].substr(0, file_names[i].find_first_of('_'));
		std::string file_path = file_names[i];
		std::string read_pcd_path = pcd_folder_path + "\\" + file_path;
		printf("%s\n", file_path.c_str());
		char temp[100];
		sprintf(temp, "%d_gps.txt", atoi(number_str.c_str()));

		std::string gps_path = temp;
		std::string read_gps_path = gps_folder_path + "\\" + gps_path;
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
		Vec2d location_vec = GnssProcesser::get_instance()->get_distance1(gga_data.lat, gga_data.lon, lat0, lon0);
		Vec2d trans_vec;
		trans_vec.x = -1 * location_vec.x;
		trans_vec.y = -1 * location_vec.y;
		float car_angle = (360 - ptnlavr_data.yaw) + 180;
		float xoy_rotation_angle = car_angle * PI / 180;
		float xoz_rotation_angle = -73 * PI / 180;
		float theta1 = xoz_rotation_angle;//俯仰
		float theta2 = xoy_rotation_angle;//水平
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
	system("pause");
}

void MappingProcesser::DoTransform(float theta1, float theta2, float trans_x, float trans_y, void* pData, size_t count) {
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
