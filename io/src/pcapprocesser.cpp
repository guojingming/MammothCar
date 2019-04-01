#include "pcapprocesser.h"

using namespace mammoth;

pcap_t * PcapProcesser::device = nullptr;

double * LidarConfig::hdl32_vertical_angles = new double[32]{
    -30.67, -9.33, -29.33, -8.00, -28.00, -6.66, -26.66, -5.33, -25.33, -4.00, -24.00, -2.67, -22.67, -1.33, -21.33, 0, -20, 1.33, -18.67, 2.67, -17.33, 4, -16, 5.33, -14.67, 6.67, -13.33, 8, -12, 9.33, -10.67, 10.67
};

uint8_t * LidarConfig::hdl32_vertical_ids = new uint8_t[32] {
    -23 * -1 + 9, -7 * -1 + 9, -22 * -1 + 9, -6 * -1 + 9, -21 * -1 + 9, -5 * -1 + 9, -20 * -1 + 9, -4 * -1 + 9, -19 * -1 + 9, -3 * -1 + 9, -18 * -1 + 9, -2 * -1 + 9, -17 * -1 + 9, -1 * -1 + 9, -16 * -1 + 9, 0 * -1 + 9, -15 * -1 + 9, 1 * -1 + 9, -14 * -1 + 9, 2 * -1 + 9, -13 * -1 + 9, 3 * -1 + 9, -12 * -1 + 9, 4 * -1 + 9, -11 * -1 + 9, 5 * -1 + 9, -10 * -1 + 9, 6 * -1 + 9, -9 * -1 + 9, 7 * -1 + 9, -8 * -1 + 9, 8 * -1 + 9
};

double * LidarConfig::vlp16_vertical_angles = new double[32]{
    -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15, -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15
};

uint8_t * LidarConfig::vlp16_vertical_ids = new uint8_t[32]{
    0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15, 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15
};

PcapProcesser * PcapProcesser::layer = nullptr;

PcapProcesser::PcapProcesser() {
	angle_piece = 120;
	storage_flag = false;
    path_prefix = "E:/DataSpace/LidarDataSpace/lidar_";
	std::string time_code = TimeUtil::get_time_code();
	//std::string time_code_millsecond = TimeUtil::get_time_code_millsecond();
	root_path = path_prefix + time_code + "/";
}

PcapProcesser * PcapProcesser::get_instance() {
	if (layer == nullptr) {
		layer = new PcapProcesser();
	}
	return layer;
}

PcapProcesser::~PcapProcesser() {
	if (layer != nullptr) {
		delete layer;
	}
}

void PcapProcesser::trans_pcap_to_pcd(std::string pcap_path, std::vector<pcl::PointCloud<PointType>::Ptr> & vec, int seg_count) {
	pcap_t * adhandle = get_pcap_file_data(pcap_path);
	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;
	int count = 0;
	int maxFlectivity = 0;
	int res = 0;
	int frame_count = vec.size();
	int current_frame_count = 0;
	int index_frame_count = 0;
	for (int i = 0; i < frame_count; i++) {
		vec[i]->clear();
	}
	int pack_id = 0;
	while ((res = pcap_next_ex(adhandle, &pkthdr, &pktdata)) >= 0) {
		if (pkthdr->caplen == 1248) {
			///////////////////////
			int block_count = 12;
			int channel_count = 32;
			int flag_size = 2;
			int head_size = 42;
			int block_size = 100;
			int angle_address = 2;
			int angle_size = 2;
			int unit1_distance_address = 4;
			int unit_distance_size = 2;
			int unit1_reflectivity_address = unit1_distance_address + unit_distance_size;
			int unit_reflectivity_size = 1;
			int channel_size = unit_distance_size + unit_reflectivity_size;
			float * angles = new float[block_count];
			int * distance_mm = new int[channel_count * block_count];
			int * flectivity = new int[channel_count * block_count];
			memset(angles, 0, sizeof(float) * block_count);
			memset(distance_mm, 0, sizeof(int) * channel_count * block_count);
			memset(flectivity, 0, sizeof(int) * channel_count * block_count);
			static std::vector<double> test_data[32];
			for (int i = 0; i < block_count; i++) {
				//get azimuth
				for (int k = angle_size - 1; k >= 0; k--) {
					int index = head_size + i * block_size + angle_address + k;
					float data = pktdata[head_size + i * block_size + angle_address + k];
					angles[i] = angles[i] * 256 + data;
				}
				angles[i] = angles[i] / 100;
				for (int j = 0; j < channel_count; j++) {
					for (int k = unit_distance_size - 1; k >= 0; k--) {
						distance_mm[i * channel_count + j] = distance_mm[i * channel_count + j] * 256 + pktdata[head_size + flag_size + i * block_size + angle_size + j * channel_size + k];
					}
					for (int k = unit_reflectivity_size - 1; k >= 0; k--) {
						flectivity[i * channel_count + j] = flectivity[i * channel_count + j] * 256 + pktdata[head_size + flag_size + i * block_size + angle_size + j * channel_size + unit_distance_size + k];
						if (maxFlectivity < flectivity[i * channel_count + j]) {
							maxFlectivity = flectivity[i * channel_count + j];
						}
						
					}
					MyPoint3D point;
					float distance = distance_mm[i * channel_count + j] / 1000.0;
					int flectivity_value = flectivity[i * channel_count + j];
					float horizontal_angle = angles[i] * PI / 180;
					float vertical_angle = LidarConfig::hdl32_vertical_angles[j % 32] * PI / 180;
					point.z = distance * sin(vertical_angle);
					point.y = distance * cos(vertical_angle) * sin(-horizontal_angle);
					point.x = distance * cos(vertical_angle) * cos(-horizontal_angle);
					PointType pclPoint;
					pclPoint.x = 2 * (point.y);
					pclPoint.y = 2 * (point.x);
					pclPoint.z = 2 * point.z;
					pclPoint.r = 0;
					pclPoint.b = flectivity_value;
					pclPoint.g = LidarConfig::hdl32_vertical_ids[j % 32];
					vec[index_frame_count]->push_back(pclPoint);
				}
			}
			if (count >= 240) {
				count = 0;
				if (current_frame_count == seg_count) {
					float seg = maxFlectivity * 1.0 / 256;
					int pointSize = vec[index_frame_count]->points.size();
					for (int i = 0; i < pointSize; i++) {
						uint8_t flex = vec[index_frame_count]->points[i].b;
						uint8_t laser_id = vec[index_frame_count]->points[i].g;
						vec[index_frame_count]->points[i].r = flex;
						vec[index_frame_count]->points[i].b = 128;
						vec[index_frame_count]->points[i].g = laser_id;
					}
					index_frame_count++;
					if (index_frame_count == frame_count) {
						pcap_close(adhandle);
						return;
					}
					current_frame_count = 0;
				} else {
					current_frame_count++;
				}
			}
			count++;
			delete angles;
			delete distance_mm;
			delete flectivity;
		}
		if (res == 0) {
			continue;
		}
		pack_id++;
	}
	pcap_close(adhandle);
	return;
}

void PcapProcesser::play_pcap_file(std::string pcap_path, int start_packet_number) {
	pcap_t * adhandle = get_pcap_file_data(pcap_path);
	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;
	bool start_flag = false;
	int count = 0;
	int start_count = 0;
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	int maxFlectivity = 0;
	int res = 0;
	float angle_min = 0;
	float angle_max = 0;
	bool start_counting = true;
	while ((res = pcap_next_ex(adhandle, &pkthdr, &pktdata)) >= 0) {
		if (pkthdr->caplen == 1248) {
			///////////////////////
			int block_count = 12;
			int channel_count = 32;
			int flag_size = 2;
			int head_size = 42;
			int block_size = 100;
			int angle_address = 2;
			int angle_size = 2;
			int unit1_distance_address = 4;
			int unit_distance_size = 2;
			int unit1_reflectivity_address = unit1_distance_address + unit_distance_size;
			int unit_reflectivity_size = 1;
			int channel_size = unit_distance_size + unit_reflectivity_size;
			float * angles = new float[block_count];
			int * distance_mm = new int[channel_count * block_count];
			int * flectivity = new int[channel_count * block_count];
			memset(angles, 0, sizeof(float) * block_count);
			memset(distance_mm, 0, sizeof(int) * channel_count * block_count);
			memset(flectivity, 0, sizeof(int) * channel_count * block_count);
			for (int i = 0; i < block_count; i++) {
				for (int k = angle_size - 1; k >= 0; k--) {
					int index = head_size + i * block_size + angle_address + k;
					float data = pktdata[head_size + i * block_size + angle_address + k];
					angles[i] = angles[i] * 256 + data;
				}
				angles[i] = angles[i] / 100;
				for (int j = 0; j < channel_count; j++) {
					for (int k = unit_distance_size - 1; k >= 0; k--) {
						distance_mm[i * channel_count + j] = distance_mm[i * channel_count + j] * 256 + pktdata[head_size + flag_size + i * block_size + angle_size + j * channel_size + k];
					}
					for (int k = unit_reflectivity_size - 1; k >= 0; k--) {
						flectivity[i * channel_count + j] = flectivity[i * channel_count + j] * 256 + pktdata[head_size + flag_size + i * block_size + angle_size + j * channel_size + unit_distance_size + k];
						if (maxFlectivity < flectivity[i * channel_count + j]) {
							maxFlectivity = flectivity[i * channel_count + j];
						}
					}
				}
			}
			for (int i = 0; i < block_count; i++) {
				for (int j = 0; j < channel_count; j++) {
					MyPoint3D point;
					float distance = distance_mm[i * channel_count + j] / 1000.0;
					int flectivity_value = flectivity[i * channel_count + j];
					float horizontal_angle = angles[i] * PI / 180;
					float vertical_angle = LidarConfig::hdl32_vertical_angles[j % 32] * PI / 180;
					point.z = distance * sin(vertical_angle);
					point.y = distance * cos(vertical_angle) * sin(horizontal_angle);
					point.x = distance * cos(vertical_angle) * cos(horizontal_angle);
					PointType pclPoint;
					pclPoint.x = 2 * (point.y);
					pclPoint.y = -2 * (point.x);
					pclPoint.z = 2 * point.z;
					pclPoint.r = 0;
					pclPoint.b = flectivity_value;
					pclPoint.g = 0;
					scene->push_back(pclPoint);
				}
			}
			if (count >= 240) {
				float seg = maxFlectivity * 1.0 / 256;
				int pointSize = scene->points.size();
				for (int i = 0; i < scene->points.size(); i++) {
					int flex = scene->points[i].b;
					scene->points[i].r = (int)(flex / seg);
					scene->points[i].b = 255 - (int)(flex / seg) * 5;
					scene->points[i].g = (int)(flex / seg) * 5;
				}
				PointViewer::get_instance()->set_point_cloud(scene);
				scene->clear();
			}
			delete angles;
			delete distance_mm;
			delete flectivity;
		}
		count++;
	}
	if (res == -1) {
		printf("Error reading the packets: %s\n", pcap_geterr(adhandle));
		return;
	}
	pcap_close(adhandle);
	return;
}

pcap_t * PcapProcesser::get_pcap_file_data(std::string pcap_path) {
	char errbuf[100];
	pcap_t *pfile = pcap_open_offline(pcap_path.c_str(), errbuf);
	if (NULL == pfile) {
		printf("%s\n", errbuf);
		return nullptr;
	}
	return pfile;
}

pcap_t * PcapProcesser::get_pcap_dev_handle(int ethernet_number){
	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;	
	pcap_if_t *alldevs = NULL;
	pcap_if_t *d = NULL;
	int inum = 0;
	int i = 0;
	pcap_t *adhandle;
	char errbuf[PCAP_ERRBUF_SIZE];
	/* Retrieve the device list */
	if (pcap_findalldevs(&alldevs, errbuf) == -1) {
		fprintf(stderr, "Error in pcap_findalldevs: %s\n", errbuf);
		exit(1);
	}
	/* Print the list */
	for (d = alldevs; d; d = d->next) {
		printf("%d. %s", ++i, d->name);
		if (d->description)
			printf(" (%s)\n", d->description);
		else
			printf(" (No description available)\n");
	}
	if (i == 0) {
		printf("\nNo interfaces found! Make sure WinPcap is installed.\n");
		return nullptr;
	}
	printf("Enter the interface number (1-%d):", i);
	//scanf("%d", &inum);
	inum = ethernet_number;
	if (inum < 1 || inum > i) {
		printf("\nIntrface number out of range.\n");
		/* Free the device list */
		pcap_freealldevs(alldevs);
		return nullptr;
	}
	/* Jump to the selected adapter */
	for (d = alldevs, i = 0; i< inum - 1; d = d->next, i++);
	/* Open the device */
	/* Open the adapter */
	if ((adhandle = pcap_open_live(d->name,   // ����������
		65536,         // Ҫ��ȡ���ݰ��Ĵ�С
		// 65536 grants that the whole packet will be captured on all the MACs.
		1,             // �������ڻ���ģʽ
		1000,          // ���ݰ���ȡ��ʱ
		errbuf         // ��Ŵ�����Ϣ�Ļ�����
		)) == NULL) {
		fprintf(stderr, "\nUnable to open the adapter. %s is not supported by WinPcap\n", d->name);
		/* Free the device list */
		pcap_freealldevs(alldevs);
		return nullptr;
	}
	printf("\nlistening on %s...\n\n", d->description);
	/* At this point, we don't need any more the device list. Free it */
	pcap_freealldevs(alldevs);
	return adhandle;
}

pcap_t * PcapProcesser::get_pcap_dev_handle() {
	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;
	pcap_if_t *alldevs = NULL;
	pcap_if_t *d = NULL;
	int inum = 0;
	int i = 0;
	pcap_t *adhandle;
	char errbuf[PCAP_ERRBUF_SIZE];
	/* Retrieve the device list */
	if (pcap_findalldevs(&alldevs, errbuf) == -1) {
		fprintf(stderr, "Error in pcap_findalldevs: %s\n", errbuf);
		exit(1);
	}
	/* Print the list */
	for (d = alldevs; d; d = d->next) {
		printf("%d. %s", ++i, d->name);
		if (d->description)
			printf(" (%s)\n", d->description);
		else
			printf(" (No description available)\n");
	}
	if (i == 0) {
		printf("\nNo interfaces found! Make sure WinPcap is installed.\n");
		return nullptr;
	}
	printf("Enter the interface number (1-%d):", i);
	scanf("%d", &inum);
	//inum = ethernet_number;
	if (inum < 1 || inum > i) {
		printf("\nIntrface number out of range.\n");
		/* Free the device list */
		pcap_freealldevs(alldevs);
		return nullptr;
	}
	/* Jump to the selected adapter */
	for (d = alldevs, i = 0; i< inum - 1; d = d->next, i++);
	/* Open the device */
	/* Open the adapter */
	if ((adhandle = pcap_open_live(d->name,   // ����������
		65536,         // Ҫ��ȡ���ݰ��Ĵ�С
		// 65536 grants that the whole packet will be captured on all the MACs.
		1,             // �������ڻ���ģʽ
		1000,          // ���ݰ���ȡ��ʱ
		errbuf         // ��Ŵ�����Ϣ�Ļ�����
		)) == NULL) {
		fprintf(stderr, "\nUnable to open the adapter. %s is not supported by WinPcap\n", d->name);
		/* Free the device list */
		pcap_freealldevs(alldevs);
		return nullptr;
	}
	printf("\nlistening on %s...\n\n", d->description);
	/* At this point, we don't need any more the device list. Free it */
	pcap_freealldevs(alldevs);
	return adhandle;
}

void PcapProcesser::parameter_init(float angle_piece, std::string path_prefix) {
	this->angle_piece = angle_piece;
	this->path_prefix = path_prefix;
}

void PcapProcesser::get_current_frame(pcap_t * cur_device, pcl::PointCloud<PointType>::Ptr & scene, int config) {
	scene->clear();
	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;
	int count = 0;
	int maxFlectivity = 0;
	int block_count = 12;
	int channel_count = 32;
	int flag_size = 2;
	int head_size = 42;
	int block_size = 100;
	int angle_address = 2;
	int angle_size = 2;
	int unit_distance_size = 2;
	int unit_reflectivity_size = 1;
	int channel_size = unit_distance_size + unit_reflectivity_size;
	float * angles = new float[block_count];
	int * distance_mm = new int[channel_count * block_count];
	int * flectivity = new int[channel_count * block_count];
	while (pcap_next_ex(cur_device, &pkthdr, &pktdata) >= 0) {
		
		if (pkthdr->caplen == 1248) {
			memset(angles, 0, sizeof(float) * block_count);
			memset(distance_mm, 0, sizeof(int) * channel_count * block_count);
			memset(flectivity, 0, sizeof(int) * channel_count * block_count);
			for (int i = 0; i < block_count; i++) {
				for (int k = angle_size - 1; k >= 0; k--) {
					int index = head_size + i * block_size + angle_address + k;
					float data = pktdata[head_size + i * block_size + angle_address + k];
					angles[i] = angles[i] * 256 + data;
				}
				angles[i] = angles[i] / 100;
				for (int j = 0; j < channel_count; j++) {
					float distance = 0;
					if(config != 2){
						for (int k = unit_distance_size - 1; k >= 0; k--) {
							distance_mm[i * channel_count + j] = distance_mm[i * channel_count + j] * 256 + pktdata[head_size + flag_size + i * block_size + angle_size + j * channel_size + k];
						}
						for (int k = unit_reflectivity_size - 1; k >= 0; k--) {
							flectivity[i * channel_count + j] = flectivity[i * channel_count + j] * 256 + pktdata[head_size + flag_size + i * block_size + angle_size + j * channel_size + unit_distance_size + k];
							if (maxFlectivity < flectivity[i * channel_count + j]) {
								maxFlectivity = flectivity[i * channel_count + j];
							}
						}
						distance = distance_mm[i * channel_count + j] / 1000.0;
					}
					int flectivity_value = flectivity[i * channel_count + j];
					float horizontal_angle = angles[i] * PI / 180;
					
					MyPoint3D point;
					float vertical_angle = 0;
					if (config == 0) {
						vertical_angle = LidarConfig::hdl32_vertical_angles[j % 32] * PI / 180;
					} else if (config == 1) {
						vertical_angle = LidarConfig::vlp16_vertical_angles[j % 32] * PI / 180;
					}
					point.z = distance * sin(vertical_angle);
					point.y = distance * cos(vertical_angle) * sin(-horizontal_angle);
					point.x = distance * cos(vertical_angle) * cos(-horizontal_angle);
					PointType pclPoint;
					pclPoint.x = 2 * point.x;
					pclPoint.y = 2 * point.y;
					pclPoint.z = 2 * point.z;
					if (config == 0) {
						pclPoint.r = flectivity_value;
						pclPoint.b = 200;
						pclPoint.g = LidarConfig::hdl32_vertical_ids[j % 32];
					} else if (config == 1) {
						pclPoint.r = flectivity_value;
						pclPoint.b = LidarConfig::vlp16_vertical_ids[j % 32];
						pclPoint.g = 200;
					}
					scene->push_back(pclPoint);
				}
			}
			if (count >= 180) { //177  180
				delete angles;
				delete distance_mm;
				delete flectivity;
				count = 0;
				break;
			}
			count++;
		}
	}
}