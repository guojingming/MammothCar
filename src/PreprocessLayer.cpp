#include "PreprocessLayer.h"

using namespace mammoth::layer;
using namespace mammoth::config;

pcap_t* PcapTransformLayer::device = nullptr;

AttitudeLayerBase::AttitudeLayerBase()
	: m_availableCount(10) {
	m_syncBuffer.resize(10);
	m_frameInfoVector.resize(10);
	for (size_t i = 0; i < m_frameInfoVector.size(); i++) {
		m_frameInfoVector[i].m_frameId = i;
	}
}

GPSPackage & AttitudeLayerBase::MergePackage(GPSPackage & packageMain, const GPSPackage & packageSub) {
	packageMain.m_roll = packageSub.m_roll;
	packageMain.m_vhdtime = packageSub.m_vhdtime;
	return packageMain;
}

bool AttitudeLayerBase::Sync(GPSPackage & package) {
	// std::cout << "availableCount : " << m_availableCount << std::endl;
	// check frame type
	float frameTime = 0.0f;
	GnssEthernetInput::SolveMode frameMode;
	if (package.m_avrtime >= 0.0f) {
		frameTime = package.m_avrtime;
		frameMode = GnssEthernetInput::SolveMode::MAIN;
	} else if (package.m_vhdtime >= 0.0f) {
		frameTime = package.m_vhdtime;
		frameMode = GnssEthernetInput::SolveMode::SUB;
	} else {
		std::cout << "[Sync] time is unavalible." << std::endl;
		return false;
	}
	// sort for time
	// if (frameTime - m_lastFrameTime > 0.051f)
	// {
	//   std::cout << "[Sync] Big cross 20hz" << std::endl;
	//   std::cout << "[Sync] " << std::setprecision(2) << std::setiosflags(std::ios::fixed) << frameTime << " vs. " << m_lastFrameTime << std::endl;
	// }
	if (m_lastFrameTime > frameTime) {
		// std::cout << "[Sync] be late." << std::endl;
		// std::cout << "[Sync] " << frameMode << " vs. " << m_lastFrameMode << std::endl;
		// std::cout << "[Sync] " << std::setprecision(2) << std::setiosflags(std::ios::fixed) << frameTime << " vs. " << m_lastFrameTime << std::endl;
	}
	m_lastFrameTime = frameTime;
	m_lastFrameMode = frameMode;
	// find sync frame
	for (std::vector<FrameInfo>::iterator it = m_frameInfoVector.begin(); it != m_frameInfoVector.end(); it++) {
		if ((!it->m_available) && (it->m_frameTime - frameTime < 0.01f) && (it->m_needMode == frameMode)) {
			if (it->m_needMode == GnssEthernetInput::SolveMode::MAIN) {
				m_syncBuffer[it->m_frameId] = MergePackage(package, m_syncBuffer[it->m_frameId]);
			} else if (it->m_needMode == GnssEthernetInput::SolveMode::SUB) {
				m_syncBuffer[it->m_frameId] = MergePackage(m_syncBuffer[it->m_frameId], package);
			} else {
				std::cout << "Unavailable Mode." << std::endl;
				return false;
			}
			m_syncFrameId = it->m_frameId;
			// NOTE : set m_available
			return true;
		}
	}
	// new frame
	for (std::vector<FrameInfo>::iterator it = m_frameInfoVector.begin(); it != m_frameInfoVector.end(); it++) {
		if (it->m_available) {
			m_syncBuffer[it->m_frameId] = package;
			if (frameMode == GnssEthernetInput::SolveMode::MAIN) {
				it->m_needMode = GnssEthernetInput::SolveMode::SUB;
				it->m_frameTime = package.m_avrtime;
			} else if (frameMode == GnssEthernetInput::SolveMode::SUB) {
				it->m_needMode = GnssEthernetInput::SolveMode::MAIN;
				it->m_frameTime = package.m_vhdtime;
			} else {
				std::cout << "unavailable Mode." << std::endl;
				return false;
			}
			it->m_available = false;
			m_availableCount--;
			return false;
		}
	}
	// we have no space to save package.
	std::cout << "Sync buffer is full." << std::endl;
	return false;
}

TcpAttitudeLayer::TcpAttitudeLayer()
	: m_gpsMain("169.254.1.2:20000", GnssEthernetInput::ConnMode::TCP, GnssEthernetInput::MAIN)
	, m_gpsSub("169.254.1.3:20000", GnssEthernetInput::ConnMode::TCP, GnssEthernetInput::SUB) {
}

bool TcpAttitudeLayer::Capture() {
	do {
		GPSPackage packageMain;
		if (m_gpsMain.Solve(packageMain)) {
			if (Sync(packageMain))
				break;
		}
		GPSPackage packageSub;
		if (m_gpsSub.Solve(packageSub)) {
			if (Sync(packageSub))
				break;
		}
		return false;
	} while (false);
	m_package = m_syncBuffer[m_syncFrameId];
	m_frameInfoVector[m_syncFrameId].m_available = true;
	m_availableCount++;
	return true;
}


//-----TCPAttitudeSolver-----
//-----UDPAttitudeSolver-----
UdpAttitudeLayer::UdpAttitudeLayer()
	: m_gpsMain("169.254.1.1:10000", GnssEthernetInput::ConnMode::UDP, GnssEthernetInput::MAIN) {
}

bool UdpAttitudeLayer::Capture() {
	GPSPackage package;
	if (!m_gpsMain.Solve(package))
		return false;
	if (!Sync(package))
		return false;
	m_package = m_syncBuffer[m_syncFrameId];
	m_frameInfoVector[m_syncFrameId].m_available = true;
	m_availableCount++;
	return true;
}

PcapTransformLayer * PcapTransformLayer::layer = nullptr;

PcapTransformLayer::PcapTransformLayer() {
	angle_piece = 120;
	storage_flag = false;
    path_prefix = "E:/DataSpace/LidarDataSpace/lidar_";
	std::string time_code = TimeUtil::get_time_code();
	//std::string time_code_millsecond = TimeUtil::get_time_code_millsecond();
	root_path = path_prefix + time_code + "/";
}

PcapTransformLayer * PcapTransformLayer::get_instance() {
	if (layer == nullptr) {
		layer = new PcapTransformLayer();
	}
	return layer;
}

PcapTransformLayer::~PcapTransformLayer() {
	if (layer != nullptr) {
		delete layer;
	}
}

void PcapTransformLayer::trans_pcap_to_pcd(std::string pcap_path, std::vector<pcl::PointCloud<PointType>::Ptr> & vec, int seg_count) {
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

			for (int i = 0; i < block_count; i++) {
				//get azimuth
				for (int k = angle_size - 1; k >= 0; k--) {
					int index = head_size + i * block_size + angle_address + k;
					float data = pktdata[head_size + i * block_size + angle_address + k];
					angles[i] = angles[i] * 256 + data;
				}
				angles[i] = angles[i] / 100;
				//printf("%d %f\n", i, angles[i]);
				for (int j = 0; j < channel_count; j++) {
					for (int k = unit_distance_size - 1; k >= 0; k--) {
						distance_mm[i * channel_count + j] = distance_mm[i * channel_count + j] * 256 + pktdata[head_size + flag_size + i * block_size + angle_size + j * channel_size + k];
					}
					//printf("Block %d Channel %d  %d\n", i, j, distance_mm[i * channel_count + j]);
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
					float vertical_angle = PreprocessLayerConfig::hdl32_vertical_angles[j % 32] * PI / 180;
					point.z = distance * sin(vertical_angle);
					//printf("%f\n", horizontal_angle);
					point.y = distance * cos(vertical_angle) * sin(horizontal_angle);
					point.x = distance * cos(vertical_angle) * cos(horizontal_angle);
					PointType pclPoint;
					pclPoint.x = 2 * (point.y);
					pclPoint.y = -2 * (point.x);
					pclPoint.z = 2 * point.z;
					//if (pclPoint.z == 0) {
					//	//printf("%f %f %f\n", pclPoint.x, pclPoint.y, pclPoint.z);
					//}

					pclPoint.r = 0;
					pclPoint.b = flectivity_value;
					pclPoint.g = PreprocessLayerConfig::hdl32_vertical_ids[j % 32];
					if (current_frame_count == seg_count) {
						vec[index_frame_count]->push_back(pclPoint);
					}
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

void PcapTransformLayer::play_pcap_file(std::string pcap_path, int start_packet_number) {
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
	float angle_map[240];//0 1.5 3 4.5
	memset(angle_map, 0, sizeof(float) * 240);
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
			float angle_average = 0;
			for (int i = 0; i < block_count; i++) {
				angle_average += angles[i];
			}
			angle_average /= block_count;
			int angle_index = (int)(angle_average / 1.5);
			if (angle_map[angle_index] == 0) {
				//��ӵ���
				for (int i = 0; i < block_count; i++) {
					for (int j = 0; j < channel_count; j++) {
						MyPoint3D point;
						float distance = distance_mm[i * channel_count + j] / 1000.0;
						int flectivity_value = flectivity[i * channel_count + j];
						float horizontal_angle = angles[i] * PI / 180;
						float vertical_angle = PreprocessLayerConfig::hdl32_vertical_angles[j % 32] * PI / 180;
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
				angle_map[angle_index] = 1;
			}
			//����Ƿ�չ���һȦ
			int angle_count = 0;
			for (int i = 0; i < 240; i++) {
				if (angle_map[i] == 1) {
					angle_count++;
				}
			}
			if (angle_count >= 235) {
				//����  ��ʾ
				float seg = maxFlectivity * 1.0 / 256;
				int pointSize = scene->points.size();
				for (int i = 0; i < scene->points.size(); i++) {
					int flex = scene->points[i].b;
					scene->points[i].r = (int)(flex / seg);
					scene->points[i].b = 255 - (int)(flex / seg) * 5;
					scene->points[i].g = (int)(flex / seg) * 5;
				}
				//PointViewer::get_instance()->set_point_cloud(scene);
				//scene->clear();
				memset(angle_map, 0, sizeof(float) * 240);
			}
			delete angles;
			delete distance_mm;
			delete flectivity;
		}
		if (res == 0) {
			continue;
		}
		static double m_Packet_Count = 0;
#ifdef WIN32
		static DWORD  m_PacketsLen = 0;
		static DWORD  m_TickCount = 0;
#else 
		static long long  m_PacketsLen = 0;
		static long long  m_TickCount = 0;
#endif
		static double m_Speed = 0.0;
#ifdef WIN32
		m_PacketsLen += pkthdr->len;
		m_Packet_Count++;
		if (GetTickCount() - m_TickCount > 1000) {
			m_Speed = m_PacketsLen / 1000.0;
			m_TickCount = GetTickCount();
			m_PacketsLen = 0;
			m_Packet_Count = 0;
		}
#endif
	}
	if (res == -1) {
		printf("Error reading the packets: %s\n", pcap_geterr(adhandle));
		return;
	}
	pcap_close(adhandle);
	return;
}

pcap_t * PcapTransformLayer::get_pcap_file_data(std::string pcap_path) {
	char errbuf[100];
	pcap_t *pfile = pcap_open_offline(pcap_path.c_str(), errbuf);
	if (NULL == pfile) {
		printf("%s\n", errbuf);
		return nullptr;
	}
	return pfile;
}

pcap_t * PcapTransformLayer::get_pcap_dev_handle() {
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
	inum = 1;
	if (inum < 1 || inum > i) {
		printf("\nInterface number out of range.\n");
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

void PcapTransformLayer::parameter_init(float angle_piece, std::string path_prefix) {
	this->angle_piece = angle_piece;
	this->path_prefix = path_prefix;
}

void PcapTransformLayer::get_current_frame(pcl::PointCloud<PointType>::Ptr & scene, int laser_lines_number) {
	if(laser_lines_number != 32 && laser_lines_number != 16){
		return;
	}
	if (device == nullptr) {
		device = get_pcap_dev_handle();
	}
	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;
	bool start_flag = false;
	int count = 0;
	int start_count = 0;
	int maxFlectivity = 0;
	int res = 0;
	float angle_min = 0;
	float angle_max = 0;
	bool start_counting = true;
	float angle_map[240];//0 1.5 3 4.5
	scene->clear();
	memset(angle_map, 0, sizeof(float) * 240);
	while ((res = pcap_next_ex(device, &pkthdr, &pktdata)) >= 0) {
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
			float angle_average = 0;
			for (int i = 0; i < block_count; i++) {
				angle_average += angles[i];
			}
			angle_average /= block_count;
			int angle_index = (int)(angle_average / 1.5);
			if (angle_map[angle_index] == 0) {
				//��ӵ���
				for (int i = 0; i < block_count; i++) {
					for (int j = 0; j < channel_count; j++) {
						MyPoint3D point;
						float distance = distance_mm[i * channel_count + j] / 1000.0;
						int flectivity_value = flectivity[i * channel_count + j];
						float horizontal_angle = angles[i] * PI / 180;
						float vertical_angle = 0;
						if(laser_lines_number == 32){
							vertical_angle = PreprocessLayerConfig::hdl32_vertical_angles[j % 32] * PI / 180;
						}else if(laser_lines_number == 16){
							vertical_angle = PreprocessLayerConfig::vlp16_vertical_angles[j % 32] * PI / 180;
						}	
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
				angle_map[angle_index] = 1;
			}
			//����Ƿ�չ���һȦ
			int angle_count = 0;
			for (int i = 0; i < 240; i++) {
				if (angle_map[i] == 1) {
					angle_count++;
				}
			}
			if (angle_count >= 235) {
				//����  ��ʾ
				float seg = maxFlectivity * 1.0 / 256;
				int pointSize = scene->points.size();
				for (int i = 0; i < scene->points.size(); i++) {
					int flex = scene->points[i].b;
					scene->points[i].r = (int)(flex / seg);
					scene->points[i].b = 255 - (int)(flex / seg) * 5;
					scene->points[i].g = (int)(flex / seg) * 5;
				}
				memset(angle_map, 0, sizeof(float) * 240);
				break;
			}
			delete angles;
			delete distance_mm;
			delete flectivity;
		}
	}
}

void PcapTransformLayer::get_current_frame(const char * path, HPCD & file, int laser_lines_number) {
	if(laser_lines_number != 32 && laser_lines_number != 16){
		return;
	}
	if (device == nullptr) {
		device = get_pcap_dev_handle();
	}
	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;
	bool start_flag = false;
	int count = 0;
	int res = 0;
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
	std::vector<XYZRGBA> points;
	while ((res = pcap_next_ex(device, &pkthdr, &pktdata)) >= 0) {
		if (pkthdr->caplen == 1248) {
			///////////////////////
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
					}
				}
			}

			for (int i = 0; i < block_count; i++) {
				for (int j = 0; j < channel_count; j++) {
					MyPoint3D point;
					float distance = distance_mm[i * channel_count + j] / 1000.0;
					int flectivity_value = flectivity[i * channel_count + j];
					float horizontal_angle = angles[i] * PI / 180;
					
					float vertical_angle = 0;
					if(laser_lines_number == 32){
						vertical_angle = PreprocessLayerConfig::hdl32_vertical_angles[j % 32] * PI / 180;
					}else if(laser_lines_number == 16){
						vertical_angle = PreprocessLayerConfig::vlp16_vertical_angles[j % 32] * PI / 180;
					}	
					point.z = distance * sin(vertical_angle);
					point.y = distance * cos(vertical_angle) * sin(horizontal_angle);
					point.x = distance * cos(vertical_angle) * cos(horizontal_angle);
					XYZRGBA pclPoint;
					pclPoint.x = 2 * (point.y);
					pclPoint.y = -2 * (point.x);
					pclPoint.z = 2 * point.z;
					pclPoint.r = flectivity_value;
					pclPoint.b = 255 - flectivity_value;
					pclPoint.g = flectivity_value * 5;
					pclPoint.a = 0;
					if (!(pclPoint.x <= 0.02 && pclPoint.y <= 0.02 && pclPoint.z <= 0.02)) {
						points.push_back(pclPoint);
					}
				}
			}
			if (count >= 240) {
				count = 0;
				file = PcdUtil::pcdOpen(path);
				PcdUtil::pcdWrite(file, points.data(), points.size());
				PcdUtil::pcdClose(file);
				points.clear();
				break;
			}
			count++;
		}
	}
}

GnssTransformLayer * GnssTransformLayer::layer = nullptr;

GnssTransformLayer::GnssTransformLayer() {
	
}

GnssTransformLayer::~GnssTransformLayer() {
	if (layer != nullptr) {
		delete layer;
	}
}

GnssTransformLayer * GnssTransformLayer::get_instance() {
	if (layer == nullptr) {
		layer = new GnssTransformLayer();
	}
	return layer;
}

void GnssTransformLayer::pre_gps_process(const std::string & gps_folder_path, const std::string& pro_gps_folder_path, int start_data_number, int end_data_number) {
	for (int i = start_data_number; i <= end_data_number; i++) {
		char file_name[100];
		sprintf(file_name, "log%d.txt", i);
		std::string file_name_str = file_name;
		std::string file_path = gps_folder_path + file_name_str;
		std::string out_file_path = pro_gps_folder_path + file_name_str;
		FileUtil file(file_path.c_str(), 0);
		std::string str = "";
		std::string gga_str = "";
		std::string hdt_str = "";
		while (true) {
		//while ((str = file.read_line()).length() > 0) {
			str = file.read_line();
			if (str.find("GGA") != std::string::npos) {
				gga_str = str;
				if (hdt_str.length() > 0) {
					FileUtil out_file(out_file_path.c_str(), 2);
					out_file.write_line(gga_str.substr(0, 89), true);
					out_file.write_line(hdt_str.substr(0, 66), false);
					break;
				}
			} else if (str.find("AVR") != std::string::npos) {
				hdt_str = str;
				if (gga_str.length() > 0) {
					FileUtil out_file(out_file_path.c_str(), 2);
					out_file.write_line(gga_str.substr(0, 89), true);
					out_file.write_line(hdt_str.substr(0, 66), false);
					break;
				}
			}
		}
	}
}

GPHDT_Data GnssTransformLayer::decodeGPHDT(std::string gphdt_msg) {
	GPHDT_Data data;
	std::vector<std::string> vec;
	vec.clear();
	std::string delim = ",";
	auto split = [](std::string& s, std::string& delim, std::vector<std::string>* ret) {
		size_t last = 0;
		size_t index = s.find_first_of(delim, last);
		while (index != std::string::npos) {
			ret->push_back(s.substr(last, index - last));
			last = index + 1;
			index = s.find_first_of(delim, last);
		}
		if (index - last>0) {
			ret->push_back(s.substr(last, index - last));
		}
	};
	split(gphdt_msg, delim, &vec);
	if (vec.size() < 3) {
		data.yaw = 0;
	} else if (strcmp(vec[0].c_str(), "$GPHDT") == 0 || strcmp(vec[0].c_str(), "$GNHDT") == 0) {
		data.yaw = atof(vec[1].c_str());
	} else {

	}
	return data;
}

GPGGA_Data GnssTransformLayer::decodeGPGGA(std::string gpgga_msg) {
	GPGGA_Data data;
	std::vector<std::string> vec;
	vec.clear();
	std::string delim = ",";
	auto split = [](std::string& s, std::string& delim, std::vector<std::string>* ret) {
		size_t last = 0;
		size_t index = s.find_first_of(delim, last);
		while (index != std::string::npos) {
			ret->push_back(s.substr(last, index - last));
			last = index + 1;
			index = s.find_first_of(delim, last);
		}
		if (index - last>0) {
			ret->push_back(s.substr(last, index - last));
		}
	};
	split(gpgga_msg, delim, &vec);
	if (vec.size() < 7) {
		data.lat = 0;
		data.lon = 0;
	} else if (strcmp(vec[0].c_str(), "$GPGGA") == 0 || strcmp(vec[0].c_str(), "$GNGGA") == 0) {
		data.lat = atof(vec[2].c_str());
		data.lat_dir = vec[3].c_str();
		data.lon = atof(vec[4].c_str());
		data.lon_dir = vec[5].c_str();
		data.state = atoi(vec[6].c_str());
	} else {

	}
	return data;
}

PTNLAVR_Data GnssTransformLayer::decodePTNLAVR(std::string ptnlavr_msg) {
	PTNLAVR_Data data;
	std::vector<std::string> vec;
	vec.clear();
	std::string delim = ",";
	auto split = [](std::string& s, std::string& delim, std::vector<std::string>* ret) {
		size_t last = 0;
		size_t index = s.find_first_of(delim, last);
		while (index != std::string::npos) {
			ret->push_back(s.substr(last, index - last));
			last = index + 1;
			index = s.find_first_of(delim, last);
		}
		if (index - last>0) {
			ret->push_back(s.substr(last, index - last));
		}
	};
	split(ptnlavr_msg, delim, &vec);
	if (vec.size() < 10) {
		data.yaw = 0;
	} else if (strcmp(vec[0].c_str(), "$PTNL") == 0) {
		std::string str = vec[3].c_str();
		str = str.substr(1);
		data.yaw = atof(str.c_str());
	} else {

	}
	return data;
}

double GnssTransformLayer::DeltaLat(const DMS & base, const DMS & dest) {
	double distance = (dest.dd - base.dd) * 111000.0f;
	distance += (dest.mm - base.mm) * 1850.0f;
	distance += (dest.ss - base.ss) * 30.9f;
	return distance;
}

double GnssTransformLayer::DeltaLon(const DMS & base, const DMS & dest) {
	double distance = (dest.dd - base.dd) * 85390.0f;
	distance += (dest.mm - base.mm) * 1420.0f;
	distance += (dest.ss - base.ss) * 23.6f;
	return distance;
}

Vec2d GnssTransformLayer::get_distance1(double latDest, double lngDest, double latOrg, double lngOrg) {
	Vec2d vec;
	DMS latDestDms(latDest);
	DMS lngDestDms(lngDest);
	DMS latOrgDms(latOrg);
	DMS lngOrgDms(lngOrg);
	vec.x = DeltaLon(lngOrgDms, lngDestDms);
	vec.y = DeltaLat(latOrgDms, latDestDms);
	return vec;
}