#include "gnssprocesser.h"

using namespace mammoth::util;
using namespace mammoth::io;

AttitudeBase::AttitudeBase()
	: m_availableCount(10) {
	m_syncBuffer.resize(10);
	m_frameInfoVector.resize(10);
	for (size_t i = 0; i < m_frameInfoVector.size(); i++) {
		m_frameInfoVector[i].m_frameId = i;
	}
}

GPSPackage & AttitudeBase::MergePackage(GPSPackage & packageMain, const GPSPackage & packageSub) {
	packageMain.m_roll = packageSub.m_roll;
	packageMain.m_vhdtime = packageSub.m_vhdtime;
	return packageMain;
}

bool AttitudeBase::Sync(GPSPackage & package) {
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

TcpAttitude::TcpAttitude()
	: m_gpsMain("169.254.1.2:20000", GnssEthernetInput::ConnMode::TCP, GnssEthernetInput::MAIN)
	, m_gpsSub("169.254.1.3:20000", GnssEthernetInput::ConnMode::TCP, GnssEthernetInput::SUB) {
}

bool TcpAttitude::Capture() {
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
UdpAttitude::UdpAttitude()
	: m_gpsMain("169.254.1.1:10000", GnssEthernetInput::ConnMode::UDP, GnssEthernetInput::MAIN) {
}

bool UdpAttitude::Capture() {
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





GnssProcesser * GnssProcesser::layer = nullptr;

GnssProcesser::GnssProcesser() {
	
}

GnssProcesser::~GnssProcesser() {
	if (layer != nullptr) {
		delete layer;
	}
}

GnssProcesser * GnssProcesser::get_instance() {
	if (layer == nullptr) {
		layer = new GnssProcesser();
	}
	return layer;
}

void GnssProcesser::pre_gps_process(const std::string & gps_folder_path, const std::string& pro_gps_folder_path, int start_data_number, int end_data_number) {
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

GPHDT_Data GnssProcesser::decodeGPHDT(std::string gphdt_msg) {
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

GPGGA_Data GnssProcesser::decodeGPGGA(std::string gpgga_msg) {
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

PTNLAVR_Data GnssProcesser::decodePTNLAVR(std::string ptnlavr_msg) {
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

double GnssProcesser::DeltaLat(const DMS & base, const DMS & dest) {
	double distance = (dest.dd - base.dd) * 111000.0f;
	distance += (dest.mm - base.mm) * 1850.0f;
	distance += (dest.ss - base.ss) * 30.9f;
	return distance;
}

double GnssProcesser::DeltaLon(const DMS & base, const DMS & dest) {
	double distance = (dest.dd - base.dd) * 85390.0f;
	distance += (dest.mm - base.mm) * 1420.0f;
	distance += (dest.ss - base.ss) * 23.6f;
	return distance;
}

Vec2d GnssProcesser::get_distance1(double latDest, double lngDest, double latOrg, double lngOrg) {
	Vec2d vec;
	DMS latDestDms(latDest);
	DMS lngDestDms(lngDest);
	DMS latOrgDms(latOrg);
	DMS lngOrgDms(lngOrg);
	vec.x = DeltaLon(lngOrgDms, lngDestDms);
	vec.y = DeltaLat(latOrgDms, latDestDms);
	return vec;
}