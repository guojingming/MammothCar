#pragma once

#include "UnionConfig.h"

#include "PcdUtilLayerConfig.h"

#ifdef WIN32
#include "pcap.h"
#include <windows.h>
#include <omp.h>
#include <direct.h>  
#include <io.h>
#endif

#include "StdUtilLayer.h"
#include "InputLayer.h"
#include "PcdUtilLayer.h"

namespace mammoth {
	namespace config {
		class PreprocessLayerConfig {
		public:
			static double * hdl32_vertical_angles;
			static uint8_t * hdl32_vertical_ids;
			static double * vlp16_vertical_angles;
			static uint8_t * vlp16_vertical_ids;
			static double * pandar40P_vertical_angles;
			static double * pandar40P_horizontal_angles;
			static double *banewakeCE30D_vertical_angles;
			// static double * rslidar16_vertical_angles;
			// static double * rslidar16_horizontal_angles;
		};

		//enum PRESET_CONFIGS {VLP16, HDL32E, PANDAR40P};
	}
}
namespace mammoth {
	namespace layer {
		class AttitudeLayerBase {
		public:
			AttitudeLayerBase();
			virtual bool Capture() = 0;
			GPSPackage m_package;
		protected:
			std::vector<GPSPackage> m_syncBuffer;
			GPSPackage & MergePackage(GPSPackage & packageMain, const GPSPackage & packageSub);
			virtual bool Sync(GPSPackage & package);
			size_t m_syncFrameId;
			struct FrameInfo {
				FrameInfo() : m_frameId(0), m_available(true), m_frameTime(0.0f) {}
				size_t m_frameId;
				bool m_available;
				float m_frameTime;
				GnssEthernetInput::SolveMode m_needMode;
			};
			std::vector<FrameInfo> m_frameInfoVector;
			int m_availableCount;
			float m_lastFrameTime;
			GnssEthernetInput::SolveMode m_lastFrameMode;
		};

		// TODO : add tcp attitudeSolver
		class TcpAttitudeLayer : public AttitudeLayerBase {
		public:
			TcpAttitudeLayer();
			virtual bool Capture();
		private:
			GnssEthernetInput m_gpsMain;
			GnssEthernetInput m_gpsSub;
		};

		class UdpAttitudeLayer : public AttitudeLayerBase {
		public:
			UdpAttitudeLayer();
			virtual bool Capture();

		private:
			GnssEthernetInput m_gpsMain;
		};

		class PcdTransformLayer{
		public:
			static PcdTransformLayer * get_instance();
			void rotation(pcl::PointCloud<PointType>::Ptr & cloud, float x, float y, float z);
			void translation(pcl::PointCloud<PointType>::Ptr & cloud, float x, float y, float z);
			~PcdTransformLayer();
			void combine(pcl::PointCloud<PointType>::Ptr & cloud1, pcl::PointCloud<PointType>::Ptr & cloud2, pcl::PointCloud<PointType>::Ptr & combine_cloud);
		private:
			static PcdTransformLayer * layer;
			PcdTransformLayer();
		};

		class PcapTransformLayer {
		public:
			static PcapTransformLayer * get_instance();
			pcap_t * get_pcap_dev_handle(int ethernet_number);
			pcap_t * get_pcap_file_data(std::string pcap_path);
			pcap_t * get_pcap_dev_handle();
			void trans_pcap_to_pcd(std::string pcap_path, std::vector<pcl::PointCloud<PointType>::Ptr> & vec, int seg_count = 0);
			void play_pcap_file(std::string pcap_path, int start_packet_number = 0);
			void get_current_frame(pcap_t * cur_device, pcl::PointCloud<PointType>::Ptr & scene, int config);
			void get_current_frame_pandar(const char * path, HPCD & file);
			void get_current_frame_CE30D(cv::Mat & img, pcap_t * cur_device, pcl::PointCloud<PointType>::Ptr & scene, int config);
			void parameter_init(float angle_piece, std::string path_prefix);
			~PcapTransformLayer();
		private:
			static pcap_t * device;
			PcapTransformLayer();
			bool storage_flag;
			float angle_piece;
			std::string path_prefix;
			std::string root_path;
			static PcapTransformLayer * layer;
		};

		typedef struct {
			//jingdu
			double lon;
			std::string lon_dir;  // E  or  W
			//weidu
			double lat;
			std::string lat_dir;  // S  or  N
			int state;
		}GPGGA_Data;

		typedef struct {
			double yaw;
			int state;
		}GPHDT_Data;

		typedef struct {
			double yaw;
		}PTNLAVR_Data;

		struct Vec2d {
			double x;
			double y;
		};

		struct DMS {
			DMS(double dm) {
				dd = (int)dm / 100;
				mm = (int)dm - 100 * dd;
				ss = (dm - 100 * dd - mm) * 60.0f;
			}
			int dd;
			int mm;
			double ss;
		};

		class GnssTransformLayer {
		public:
			static GnssTransformLayer * get_instance();
			~GnssTransformLayer();
			void pre_gps_process(const std::string & gps_folder_path, const std::string& pro_gps_folder_path, int start_data_number, int end_data_number);
			GPGGA_Data decodeGPGGA(std::string gpgga_msg);
			GPHDT_Data decodeGPHDT(std::string gphdt_msg);
			PTNLAVR_Data decodePTNLAVR(std::string ptnlavr_msg);
			Vec2d get_distance1(double latDest, double lngDest, double latOrg, double lngOrg);
		private:
			GnssTransformLayer();
			double DeltaLat(const DMS & base, const DMS & dest);
			double DeltaLon(const DMS & base, const DMS & dest);
			

			static GnssTransformLayer * layer;

		};


		template<typename T>
		class DataBuffer {
		public:
			DataBuffer(const std::vector<T> & splits);

			bool Charge(std::vector<T> & elems);
			bool Charge(const T * elems, const size_t & len);
			bool Discharge(std::vector<std::vector<T> > & elemVectors);
		private:
			bool ChargeElem(const T & elem);

			std::vector<T> m_splitElements;
			size_t m_splitId;

			std::vector<std::vector<T> > m_bufferVector;
			size_t m_chargingVectorId;
			std::vector<size_t> m_chargedIdVector;
			std::vector<size_t> m_dischargedIdVector;
		};

		template<typename T>
		DataBuffer<T>::DataBuffer(const std::vector<T> & splits)
			: m_splitElements(splits)
			, m_chargingVectorId(0)
			, m_splitId(0) {
			m_bufferVector.resize(1);
		}

		template<typename T>
		bool DataBuffer<T>::Charge(std::vector<T> & elems) {

			for (size_t i = 0; i < elems.size(); i++) {
				ChargeElem(elems[i]);
			}
			return true;
		}

		template<typename T>
		bool DataBuffer<T>::Charge(const T * elems, const size_t & len) {
			for (size_t i = 0; i < len; i++) {
				ChargeElem(elems[i]);
			}
			return true;
		}

		template<typename T>
		bool DataBuffer<T>::ChargeElem(const T & elem) {
			m_bufferVector[m_chargingVectorId].push_back(elem);

			if (elem == m_splitElements[m_splitId]) {
				m_splitId++;
				if (m_splitId == m_splitElements.size()) {
					m_splitId = 0;
					m_chargedIdVector.push_back(m_chargingVectorId);
					if (m_dischargedIdVector.empty()) {
						m_bufferVector.push_back(std::vector<T>());
						m_chargingVectorId = m_bufferVector.size() - 1;
					} else {
						m_chargingVectorId = *(m_dischargedIdVector.end() - 1);
						m_dischargedIdVector.pop_back();
						m_bufferVector[m_chargingVectorId].clear();
					}
				}
			} else {
				m_splitId = 0;
			}
			return true;
		}

		template<typename T>
		bool DataBuffer<T>::Discharge(std::vector<std::vector<T> > & elemVectors) {
			elemVectors.clear();
			for (std::vector<size_t>::iterator it = m_chargedIdVector.begin(); it != m_chargedIdVector.end(); it++) {
				elemVectors.push_back(m_bufferVector[*it]);
			}
			m_dischargedIdVector.insert(m_dischargedIdVector.end(), m_chargedIdVector.begin(), m_chargedIdVector.end());
			m_chargedIdVector.clear();
			return true;
		}

	}
}


double * PreprocessLayerConfig::hdl32_vertical_angles = new double[32]{
	-30.67,
	-9.33,
	-29.33,
	-8.00,
	-28.00,
	-6.66,
	-26.66,
	-5.33,
	-25.33,
	-4.00,
	-24.00,
	-2.67,
	-22.67,
	-1.33,
	-21.33,
	0,
	-20,
	1.33,
	-18.67,
	2.67,
	-17.33,
	4,
	-16,
	5.33,
	-14.67,
	6.67,
	-13.33,
	8,
	-12,
	9.33,
	-10.67,
	10.67
};

uint8_t * PreprocessLayerConfig::hdl32_vertical_ids = new uint8_t[32] {
	-23 * -1 + 9,
	-7 * -1 + 9,
	-22 * -1 + 9,
	-6 * -1 + 9,
	-21 * -1 + 9,
	-5 * -1 + 9,
	-20 * -1 + 9,
	-4 * -1 + 9,
	-19 * -1 + 9,
	-3 * -1 + 9,
	-18 * -1 + 9,
	-2 * -1 + 9,
	-17 * -1 + 9,
	-1 * -1 + 9,
	-16 * -1 + 9,
	0 * -1 + 9,
	-15 * -1 + 9,
	1 * -1 + 9,
	-14 * -1 + 9,
	2 * -1 + 9,
	-13 * -1 + 9,
	3 * -1 + 9,
	-12 * -1 + 9,
	4 * -1 + 9,
	-11 * -1 + 9,
	5 * -1 + 9,
	-10 * -1 + 9,
	6 * -1 + 9,
	-9 * -1 + 9,
	7 * -1 + 9,
	-8 * -1 + 9,
	8 * -1 + 9
};


double * PreprocessLayerConfig::vlp16_vertical_angles = new double[32]{
	-15,
	1,
	-13,
	3,
	-11,
	5,
	-9,
	7,
	-7,
	9,
	-5,
	11,
	-3,
	13,
	-1,
	15,
	-15,
	1,
	-13,
	3,
	-11,
	5,
	-9,
	7,
	-7,
	9,
	-5,
	11,
	-3,
	13,
	-1,
	15
};

uint8_t * PreprocessLayerConfig::vlp16_vertical_ids = new uint8_t[32]{
	0,
	2,
	4,
	6,
	8,
	10,
	12,
	14,
	1,
	3,
	5,
	7,
	9,
	11,
	13,
	15,
	0,
	2,
	4,
	6,
	8,
	10,
	12,
	14,
	1,
	3,
	5,
	7,
	9,
	11,
	13,
	15
};


double * PreprocessLayerConfig::pandar40P_horizontal_angles = new double[40]{
	0, 0, 0, 0, -2.5, -2.5, 2.5, -5,
	-2.5, 2.5, -5, -2.5, 2.5, -5, 0, 2.5,
	-5, 0, 5, -2.5, 0, 5, -2.5, 0,
	5, -2.5, 2.5, 5, -2.5, 2.5, 2.5, 2.5,
	0, 0, 0, 0, -2.5, -2.5, -2.5, -2.5
};

double * PreprocessLayerConfig::pandar40P_vertical_angles = new double[40]{
	7,6,5,4,3,2,1.67,1.33,1,0.67,
	0.33,0,-0.33,-0.67,-1,-1.33,-1.67,-2,-2.33,-2.67,
	-3,-3.33,-3.67,-4,-4.33,-4.67,-5,-5.33,-5.67,-6,
	-7,-8,-9,-10,-11,-12,-13,-14,-15,-16
};

// double * rslidar16_horizontal_angles = new double[32]{
// 	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
// };

// double * rslidar16_vertical_angles = new double[32]{
// 	-15,-13,-11,-9,-7,-5,-3,-1,15,13,11,9,7,5,3,1,-15,-13,-11,-9,-7,-5,-3,-1,15,13,11,9,7,5,3,1
// };

double *PreprocessLayerConfig::banewakeCE30D_vertical_angles = new double[20]{
	1.9,
	1.7,
	1.5,
	1.3,
	1.1,
	0.9,
	0.7,
	0.5,
	0.3,
	0.1,
	-0.1,
	-0.3,
	-0.5,
	-0.7,
	-0.9,
	-1.1,
	-1.3,
	-1.5,
	-1.7,
	-1.9
};