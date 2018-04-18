#pragma once

#include "PreprocessLayerConfig.h"

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

		class PcapTransformLayer {
		public:
			static PcapTransformLayer * get_instance();
			void trans_pcap_to_pcd(std::string pcap_path, std::vector<pcl::PointCloud<PointType>::Ptr> & vec, int seg_count = 0);
			void play_pcap_file(std::string pcap_path, int start_packet_number = 0);
			void get_current_frame(pcl::PointCloud<PointType>::Ptr & cloud, int laser_lines_number);
			void get_current_frame(const char * path, HPCD & file, int laser_lines_number);
			void parameter_init(float angle_piece, std::string path_prefix);
			~PcapTransformLayer();
		private:
			static pcap_t * device;
			PcapTransformLayer();
			bool storage_flag;
			float angle_piece;
			std::string path_prefix;
			std::string root_path;
			pcap_t * get_pcap_file_data(std::string pcap_path);
			pcap_t * get_pcap_dev_handle();
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