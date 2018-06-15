#pragma once

#include "InputLayerConfig.h"

namespace mammoth {
	namespace layer {
		class GyroscopeSerialInput {
		public:
			//static void start(std::string serial_number, int baud_rate);
			static void startAsync(std::string serial_number, int baud_rate, float* buffer, int buffer_size);
			static void startSync(std::string serial_number, int baud_rate, float* buffer, int buffer_size);
			static int readSync(char * buffer, int buffer_size);
			static int packet_count0;
			static int packet_count1;
			static int packet_count2;
			static int packet_count3;
			static void stop();
			static void decode(std::string const &data);
			static void read_gyroscope(char * , int);
			static bool has_angle_data;
		private:
			static std::string char_to_hex(std::string const &string);
			static unsigned char hex_to_char(std::string const &string);
			
			static void decode_imu_data();
			static void read_gyroscope(const std::string& real_data, void* context);
			static unsigned char temp[11];
			static int temp_count;
			static double a[3], w[3], Angle[3], T;
			static int cmd_count;
			static int gyroscope_flag;
			
			static float* buffer;
			static int buffer_size;

			

			static SyncCom sync_com;

		};


		struct GPSPackage {
			GPSPackage()
				: m_yaw(0.0f)
				, m_pitch(0.0f)
				, m_roll(0.0f)
				, m_avrtime(-1.0f)
				, m_vhdtime(-1.0f)
				, m_latitude(0.0f)
				, m_latitudeDirection('N')
				, m_longitude(0.0f)
				, m_longitudeDirection('E')
				, m_elevation(0.0f)
				, m_gpsQuality(0)
				, m_yawErr(0.0f)
				, m_latitudeErr(0.0f)
				, m_longitudeErr(0.0f)
				, m_elevationErr(0.0f) {
			}
			// Solved Data
			float m_yaw;                //.xxxxnnn
			float m_pitch;              //.xxxxnnn
			float m_roll;               //.xxxnnn
			float m_avrtime;            //.xxnn
			float m_vhdtime;            //.xxnn
			double m_latitude;          //.xxxxxxxxnnnnn
			char m_latitudeDirection;   //N S
			double m_longitude;         //.xxxxxxxxnnnnn
			char m_longitudeDirection;  //E W
			float m_elevation;          //.xxxnnn
			unsigned char m_gpsQuality; //0 1 2 3 4
			float m_yawErr;             //.x
			float m_latitudeErr;        //.xxxnnn
			float m_longitudeErr;       //.xxxnnn
			float m_elevationErr;       //.xxxnnn
			void Output() {
				std::cout << "yaw / pitch / roll               : [" << std::setprecision(4) << std::setiosflags(std::ios::fixed) << m_yaw << " , " << m_pitch << " , " << m_roll << "]" << std::endl;
				std::cout << "latitude / longitude / elevation : [" << std::setprecision(8) << std::setiosflags(std::ios::fixed) << m_latitude << " " << m_latitudeDirection << " , " << m_longitude << " " << m_longitudeDirection << " , " << m_elevation << "]" << std::endl;
				std::cout << "yErr / laErr / loErr / elErr     : [" << std::setprecision(3) << std::setiosflags(std::ios::fixed) << m_yawErr << " , " << m_latitudeErr << " , " << m_longitudeErr << " , " << m_elevationErr << "]" << std::endl;
				std::cout << "Quality                          : [" << m_gpsQuality << "]" << std::endl;
			}
		};

		class GnssEthernetInput {
		public:
			enum ConnMode {
				SERIAL,
				UDP,
				TCP
			};
			enum SolveMode {
				MAIN,
				SUB
			};
			GnssEthernetInput(const std::string& entry, const ConnMode& connMode = ConnMode::UDP, const SolveMode& solveMode = SolveMode::MAIN);
			bool Solve(GPSPackage& package);
			// Solved Data
			float m_yaw;
			float m_pitch;
			float m_roll;
			float m_avrtime;
			float m_vhdtime;
			float m_latitude;
			char m_latitudeDirection;
			float m_longitude;
			char m_longitudeDirection;
			float m_elevation;
			unsigned char m_gpsQuality;
			float m_yawErr;
			float m_latitudeErr;
			float m_longitudeErr;
			float m_elevationErr;
		private:
			// Solve Mode
			ConnMode m_connMode;
			SolveMode m_solveMode;

			// Solve Proc
			bool _Accept(std::string& msg);
			bool _Segment(const std::string& msg, std::vector<std::string>& segment);
			bool _Phrase(const std::string& msg, std::vector<std::string>& token);
			std::string _Identify(std::vector<std::string>& tokens);
			bool _Adapt(std::vector<std::string>& token, GPSPackage& package, const std::string& msgType);

			// Solve format
			bool _AVRSolver(std::vector<std::string>& token, GPSPackage& package);
			bool _GGASolver(std::vector<std::string>& token, GPSPackage& package);
			bool _VHDSolver(std::vector<std::string>& token, GPSPackage& package);
			bool _GSTSolver(std::vector<std::string>& token, GPSPackage& package);
			// internal data
			std::vector<std::string> m_header;
			std::vector<std::string> m_ptnlHeader;
			SolverFunctionMap m_solverFunctionMap;

			// Data entry
			TcpClient m_tcpClient;
			UdpServer m_udpServer;
		};
	
		struct ImuPackage {
			ImuPackage() {}
			double m_gyroX;
			double m_gyroY;
			double m_gyroZ;
			double m_acclX;
			double m_acclY;
			double m_acclZ;
			double m_deltaAngleX;
			double m_deltaAngleY;
			double m_deltaAngleZ;
			double m_deltaVelX;
			double m_deltaVelY;
			double m_deltaVelZ;
			double m_angleX;
			double m_angleY;
			double m_angleZ;
			double m_maginX;
			double m_maginY;
			double m_maginZ;
			void Output() {
				std::cout << "Gyro : [ " << std::setprecision(10) << std::setiosflags(std::ios::fixed) << m_gyroX << " , " << m_gyroY << " , " << m_gyroZ << " ]\n";
				std::cout << "Accl : [ " << std::setprecision(10) << std::setiosflags(std::ios::fixed) << m_acclX << " , " << m_acclY << " , " << m_acclZ << " ]\n";
				// if (m_deltaAngleX > 0.03f || m_deltaAngleX < -0.03f)
				//   m_angleX += m_deltaAngleX;
				// if (m_deltaAngleY > 0.03f || m_deltaAngleY < -0.03f)
				//   m_angleY += m_deltaAngleY;
				// if (m_deltaAngleZ > 0.03f || m_deltaAngleZ < -0.03f)
				//   m_angleZ += m_deltaAngleZ;
				// std::cout << "deltaAngle : [ " << std::setprecision(10) << std::setiosflags(std::ios::fixed) << m_deltaAngleX << ", " << m_deltaAngleY << ", " << m_deltaAngleZ << " ]" << std::endl;

				//std::cout << "Angle      : [ " << std::setprecision(10) << std::setiosflags(std::ios::fixed) << m_angleX << ", " << m_angleY << ", " << m_angleZ << " ]" << std::endl;
				// std::cout << "latitude / longitude / elevation : [" << std::setprecision(8) << std::setiosflags(std::ios::fixed) << m_latitude << " " << m_latitudeDirection << ", " << m_longitude << " " << m_longitudeDirection << "]" << std::endl;
				// std::cout << "yErr / laErr / loErr / elErr     : [" << std::setprecision(3) << std::setiosflags(std::ios::fixed) << m_yawErr << ", " << m_latitudeErr << ", " << m_longitudeErr << ", " << m_elevationErr << "]" << std::endl;
				// std::cout << "Quality                          : [" << m_gpsQuality << "]" << std::endl;
			}
		};

		class ImuSolver {
		public:
			ImuSolver() {}
			bool Solve(std::vector<unsigned char> & msg);
			ImuPackage m_imuPackage;
		private:
			
			bool _Phrase(const std::string& msg, std::vector<std::string>& token);

			bool _GyroSolver(short lowHex, short highHex, double & value);
			bool _AcclSolver(short lowHex, short highHex, double & value);
			bool _AngleSolver(short lowHex, short highHex, double & value);
			bool _VelSolver(short lowHex, short highHex, double & value);
			bool _MaginSolver(short highHex, double & value);

		};
	}
}