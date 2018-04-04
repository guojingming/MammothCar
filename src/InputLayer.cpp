#include "InputLayer.h"

using namespace mammoth::layer;
using namespace mammoth::config;

unsigned char GyroscopeSerialInput::temp[11] = {0};
int GyroscopeSerialInput::temp_count = 0;
double GyroscopeSerialInput::a[3] = {0};
double GyroscopeSerialInput::w[3] = {0};
double GyroscopeSerialInput::Angle[3] = {0};
double GyroscopeSerialInput::T = 0;
float* GyroscopeSerialInput::buffer = nullptr;
int GyroscopeSerialInput::buffer_size = -1;
int GyroscopeSerialInput::cmd_count = 0;
int GyroscopeSerialInput::gyroscope_flag = 0;
SyncCom GyroscopeSerialInput::sync_com;
bool GyroscopeSerialInput::has_angle_data = false;

int GyroscopeSerialInput::packet_count0 = 0;
int GyroscopeSerialInput::packet_count1 = 0;
int GyroscopeSerialInput::packet_count2 = 0;
int GyroscopeSerialInput::packet_count3 = 0;

void GyroscopeSerialInput::startAsync(std::string serial_number, int baud_rate, float* buffer, int buffer_size) {
	GyroscopeSerialInput::buffer = buffer;
	GyroscopeSerialInput::buffer_size = buffer_size;
	SerialUtil::openAsync(serial_number.c_str(), baud_rate, read_gyroscope);
}

void GyroscopeSerialInput::startSync(std::string serial_number, int baud_rate, float* buffer, int buffer_size) {
	GyroscopeSerialInput::buffer = buffer;
	GyroscopeSerialInput::buffer_size = buffer_size;
	sync_com = SerialUtil::openSync(serial_number.c_str(), baud_rate);
}

int GyroscopeSerialInput::readSync(char * buffer, int buffer_size) {
	int count = sync_com.Read(buffer, buffer_size);
	for (int i = 0; i < count; i++) {
		if (buffer[i] == 0x53) {
			packet_count3++;
		}
	}

	return count;
}

void GyroscopeSerialInput::stop() {

}

std::string GyroscopeSerialInput::char_to_hex(std::string const &string) {
	std::string ret;
	//printf("s:%s\n",s.c_str());
	for (unsigned i = 0; i != string.size(); ++i) {
		char hex[5];
		sprintf(hex, "%#.2x", (unsigned char)string[i]);
		ret += hex;
	}
	//printf("ret:%s\n",ret.c_str());
	return ret;
}

unsigned char GyroscopeSerialInput::hex_to_char(std::string const &string) {
	short high = 0;
	short low = 0;
	unsigned char res = 0;
	//printf("a : %s\n",s.c_str());
	switch (string.c_str()[2]) {
		case '0':high = 0; break;
		case '1':high = 16; break;
		case '2':high = 32; break;
		case '3':high = 48; break;
		case '4':high = 64; break;
		case '5':high = 80; break;
		case '6':high = 96; break;
		case '7':high = 112; break;
		case '8':high = 128; break;
		case '9':high = 144; break;
		case 'a':high = 160; break;
		case 'b':high = 176; break;
		case 'c':high = 192; break;
		case 'd':high = 208; break;
		case 'e':high = 224; break;
		case 'f':high = 240; break;
		default:high = 0; break;
	}
	switch (string.c_str()[3]) {
		case '0':low = 0; break;
		case '1':low = 1; break;
		case '2':low = 2; break;
		case '3':low = 3; break;
		case '4':low = 4; break;
		case '5':low = 5; break;
		case '6':low = 6; break;
		case '7':low = 7; break;
		case '8':low = 8; break;
		case '9':low = 9; break;
		case 'a':low = 10; break;
		case 'b':low = 11; break;
		case 'c':low = 12; break;
		case 'd':low = 13; break;
		case 'e':low = 14; break;
		case 'f':low = 15; break;
		default:low = 0; break;
	}
	res = low + high;
	return res;
}

void GyroscopeSerialInput::decode(std::string const &data) {
	std::string str = char_to_hex(data);
	//printf("no:%d  %s\n", sum_count, str.c_str());
	if (strcmp(str.c_str(), "00") == 0) {
		//return;
	}
	if (strcmp(str.c_str(), "0x55") == 0) {
		cmd_count = 0;
		for (int i = 0; i<11; i++) {
			temp[i] = 0;
		}
		gyroscope_flag = 1;
		temp[0] = hex_to_char("0x55");
		//printf("sum:%d cmd:%d temp[%d]:%s\n",sum_count,cmd_count,cmd_count,str.c_str());
		return;
	}
	if (gyroscope_flag == 1) {
		//printf("%d\n", cmd_count);
		cmd_count++;
		temp[cmd_count] = hex_to_char(str);
		//printf("sum:%d cmd:%d temp[%d]:%s\n",sum_count,cmd_count,cmd_count,str.c_str());
		//if(cmd_count == 1 && (temp[cmd_count] == 0x51 || temp[cmd_count] == 0x52)){
		//	gyroscope_flag = 0;
		//	return;
		//}
		//printf("%d\n", hex2char(char2hex(str)));
		if (cmd_count == 10) {
			if (temp[1] == 0x53) {
				int sum_check = 0;
				for (int i = 0; i<11; i++) {
					//printf(" %d", temp[i]);
					//sum_check += temp[i];
				}
				//printf("  check: %d", sum_check);
				//printf("\n");
			}

			decode_imu_data();
			for (int i = 0; i<11; i++) {
				temp[i] = 0;
			}
			cmd_count = 0;
		}
	}
}

void GyroscopeSerialInput::decode_imu_data() {
	switch (temp[1]) {
	case 0x51:
		//printf("0x51\n");
		packet_count0++;
		a[0] = (short(temp[3] << 8 | temp[2])) / 32768.0 * 16;
		a[1] = (short(temp[5] << 8 | temp[4])) / 32768.0 * 16;
		a[2] = (short(temp[7] << 8 | temp[6])) / 32768.0 * 16;
		//T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
	    //printf("a = %4.3f\t%4.3f\t%4.3f\t\r\n",a[0],a[1],a[2]);
		break;
	case 0x52:
		//printf("0x52\n");
		packet_count1++;
		w[0] = (short(temp[3] << 8 | temp[2])) / 32768.0 * 2000;
		w[1] = (short(temp[5] << 8 | temp[4])) / 32768.0 * 2000;
		w[2] = (short(temp[7] << 8 | temp[6])) / 32768.0 * 2000;
		//T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
		//printf("w = %4.3f\t%4.3f\t%4.3f\t\r\n",w[0],w[1],w[2]);
		break;
	case 0x53:{
		packet_count2++;
		//printf("0x53xxxxxxxxxxxxxxxxxxxxxxxxx\n");
		Angle[0] = (short(temp[3] << 8 | temp[2])) / 32768.0 * 180;
		Angle[1] = (short(temp[5] << 8 | temp[4])) / 32768.0 * 180;
		Angle[2] = (short(temp[7] << 8 | temp[6])) / 32768.0 * 180;
		float transed_angle = 0;
		if (Angle[2] < 0) {
			transed_angle = -1 * Angle[2];
		} else {
			transed_angle = 360 - Angle[2];
		}
		for (int i = 0; i < buffer_size; i++) {
			switch (i) {
			case 0: buffer[i] = (Angle[0] + InputLayerConfig::gyroscope_x_diff) * InputLayerConfig::gyroscope_x_factor; break;
			case 1: buffer[i] = (Angle[1] + InputLayerConfig::gyroscope_y_diff) * InputLayerConfig::gyroscope_y_factor; break;
			case 2: buffer[i] = (Angle[2] + InputLayerConfig::gyroscope_z_diff) * InputLayerConfig::gyroscope_z_factor; break;
			}
		}
		has_angle_data = true;
		break;
	}
	default:;
		//printf("???????????????\n");
	}

	//printf("\r %f %f %f %f %f %f %f %f %f", a[0], a[1], a[2], w[0], w[1], w[2], Angle[0], Angle[1], Angle[2]);

}

void GyroscopeSerialInput::read_gyroscope(const std::string& real_data, void* context) {
	std::string data = real_data;
	std::string teststr = real_data;
	const char* a = teststr.c_str();
	char* b = new char[teststr.length() + 1];
	memset(b, 0, teststr.length() + 1);
	memcpy(b, a, teststr.length());
	unsigned char* c = (unsigned char*)b;  //  byteÓë  unsigned char*ÏàÍ¬
	for (int i = 0; i < teststr.length(); i++) {
		if ((int)(c[i]) == 0x55) {
			if (temp_count == 11) {
				decode_imu_data();
			}
			temp_count = 0;
			memset(temp, 0, sizeof(unsigned char) * 11);
		}
		if (temp_count >= 11) {
			temp_count = 0;
			memset(temp, 0, sizeof(char) * 11);
			memset(buffer, 0, sizeof(char) * buffer_size);
		}
		temp[temp_count] = c[i];
		temp_count++;
	}
	//SerialPortInfo* own = static_cast<SerialPortInfo*>(context);
	//printf("ori: \n%s\n", data.c_str());
}

void GyroscopeSerialInput::read_gyroscope(char * input_buffer, int count) {

	for (int i = 0; i < count; i++) {
		if ((int)(input_buffer[i]) == 0x55) {
			//printf("data %d\n", c[i]);
			if (temp_count == 11) {
				decode_imu_data();
			}
			temp_count = 0;
			memset(temp, 0, sizeof(unsigned char) * 11);
		}
		if (temp_count >= 11) {
			temp_count = 0;
			memset(temp, 0, sizeof(char) * 11);
			memset(buffer, 0, sizeof(char) * buffer_size);
		}
		temp[temp_count] = input_buffer[i];
		temp_count++;
	}
	//SerialPortInfo* own = static_cast<SerialPortInfo*>(context);
	//printf("ori: \n%s\n", data.c_str());
}

GnssEthernetInput::GnssEthernetInput(const std::string& entry, const ConnMode& connMode, const SolveMode& solveMode)
	: m_connMode(connMode)
	, m_solveMode(solveMode) {
	switch (m_connMode) {
	case SERIAL:
		break;
	case UDP:
	{
		std::string ip;
		unsigned short port;
		ConvertUtil::AddressConvert(entry, ip, port);
		m_udpServer.Init();
		m_udpServer.Bind(port);
	}
	break;
	case TCP:
	{
		std::string ip;
		unsigned short port;
		ConvertUtil::AddressConvert(entry, ip, port);
		if (!m_tcpClient.Init()) {
			std::cout << "[GPSSolver]TCP is not work currently." << std::endl;
		}
		while (!m_tcpClient.Connect(ip.c_str(), port)) {
			std::cout << "[GPSSolver]TCP is not connected." << std::endl;
		}
	}
	break;
	default:
		break;
	}
	// $ADV// $GBS// $GGA// $GLL// $GRS
	// $GSA// $GST// $GSV// $HDT
	// $RMC// $ROT// $VTG// $ZDA
	m_header.push_back("GGA");
	RegisterSolver(GGA);
	m_header.push_back("GST");
	RegisterSolver(GST);
	// $PTNL,AVR// $PTNL,BPQ// $PTNL,GGK// $PTNL,PJT
	// $PTNL,PJK// $PTNL,VGK// $PTNL,VHD
	m_ptnlHeader.push_back("AVR");
	RegisterSolver(AVR);
	m_ptnlHeader.push_back("VHD");
	RegisterSolver(VHD);
}


bool GnssEthernetInput::Solve(GPSPackage& package) {
	std::string msg;
	std::vector<std::string> segments;
	std::vector<std::string> tokens;
	if (!_Accept(msg)) return false;
	if (!_Segment(msg, segments)) return false;
	for (auto segment : segments) {
		_Phrase(segment, tokens);
		_Adapt(tokens, package, _Identify(tokens));
	}
	return true;
}

bool GnssEthernetInput::_Accept(std::string & msg) {
	switch (m_connMode) {
	case SERIAL:
		// TODO
		break;
	case UDP:
		if (!m_udpServer.RecvFrom(msg)) return false;
		return true;
		break;
	case TCP:
		return m_tcpClient.Recv(msg);
		break;
	default:
		break;
	}
	return false;
}

bool GnssEthernetInput::_Segment(const std::string& msg, std::vector<std::string>& segment) {
	return ConvertUtil::SplitString(msg, "\n", segment);
}

bool GnssEthernetInput::_Phrase(const std::string& msg, std::vector<std::string>& token) {
	return ConvertUtil::SplitString(msg, ",", token);
}

std::string GnssEthernetInput::_Identify(std::vector<std::string>& tokens) {
	if (tokens[0] == "")
		return std::string("NULL");
	std::string token = tokens[0].substr(1);
	std::vector<std::string>::iterator it;
	if (token == "PTNL") {
		it = find(m_ptnlHeader.begin(), m_ptnlHeader.end(), tokens[1]);
		if (it != m_ptnlHeader.end()) {
			return *it;
		} else return std::string("NULL");
	}
	it = find_if(m_header.begin(), m_header.end(), StringFinder(token));
	if (it != m_header.end()) {
		return *it;
	}
	return std::string("NULL");
}

bool GnssEthernetInput::_Adapt(std::vector<std::string>& token, GPSPackage& package, const std::string& msgType) {
	if (m_solverFunctionMap.count(msgType)) {
		return (this->*m_solverFunctionMap[msgType])(token, package);
	}
	return false;
}

bool GnssEthernetInput::_AVRSolver(std::vector<std::string>& token, GPSPackage& package) {
	package.m_avrtime = ConvertUtil::ConvertTo<std::string, float>(token[2]);
	package.m_yaw = ConvertUtil::ConvertTo<std::string, float>(token[3]);
	package.m_pitch = ConvertUtil::ConvertTo<std::string, float>(token[5]);
	return true;
}

bool GnssEthernetInput::_GGASolver(std::vector<std::string>& token, GPSPackage& package) {
	package.m_latitude = ConvertUtil::ConvertTo<std::string, double>(token[2]);
	package.m_latitudeDirection = token[3][0];
	package.m_longitude = ConvertUtil::ConvertTo<std::string, double>(token[4]);
	package.m_longitudeDirection = token[5][0];
	//package.m_gpsQuality = atoi(token[6]);
	package.m_gpsQuality = ConvertUtil::ConvertTo<std::string, unsigned char>(token[6]);
	package.m_elevation = ConvertUtil::ConvertTo<std::string, float>(token[9]);
	return true;
}

bool GnssEthernetInput::_VHDSolver(std::vector<std::string>& token, GPSPackage& package) {
	package.m_vhdtime = ConvertUtil::ConvertTo<std::string, float>(token[2]);
	package.m_roll = ConvertUtil::ConvertTo<std::string, float>(token[6]);
	return true;
}

bool GnssEthernetInput::_GSTSolver(std::vector<std::string>& token, GPSPackage& package) {
	package.m_yawErr = ConvertUtil::ConvertTo<std::string, float>(token[5]);
	package.m_latitudeErr = ConvertUtil::ConvertTo<std::string, float>(token[6]);
	package.m_longitudeErr = ConvertUtil::ConvertTo<std::string, float>(token[7]);
	package.m_elevationErr = ConvertUtil::ConvertTo<std::string, float>(token[8]);
	return true;
}

bool ImuSolver::Solve(std::vector<unsigned char> & msg) {
	// std::vector<std::string> tokens;
	// _Phrase(msg, tokens);
	// if (tokens.size() == 6)
	// {
	//   _AngleSolver(tokens[0], tokens[1], m_imuPackage.m_deltaAngleX);
	//   _AngleSolver(tokens[2], tokens[3], m_imuPackage.m_deltaAngleY);
	//   _AngleSolver(tokens[4], tokens[5], m_imuPackage.m_deltaAngleZ);
	//   return true;
	// }
	// return false;
	unsigned short words[12];
	memset(words, 0, sizeof(words));
	if (msg.size() < 50) {
		std::cout << "Length 12" << std::endl;
		return false;
	}
	ConvertUtil::ConvertToWord(msg[0], msg[1], words[0]);
	ConvertUtil::ConvertToWord(msg[2], msg[3], words[1]);
	ConvertUtil::ConvertToWord(msg[4], msg[5], words[2]);
	ConvertUtil::ConvertToWord(msg[6], msg[7], words[3]);
	ConvertUtil::ConvertToWord(msg[8], msg[9], words[4]);
	ConvertUtil::ConvertToWord(msg[10], msg[11], words[5]);
	ConvertUtil::ConvertToWord(msg[12], msg[13], words[6]);
	ConvertUtil::ConvertToWord(msg[14], msg[15], words[7]);
	ConvertUtil::ConvertToWord(msg[16], msg[17], words[8]);
	ConvertUtil::ConvertToWord(msg[18], msg[19], words[9]);
	ConvertUtil::ConvertToWord(msg[20], msg[21], words[10]);
	ConvertUtil::ConvertToWord(msg[22], msg[23], words[11]);
	//ConvertToWord(msg[12], msg[13], words[5]);
	//std::cout << words[6] << std::endl;

	_GyroSolver(words[0], words[1], m_imuPackage.m_gyroX);
	_GyroSolver(words[2], words[3], m_imuPackage.m_gyroY);
	_GyroSolver(words[4], words[5], m_imuPackage.m_gyroZ);
	_AcclSolver(words[6], words[7], m_imuPackage.m_acclX);
	_AcclSolver(words[8], words[9], m_imuPackage.m_acclY);
	_AcclSolver(words[10], words[11], m_imuPackage.m_acclZ);

	return true;
}
bool ImuSolver::_Phrase(const std::string& msg, std::vector<std::string>& token) {
	return ConvertUtil::SplitString(msg, ",", token);
}

bool ImuSolver::_GyroSolver(short lowHex, short highHex, double & value) {
	value = highHex * 0.02f;
	double msb = 0.02f;
	for (size_t i = 0; i < 16; i++) {
		msb *= 0.5f;
		if (0x8000 == (lowHex & 0x8000)) {
			value += msb;
		}
		lowHex <<= 1;
	}
	return true;
}

bool ImuSolver::_AcclSolver(short lowHex, short highHex, double & value) {
	value = highHex * 0.8f;
	double msb = 0.8f;
	for (size_t i = 0; i < 16; i++) {
		msb *= 0.5f;
		if (0x8000 == (lowHex & 0x8000)) {
			value += msb;
		}
		lowHex <<= 1;
	}
	return true;
}

bool ImuSolver::_AngleSolver(short lowHex, short highHex, double & value) {
	// short highHex = ConvertToHex<std::string, unsigned short>(highToken);
	// short highLow = ConvertToHex<std::string, unsigned short>(lowToken);
	value = highHex * 720.0f / 32768.0f;
	double msb = 32768.0f;
	for (size_t i = 0; i < 16; i++) {
		msb *= 2.0f;
		if (0x8000 == (lowHex & 0x8000)) {
			value += 720.0f / msb;
		}
		lowHex <<= 1;
	}
	return true;
}

