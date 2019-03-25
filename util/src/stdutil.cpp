#include "stdutil.h"

using namespace mammoth::util;

bool UdpServer::Init() {
#ifdef _WIN32
	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &wsaData) != 0) return false;
	m_sockFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#elif __linux__
	m_sockFd = socket(AF_INET, SOCK_DGRAM, 0);
#endif
	if (m_sockFd == CRS_INVALID_SOCKET) {
		std::cout << "Create socket error." << std::endl;
		return false;
	}
#ifdef _WIN32
	int timeout = 1;
#elif __linux__
	struct timeval timeout = { 0, 1000 };
#endif
	setsockopt(m_sockFd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
	return true;
}

bool UdpServer::Bind(const unsigned short & servicePort) {
#ifdef _WIN32
	m_servAddr.sin_addr.S_un.S_addr = INADDR_ANY;
#elif __linux__
	bzero(&m_servAddr, sizeof(m_servAddr));
	m_servAddr.sin_addr.s_addr = htonl(INADDR_ANY); //inet_addr("127.0.0.1");
#endif
	m_servAddr.sin_family = AF_INET;
	m_servAddr.sin_port = htons(servicePort);
	if (bind(m_sockFd, (struct sockaddr*)&m_servAddr, sizeof(m_servAddr)) != CRS_SOCKET_ERROR)
		return true;
	else return false;
}

bool UdpServer::Release() {
	if (m_sockFd != CRS_INVALID_SOCKET) {
#ifdef _WIN32
		closesocket(m_sockFd);
		WSACleanup();
#elif __linux__
		close(m_sockFd);
#endif
		m_sockFd = CRS_INVALID_SOCKET;
	}
	return true;
}

bool UdpServer::SendTo(const char * data, const size_t & length, const char * clientIp, const int & port) {
	m_sendAddr.sin_family = AF_INET;
	m_sendAddr.sin_port = htons(port);
#ifdef _WIN32
	inet_pton(AF_INET, clientIp, &m_sendAddr);
#elif __linux__
	m_sendAddr.sin_addr.s_addr = inet_addr(clientIp);
	bzero(&(m_sendAddr.sin_zero), 8);
#endif

	int ret = sendto(m_sockFd, data, length, 0, (struct sockaddr*)&m_sendAddr, sizeof(m_sendAddr));
	if (ret == 0) {
		std::cout << "Connection closed." << std::endl;
		return false;
	}
	if (ret < 0) {
		std::cout << "Error." << std::endl;
		return false;
	}
	if (ret != static_cast<int>(length)) {
		std::cout << "Send length error." << std::endl;
		return false;
	}
	return true;
}

bool UdpServer::RecvFrom(char * data, size_t & length, char ** ppaddr, unsigned short * pport) {
	int addr_len = sizeof(m_servAddr);
	length = recvfrom(m_sockFd, data, length, 0/*MSG_DONTWAIT*/, (struct sockaddr*)&m_servAddr,
#ifdef _WIN32
		/*(socklen_t*)*/&addr_len
#elif __linux__
		(socklen_t*)&addr_len
#endif
		);
	if (length == SIZE_MAX) {
		// std::cout << "UDP timeout." << std::endl;
		return false;
	}

	if (ppaddr) {
		// TODO : add support to get addr
		// #ifdef _WIN32
		//     inet_ntop(AF_INET, &m_sendAddr, *ppaddr, 0);
		// #elif __linux__
		//     *ppaddr = inet_ntoa(m_sendAddr.sin_addr);
		// #endif
	}

	if (pport)
		*pport = ntohs(m_sendAddr.sin_port);
	return true;
}

bool UdpServer::RecvFrom(std::string & msg) {
	m_bufferLength = 512;
	if (!RecvFrom(m_buffer, m_bufferLength, NULL, NULL))
		return false;
	msg.assign(m_buffer, m_buffer + m_bufferLength);
	return true;
}

TcpClient::~TcpClient() {
	DisConnect();
	if (m_transform != nullptr) {
		delete[] m_transform;
		m_transform = nullptr;
	}
}
bool TcpClient::Init() {
#ifdef _WIN32
	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &wsaData) != 0) return false;
	m_sockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#elif __linux__
	m_sockFd = socket(AF_INET, SOCK_STREAM, 0);
#endif
	if (m_sockFd == CRS_INVALID_SOCKET) {
		return false;
	}
	int ret;
#ifdef _WIN32
	unsigned long ul = 1;
	ret = ioctlsocket(m_sockFd, FIONBIO, (unsigned long *)&ul);
#elif __linux__
	int flags = fcntl(m_sockFd, F_GETFL, 0);
	ret = fcntl(m_sockFd, F_SETFL, flags | O_NONBLOCK);
	// int imode = 1;
	// ioctl(sock_fd, FIONBIO, &imode);
#endif
	if (ret == CRS_SOCKET_ERROR) {
		std::cout << "Socket unblocked error." << std::endl;
		return false;
	}
	return true;
}

bool TcpClient::Connect(const char * addr, const unsigned short & servicePort) {
	m_servAddr.sin_family = AF_INET;
	m_servAddr.sin_port = htons(servicePort);
#ifdef _WIN32
	m_servAddr.sin_addr.S_un.S_addr = inet_addr(addr);
#elif __linux__
	m_servAddr.sin_addr.s_addr = inet_addr(addr);
	bzero(&(m_servAddr.sin_zero), 8);
#endif
	if (connect(m_sockFd, (struct sockaddr*)&m_servAddr, sizeof(struct sockaddr)) != CRS_SOCKET_ERROR) {
		m_isConnected = true;
		// create transform buffer
		if (m_transform == nullptr) {
			m_transform = new char[BUFFER_SIZE];
			m_transLength = BUFFER_SIZE;
		}
		if (m_transform != nullptr)
			return true;
	}
	return false;
}

bool TcpClient::DisConnect() {
	if (m_isConnected) {
#ifdef _WIN32
		closesocket(m_sockFd);
		WSACleanup();
#elif __linux__
		close(m_sockFd);
#endif
		m_sockFd = CRS_INVALID_SOCKET;
		m_isConnected = false;
		return true;
	}
	return false;
}

bool TcpClient::Recv(std::string& msg) {
	if (m_transform == nullptr) {
		std::cout << "[Recv]Client is not working." << std::endl;
		return false;
	}
	m_transLength = BUFFER_SIZE;
	m_transLength = recv(m_sockFd, m_transform, m_transLength, 0);

	if (m_transLength == CRS_SOCKET_ERROR) {
#ifdef _WIN32
		int err = WSAGetLastError();
		if (err == WSAEWOULDBLOCK) {
			std::cout << "Recv WOULDBLOCK error." << std::endl;
		} else if (err == WSAETIMEDOUT) {
			std::cout << "Recv TIMEOUT error." << std::endl;
		} else if (err == WSAENETDOWN) {
			std::cout << "Recv NETDOWN error." << std::endl;
		}
#elif __linux__
		if (errno == EWOULDBLOCK) {
			// std::cout << "Recv WOULDBLOCK error." << std::endl;
			return false;
		}
#endif
	}
	if (m_transLength == BUFFER_SIZE) {
		std::cout << "[Recv]Package is too large." << std::endl;
		return false;
	}
	m_transform[m_transLength] = '\0';
	msg = m_transform;
	return true;
}

// transform command
bool TcpClient::Send(const std::string& msg) {
	int sentSize = send(m_sockFd, msg.c_str(), msg.length(), 0);
	if (sentSize == static_cast<int>(msg.length())) {
		return true;
	}
	std::cout << "We can't send it by one package." << std::endl;
	return false;
}


std::string ConvertUtil::itoa(const int& number) {
	std::stringstream ss;
	ss << number;
	std::string str;
	ss >> str;
	return str;
}

int ConvertUtil::atoi(const std::string& str) {
	std::stringstream ss;
	ss << str;
	int res;
	ss >> res;
	return res;
}

float ConvertUtil::string2float(const std::string& str) {
	std::stringstream ss;
	ss << str;
	float ret;
	ss >> ret;
	return ret;
}

bool ConvertUtil::AddressConvert(const std::string& src, std::string& ip, unsigned short& port) {
	size_t pos = src.find_first_of(':', 0);
	ip = src.substr(0, pos);
	// port = atoi(src.substr(pos + 1));
	port = ConvertTo<std::string, unsigned short>(src.substr(pos + 1));
	return true;
}

bool ConvertUtil::SplitString(const std::string& str, const std::string& pattern, std::vector<std::string>& strlist) {
	strlist.clear();
	if (pattern.empty()) return false;
	size_t start = 0;
	size_t index = str.find_first_of(pattern, 0);
	while (index != str.npos) {
		if (start != index)
			strlist.push_back(str.substr(start, index - start));
		else
			strlist.push_back(std::string());
		start = index + 1;
		index = str.find_first_of(pattern, start);
	}
	if (!str.substr(start).empty())
		strlist.push_back(str.substr(start));
	return true;
}

void ConvertUtil::ConvertToWord(const unsigned char & low, const unsigned char & high, unsigned short & word) {
	word = high;
	word <<= 8;
	word |= low;
}

std::thread ThreadUtil::create_thread(void(*thread_func)()) {
	std::thread thread(thread_func);
	return thread;
}

void ThreadUtil::join(std::thread& thread) {
	thread.join();
}

void ThreadUtil::kill(std::thread& thread) {
	//thread.
}


void SerialUtil::openAsync(const char * serial, int baut_rate, void(*read_func)(const std::string& data, void* context)) {
	SerialPortInfo Com1info;
	ASynCom com1;
	Com1info.name = serial;
	Com1info.baudRate = baut_rate;
	Com1info.parity = NOPARITY;
	Com1info.dataBits = 8;
	Com1info.stopBits = 1;
	com1.SetRecvDataCallBack(read_func, &Com1info);
	if (true) {
		com1.Connect(Com1info);
		com1.Start();
		/*std::thread tcom1(gjm::write, &Com1, "com1_AAAAA", &count);
		if (tcom1.joinable()) {
		tcom1.join();
		}*/
	}
	system("pause");
}

SyncCom SerialUtil::openSync(const char * serial, int baut_rate) {
	SerialPortInfo Com1info;
	SyncCom com1;
	Com1info.name = serial;
	Com1info.baudRate = baut_rate;
	Com1info.parity = NOPARITY;
	Com1info.dataBits = 8;
	Com1info.stopBits = 1;
	com1.Connect(Com1info);
	return com1;
}



void SerialUtil::close() {

}

void SerialUtil::read_gps(const std::string& data, void* context) {

}

void SerialUtil::write(ASynCom* com, const std::string& msg, int* cont) {
	std::string temp;
	//ѭ������
	while (true) {
		(*cont)++;
		temp = msg + " times is " + std::to_string(*cont);
		com->SendData(temp);
		temp = "";
		Sleep(800);
	}
}

FileUtil::FileUtil() {
	this->in_file_stream_ptr = nullptr;
	this->out_file_stream_ptr = nullptr;
}

FileUtil::FileUtil(const char * path, unsigned open_mode) {
	this->in_file_stream_ptr = nullptr;
	this->out_file_stream_ptr = nullptr;
	switch (open_mode) {
	case 0:{
		//read
		this->in_file_stream_ptr = new std::ifstream(path);
		break;
	}
	case 1:{
		//append
		this->out_file_stream_ptr = new std::ofstream(path, std::ios::app);
		break;
	}
	case 2:{
		//write
		this->out_file_stream_ptr = new std::ofstream(path, std::ios::out);
		break;
	}
	}
};

void FileUtil::reload_file(const char * path, unsigned open_mode) {
	if (this->in_file_stream_ptr != nullptr) {
		this->in_file_stream_ptr->close();
	}
	if (this->out_file_stream_ptr != nullptr) {
		this->out_file_stream_ptr->close();
	}
	this->in_file_stream_ptr = nullptr;
	this->out_file_stream_ptr = nullptr;
	switch (open_mode) {
	case 0:{
		//read
		this->in_file_stream_ptr = new std::ifstream(path);
		break;
	}
	case 1:{
		//append
		this->out_file_stream_ptr = new std::ofstream(path, std::ios::app);
		break;
	}
	case 2:{
		//write
		this->out_file_stream_ptr = new std::ofstream(path, std::ios::out);
		break;
	}
	}
};

FileUtil::~FileUtil() {
	if (in_file_stream_ptr != nullptr) {
		in_file_stream_ptr->close();
		delete in_file_stream_ptr;
	}
	if (out_file_stream_ptr != nullptr) {
		out_file_stream_ptr->close();
		delete out_file_stream_ptr;
	}
}

std::string FileUtil::read_line() {
	if (in_file_stream_ptr == nullptr) {
		perror("FILE INPUT STREAM PTR is NULL!");
		exit(1);
	}
	char buffer[301];
	in_file_stream_ptr->getline(buffer, 300);
	std::string result = buffer;
	return result;
}

void FileUtil::write_line(const std::string & content, bool new_line) {
	if (out_file_stream_ptr == nullptr) {
		perror("FILE OUTPUT STREAM PTR is NULL!");
		exit(1);
	}
	(*out_file_stream_ptr) << content.c_str();
	if (new_line) {
		(*out_file_stream_ptr) << "\n";
	}
	(*out_file_stream_ptr) << std::flush;
}



void FileUtil::get_all_files(std::string path, std::vector<std::string>& files) {
	WIN32_FIND_DATAA fdata;
	HANDLE hFind = FindFirstFileA(path.c_str(), &fdata);
	if (hFind != INVALID_HANDLE_VALUE) {
		do {
			if (fdata.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
				continue;
			} else {
				files.push_back(fdata.cFileName);
			}
		} while (FindNextFileA(hFind, &fdata));
		FindClose(hFind);
	}

	int length = files.size();
	std::string * strs = new std::string[length];
	//�ֵ�������
	for (int i = 0; i < length; i++) {
		std::string str = files[i];
		std::string temp = str.substr(0, str.find_first_of("_"));
		int index = atoi(temp.c_str());
		strs[index] = str;
	}
	files.clear();
	files.reserve(length);
	for (int i = 0; i < length; i++) {
		files.push_back(strs[i]);
	}
	//delete strs;
}



SYSTEMTIME TimeUtil::sys_time;

std::string TimeUtil::get_time_str() {
	std::string time_str;
	char temp[40];
	memset(temp, 0, sizeof(char) * 40);
	GetLocalTime(&sys_time);
	sprintf(temp, "%04d/%02d/%02d %02d:%02d:%02d.%03d", sys_time.wYear, sys_time.wMonth, sys_time.wDay, sys_time.wHour, sys_time.wMinute, sys_time.wSecond, sys_time.wMilliseconds);
	time_str = temp;
	printf("%s\n", temp);
	return time_str;
}

int TimeUtil::get_time(const TIME_ITEM& time_item) {
	GetLocalTime(&sys_time);
	switch (time_item) {
	case TIME_ITEM::YEAR:{
		return sys_time.wYear;
	}
	case TIME_ITEM::MONTH:{
		return sys_time.wMonth;
	}
	case TIME_ITEM::DATE:{
		return sys_time.wDay;
	}
	case TIME_ITEM::HOUR:{
		return sys_time.wHour;
	}
	case TIME_ITEM::MINUTE:{
		return sys_time.wMinute;
	}
	case TIME_ITEM::SECOND:{
		return sys_time.wSecond;
	}
	case TIME_ITEM::MILLSECOND:{
		return sys_time.wMilliseconds;
	}
	case TIME_ITEM::DAY:{
		return sys_time.wDayOfWeek;
	}
	}
	return 0;
}

std::string TimeUtil::get_time_code() {
	std::string time_str;
	char temp[40];
	memset(temp, 0, sizeof(char) * 40);
	GetLocalTime(&sys_time);
	sprintf(temp, "%04d%02d%02d%02d%02d%02d", sys_time.wYear, sys_time.wMonth, sys_time.wDay, sys_time.wHour, sys_time.wMinute, sys_time.wSecond, sys_time.wMilliseconds);
	time_str = temp;
	return time_str;
}

std::string TimeUtil::get_time_code_millsecond() {
	std::string time_str;
	char temp[40];
	memset(temp, 0, sizeof(char) * 40);
	GetLocalTime(&sys_time);
	sprintf(temp, "%04d%02d%02d%02d%02d%02d%03d", sys_time.wYear, sys_time.wMonth, sys_time.wDay, sys_time.wHour, sys_time.wMinute, sys_time.wSecond, sys_time.wMilliseconds);
	time_str = temp;
	return time_str;
}


