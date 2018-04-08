#pragma once

#include "UnionConfig.h"
#include "SerialLibrary.h"

#include <thread>
#include <fstream>

#ifdef _WIN32
#include <time.h>
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#define CRS_SOCKET SOCKET
#define CRS_INVALID_SOCKET INVALID_SOCKET
#define CRS_SOCKET_ERROR SOCKET_ERROR
#elif __linux__
#include <unistd.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#define CRS_SOCKET int
#define CRS_INVALID_SOCKET -1
#define CRS_SOCKET_ERROR -1
#endif
#define BUFFER_SIZE 512


namespace mammoth {
	namespace config {
		class UtilLayerConfig {
		public:

		};
	}
}