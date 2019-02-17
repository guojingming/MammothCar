#pragma once

#include <thread>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#ifdef WIN32
#define _WINSOCKAPI_
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#endif

/* #include <winbase.h>
#define NOPARITY            0
#define ODDPARITY           1
#define EVENPARITY          2
#define MARKPARITY          3
#define SPACEPARITY         4


#define ONESTOPBIT          0
#define ONE5STOPBITS        1
#define TWOSTOPBITS         2
*/

struct SerialPortInfo {
	std::string name;
	unsigned int baudRate{ 115200 };
	unsigned int parity{ 'e' };
	unsigned int dataBits{ 7 };
	unsigned int stopBits{ 1 };
};


class BaseCom {
public:
	BaseCom();
	virtual ~BaseCom();
public:
	bool Connect(const SerialPortInfo& portinfo);
	void Close();
	bool IsOpen();
#ifdef WIN32
public:
	bool Connect(const std::string& portName, unsigned int baudRate = CBR_9600, unsigned int parity = NOPARITY,
		unsigned int dataBits = 8, unsigned int stopBits = ONESTOPBIT);
	bool Connect(int port, unsigned int baudRate = CBR_9600, unsigned int parity = NOPARITY, unsigned int dataBits = 8, unsigned int stopBits = ONESTOPBIT);
protected:
	virtual bool OpenPort() = 0;
	void Init();
	bool SetupPort();
	void SetComPort(int port);
	bool SetState(int BaudRate, int ByteSize, int Parity, int StopBits);
protected:
	volatile int _port;
	volatile HANDLE _com_handle;
	char _com_str[20];
	DCB _dcb;
	COMMTIMEOUTS _co;
#else
public:
	bool Connect(const std::string& portName, unsigned int baudRate = 9600, unsigned int parity = 'N',
		unsigned int dataBits = 8, unsigned int stopBits = 1);
protected:
	bool openLinuxPort(const SerialPortInfo& portinfo);
	bool setLinuxPortOpt(const SerialPortInfo& portinfo);
protected:
	int _fdSerial{ -1 };
	bool _isOpen{ false };
#endif
protected:
	SerialPortInfo _portinfo;
};

#ifdef WIN32
class SyncCom : public BaseCom {
public:
	SyncCom();
	int Read(char *buf, int buf_len);
	int SendData(const char* buf, int buf_len);
	int SendData(const char* buf);
protected:
	virtual bool OpenPort();
};
#endif

typedef void(*RecvDataCallBack)(const std::string& data, void* context);
class ASynCom : public BaseCom {
public:
	ASynCom();

	virtual ~ASynCom();

	int Read(char* buf, int buf_len);

	bool SendData(const char* buf, int buf_len);
	bool SendData(const std::string& buf);

	void Start();
	void Stop();
	void SetRecvDataCallBack(RecvDataCallBack callback, void* context);

protected:
	static void OnProcRecvData(void* context);
#ifdef WIN32
protected:
	virtual bool OpenPort();
protected:
	OVERLAPPED _ro, _wo;
	OVERLAPPED _wait_o;//WaitCommEvent use
#else
	ssize_t readDataTty(int fd, char *rcv_buf, int TimeOut, int Len);
	ssize_t sendDataTty(int fd, const char *send_buf, int Len);
	void reInit();
#endif
	bool _isExit = false;
	std::thread* _workThread = NULL;
	RecvDataCallBack _recvDataCallback = NULL;
	void* _recvDataContext = NULL;
};