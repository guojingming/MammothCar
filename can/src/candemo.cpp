#include "mammoth.h"

#include <canlib.h>
#include "candemo.h"
#include "stdio.h"

// module globals
unsigned int   m_usedBaudRate                  = 500000;
canHandle      m_usedChannel;
unsigned int   m_usedId;
unsigned int   m_usedFlags                     = 0;
unsigned int   m_Verbose                       = 1;

driverData     m_channelData;
driverData    *m_DriverConfig                 = &m_channelData;


uint32_t pcd_signal_count = 0;
uint32_t last_pcd_signal_count = 1000000;
//////////////////////////////////////////////////////////////////////////////////////
//Function that prints channel data
//////////////////////////////////////////////////////////////////////////////////////
void printDriverConfig( void )
{
  unsigned int i;

  printf("\nDriver Configuration:\n  ChannelCount=%u\n", m_DriverConfig->channelCount);
  for (i = 0; i < m_DriverConfig->channelCount; i++) {

    printf("  %s : Channel %d, isOnBus=%d, Baudrate=%u",
           m_DriverConfig->channel[i].name,
           m_DriverConfig->channel[i].channel,
           m_DriverConfig->channel[i].isOnBus,
           m_usedBaudRate);

    switch(m_usedBaudRate) {
      case canBITRATE_1M:
        printf("canBITRATE_1M");
        break;
      case canBITRATE_500K:
        printf("canBITRATE_500K");
        break;
      case canBITRATE_250K:
        printf("canBITRATE_250K");
        break;
      case canBITRATE_125K:
        printf("canBITRATE_125K");
        break;
      case canBITRATE_100K:
        printf("canBITRATE_100K");
        break;
      case canBITRATE_62K:
        printf("canBITRATE_62K");
        break;
      case canBITRATE_50K:
        printf("canBITRATE_50K");
        break;
      default:
        printf("UNKNOWN");
    }
    printf("\n    ");

    if (m_DriverConfig->channel[i].driverMode == canDRIVER_NORMAL) {
      printf("Drivermode=canDRIVER_NORMAL\n");
    } else {
      printf ("Drivermode=canDRIVER_SILENT\n");
    }
  }

  printf("\n\n");
}

////////////////////////////////////////////////////////////////////////////////////
// Open one handle to each channel present in the system.
// Set bus parameters
////////////////////////////////////////////////////////////////////////////////////
void InitDriver(void)
{
  int  i;
  canStatus  stat;

  // Initialize ChannelData.
  memset(m_channelData.channel, 0, sizeof(m_channelData.channel));
  for (i = 0; i < MAX_CHANNELS; i++) {
    m_channelData.channel[i].isOnBus      = 0;
    m_channelData.channel[i].driverMode   = canDRIVER_NORMAL;
    m_channelData.channel[i].channel      = -1;
    m_channelData.channel[i].hnd          = canINVALID_HANDLE;
    m_channelData.channel[i].txAck        = 0; // Default is TxAck off
  }
  m_channelData.channelCount = 0;


  //
  // Enumerate all installed channels in the system and obtain their names
  // and hardware types.
  //

  //initialize CANlib
  canInitializeLibrary();

  //get number of present channels
  stat = canGetNumberOfChannels((int*)&m_channelData.channelCount);

  for (i = 0; (unsigned int)i < m_channelData.channelCount; i++) {
    canHandle  hnd;

    //obtain some hardware info from CANlib
    m_channelData.channel[i].channel = i;
    canGetChannelData(i, canCHANNELDATA_CHANNEL_NAME,
                      m_channelData.channel[i].name,
                      sizeof(m_channelData.channel[i].name));
    canGetChannelData(i, canCHANNELDATA_CARD_TYPE,
                      &m_channelData.channel[i].hwType,
                      sizeof(DWORD));

    //open CAN channel
    hnd = canOpenChannel(i, canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0) {
      // error
      PRINTF_ERR(("ERROR canOpenChannel() in initDriver() FAILED Err= %d. <line: %d>\n",
                  hnd, __LINE__));
    }
    else {
      m_channelData.channel[i].hnd = hnd;
      if ((stat = canIoCtl(hnd, canIOCTL_FLUSH_TX_BUFFER, NULL, NULL)) != canOK)
        PRINTF_ERR(("ERROR canIoCtl(canIOCTL_FLUSH_TX_BUFFER) FAILED, Err= %d <line: %d>\n",
                    stat, __LINE__));
    }

    //set up the bus
    if (i == 0) {
      switch(m_usedBaudRate) {
        case 1000000:
          m_usedBaudRate = canBITRATE_1M;
          break;
        case 500000:
          m_usedBaudRate = canBITRATE_500K;
          break;
        case 250000:
          m_usedBaudRate = canBITRATE_250K;
          break;
        case 125000:
          m_usedBaudRate = canBITRATE_125K;
          break;
        case 100000:
          m_usedBaudRate = canBITRATE_100K;
          break;
        case 62500:
          m_usedBaudRate = canBITRATE_62K;
          break;
        case 50000:
          m_usedBaudRate = canBITRATE_50K;
          break;
        default:
          printf("Baudrate set to 125 kbit/s. \n");
          m_usedBaudRate = canBITRATE_125K;
          break;
      }
    }

    //set the channels busparameters
    stat = canSetBusParams(hnd, m_usedBaudRate, 0, 0, 0, 0, 0);
    if (stat < 0) {
      PRINTF_ERR(("ERROR canSetBusParams() in InitDriver(). Err = %d <line: %d>\n",
                  stat, __LINE__));
    }
  }
  printf("\n");
}

//////////////////////////////////////////////////
// Go off bus and close all open handles.
//////////////////////////////////////////////////
void Cleanup(void)
{
  unsigned int i;

  for (i = 0; i < m_channelData.channelCount; i++) {
    canStatus stat;
    if ((stat = canBusOff(m_channelData.channel[i].hnd)) != canOK)
      PRINTF_ERR(("ERROR canBusOff() FAILED Err= %d. <line: %d>\n", stat, __LINE__));


    if ((stat = canClose(m_channelData.channel[i].hnd)) != canOK) {
      PRINTF_ERR(("ERROR canClose() in Cleanup() FAILED Err= %d. <line: %d>\n",
                  stat, __LINE__));
    }
  }
}


void can_signal_get_current_frame(pcap_t * cur_device, pcl::PointCloud<PointType>::Ptr & scene, int config) {
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
					if (config != 2) {
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
			if (count >= 179){
			//if (last_pcd_signal_count != pcd_signal_count) { //177  180
				last_pcd_signal_count = pcd_signal_count;
				delete angles;
				delete distance_mm;
				delete flectivity;
				//printf("packcount: %d\n", )
				count = 0;
				break;
			}
			count++;
		}
	}
}

/*************************************************************/
/*************************************************************/
// MAIN
///////////////////////////////////////////////////////////////
int can_main(){
	HANDLE        th[MAX_CHANNELS + 1];
	static int    running                 = 1;
	DWORD         active_handle;
	char          c;
	canStatus     stat;
	unsigned int  i;
	m_usedId      = 0;
	m_usedChannel = 0;
	// open channel and set busparams, etc...
	InitDriver();
	//get std_input event handle
	th[0] = GetStdHandle(STD_INPUT_HANDLE);
	if (th[0] == INVALID_HANDLE_VALUE)
	PRINTF_ERR(("ERROR inv handle (std_input). <line: %d>\n", __LINE__));
	for (i = 1; i < (m_channelData.channelCount + 1); i++) {
		HANDLE tmp;
		//go on bus (every channel)
		stat = canBusOn(m_channelData.channel[i-1].hnd);
		if (stat < 0) {
			PRINTF_ERR(("ERROR canBusOn(). Err = %d <line: %d>\n", stat, __LINE__));
		}
		else {
			m_DriverConfig->channel[i-1].isOnBus = 1;
		}

		//get CAN - eventHandles
		stat = canIoCtl(m_channelData.channel[i-1].hnd,
						canIOCTL_GET_EVENTHANDLE,
						&tmp,
						sizeof(tmp));
		if (stat < 0) {
			PRINTF_ERR(("canIoCtl(canIOCTL_GET_EVENTHANDLE) FAILED. Err = %d <line: %d>\n",
						stat, __LINE__));
		}
		th[i] = tmp;
	}
	printDriverConfig();
	printf("\n");
	///////////////////////////////////////////
	//Main LOOP
	while (running) {
		active_handle = WaitForMultipleObjects(m_channelData.channelCount + 1,
												th,
												FALSE /*any*/,
												INFINITE);
		if (((active_handle - WAIT_OBJECT_0) > 0) &&
			((active_handle - WAIT_OBJECT_0) <= m_channelData.channelCount)){
			unsigned int    j;
			long            id;
			//unsigned char   data[32];
			uint32_t data;
			unsigned int    dlc;
			unsigned int    flags;
			DWORD           time;
			int             moreDataExist;

			do {
				moreDataExist = 0;
				for (i = 0; i < m_channelData.channelCount; i++) {
					//stat = canRead(m_channelData.channel[i].hnd, &id, &data[0], &dlc, &flags, &time);
					stat = canRead(m_channelData.channel[i].hnd, &id, &data, &dlc, &flags, &time);
					switch (stat) {
					case canOK:
						if (m_Verbose) {
							printf("RxMsg: Ch:%d ID:%08lx DLC:%u Flg:%02x T:%08lx Data: %d \n", m_channelData.channel[i].channel, id, dlc, flags, time, data);
							//printf("RxMsg: Ch:%d ID:%08lx Data: %d\n", m_channelData.channel[i].channel, id, data);
							if (id == 0x400) {
								pcd_signal_count = data;
							}
							
							if ((flags & canMSG_RTR) == 0) {
								/*for (j = 0; j < dlc; j++) {
									printf("%02x ", data[j]);
								}*/
								//printf("can: %d", data);
							}
							//printf("\n");
						}
						moreDataExist = 1;
						break;

					case canERR_NOMSG:
						// No more data on this handle
						break;

					default:
						PRINTF_ERR(("ERROR canRead() FAILED, Err= %d <line: %d>\n", stat, __LINE__));
						break;
					}
				}
			} while (moreDataExist);
		}
		//STD_INPUT event
		else if (active_handle == WAIT_OBJECT_0){
			unsigned long  n;
			INPUT_RECORD   ir;
			ReadConsoleInput(GetStdHandle(STD_INPUT_HANDLE), &ir, 1, &n);
		} //event type
	} // while
	// Go off bus & close channels.
	Cleanup();
	return NULL;
}
