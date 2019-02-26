#pragma once

#include "unionconfig.h"

#include "pcdutil.h"

#ifdef WIN32
#include "pcap.h"
#include <windows.h>
#include <omp.h>
#include <direct.h>  
#include <io.h>
#endif
#include "stdutil.h"
#include "pcdutil.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

class LidarConfig {
public:
    static double * hdl32_vertical_angles;
    static uint8_t * hdl32_vertical_ids;
    static double * vlp16_vertical_angles;
    static uint8_t * vlp16_vertical_ids;
};

namespace mammoth{
    namespace io{
        class PcapProcesser {
        public:
            static PcapProcesser * get_instance();
            pcap_t * get_pcap_dev_handle(int ethernet_number);
            pcap_t * get_pcap_file_data(std::string pcap_path);
            pcap_t * get_pcap_dev_handle();
            void trans_pcap_to_pcd(std::string pcap_path, std::vector<pcl::PointCloud<PointType>::Ptr> & vec, int seg_count = 0);
            void play_pcap_file(std::string pcap_path, int start_packet_number = 0);
            void get_current_frame(pcap_t * cur_device, pcl::PointCloud<PointType>::Ptr & scene, int config);
            void parameter_init(float angle_piece, std::string path_prefix);
            ~PcapProcesser();
        private:
            static pcap_t * device;
            PcapProcesser();
            bool storage_flag;
            float angle_piece;
            std::string path_prefix;
            std::string root_path;
            static PcapProcesser * layer;
        };
    }
}
