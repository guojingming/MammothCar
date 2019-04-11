#pragma once

#include "header.h"

namespace mammoth{
    class PcdProcesser{
    public:
        static PcdProcesser * get_instance();
        void rotation(pcl::PointCloud<PointType>::Ptr & cloud, float x, float y, float z);
        void translation(pcl::PointCloud<PointType>::Ptr & cloud, float x, float y, float z);
        void combine(pcl::PointCloud<PointType>::Ptr & cloud1, pcl::PointCloud<PointType>::Ptr & cloud2, pcl::PointCloud<PointType>::Ptr & combine_cloud);
        void pcd_to_bin(std::string pcd_path, string bin_path);
        pcl::PointCloud<pcl::PointXYZI>::Ptr bin_to_pcd(std::string &in_file);
        ~PcdProcesser();
    private:
        static PcdProcesser * layer;
        PcdProcesser();
    };
}
