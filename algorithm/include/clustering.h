#pragma once

#include "unionconfig.h"
#include "pcdutil.h"
#include "stdutil.h"

#define _Key(x,y) ((x & 0x0000FFFF)|(y<<16))

namespace mammoth{
    class DimensionReductionCluster {
    public:
        static void start_clusting(pcl::PointCloud<PointType>::Ptr & cloud);
    private:
        static std::vector<uint32_t> cube_handles;
        static pcl::PointCloud<PointType>::Ptr birdview_picture_grid(pcl::PointCloud<PointType>::Ptr& cloud);
        static int cz_region(cv::Mat src, std::vector<std::vector<cv::Point>> &points, int filter_size, bool fill_flag, int pic_width, int pic_height);
    };
}