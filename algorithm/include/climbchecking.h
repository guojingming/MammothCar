#pragma once

#include "algorithm.h"

namespace mammoth{
    namespace algorithm{
        class ClimbingLayer {
        public:
            ~ClimbingLayer();
            static ClimbingLayer * get_instance();
            float get_height_threshold(float min_x, float max_x);
            void climbing_check(pcl::PointCloud<PointType>::Ptr & cloud, char * point_path, char * result_path, int frame_count);
            float get_ground_average_altitude(pcl::PointCloud<PointType>::Ptr & cloud);
        private:
            static ClimbingLayer * layer;
            ClimbingLayer();
        };
    }
}