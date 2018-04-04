#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h> 

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp" 

#include "UnionConfig.h"

#include "glViewer.h"
#include "DataFormat.h"
#include "StdUtilLayer.h"

#define PointType pcl::PointXYZRGBA

namespace mammoth {
	namespace config {
		
	}
}