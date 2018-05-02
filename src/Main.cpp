#include "Mammoth.h"

using namespace mammoth::layer;

void cz_test();
void gjm_test();
void byz_test();

int main(int argc, char ** argv) {
	
	//cz_test();
	gjm_test();
	//byz_test();

	return 0;
}

void cz_test() {
	int frame_count = 5;
	std::vector<pcl::PointCloud<PointType>::Ptr> vec;
	for (int i = 0; i < frame_count; i++) {
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
		vec.push_back(cloud);
	}
	PcapTransformLayer::get_instance()->trans_pcap_to_pcd("D:\\Download\\8.pcap", vec, 2);
	
	for (int i = 0; i < frame_count; i++) {
		printf("%d\n", vec[i]->size());
		char temp[32];
		sprintf(temp, "D:\\%d.pcd", i);
		PcdUtil::save_pcd_file(temp, vec[i]);
	}

	system("pause");
}


void gjm_test() {
	//GNSSFilter gnssFilter(0,0,0,0,0,0);

	//ObjectTracking::start_tracking(0, 0);
	MammothViewer* p_viewer = OpencvViewerManager::get_instance()->create_viewer("hello", MyPoint2D(0, 0), MyPoint2D(1440, 1080));
	int i = 0;
	while(true){
		p_viewer->clear_window();
		MyBox box;
		box.point1.x = 500 + 100 * sin(i * 3.1415926 / 180);
		box.point1.y = 500 + 100 * cos(i * 3.1415926 / 180);
		box.point2.x = 500 + 100 * sin(i * 3.1415926 / 180) + 100;
		box.point2.y = 500 + 100 * cos(i * 3.1415926 / 180);
		box.point4.x = 500 + 100 * sin(i * 3.1415926 / 180);
		box.point4.y = 500 + 100 * cos(i * 3.1415926 / 180) + 100;
		box.point3.x = 500 + 100 * sin(i * 3.1415926 / 180) + 100;
		box.point3.y = 500 + 100 * cos(i * 3.1415926 / 180) + 100;
		MyPoint3D color(255, 0, 0);
		p_viewer->draw_rectangle(box,color, 2, true);
		p_viewer->wait_rendering(1);
		i+=4;
		
	}

	//�µ�pcd��ȡ����
	//PointViewer::get_instance()->init_point_viewer();
	/*PCDFILE f;
	PcdUtil::read_pcd_file("C:\\DataSpace\\map\\0406-1.pcd", &f);
	PointViewer::get_instance()->set_point_cloud(f);
	system("pause");*/

	//��¼����
	//DataGatherLayer::get_instance()->start_grab("C:\\DataSpace\\LidarDataSpace\\lidar_20180322\\gps_data","C:\\DataSpace\\LidarDataSpace\\lidar_20180322\\pcd_data","C:\\DataSpace\\LidarDataSpace\\lidar_20180322\\imu_data");
	
	//�����㷨
	/*PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	PcdUtil::read_pcd_file("D:\\TTT.pcd", cloud);
	DimensionReductionCluster::start_clusting(cloud);
	printf("%d\n", cloud->size());
	PointViewer::get_instance()->set_point_cloud(cloud);
	system("pause");*/
	
	//PCDתXYZ
	//PcdUtil::trans_pcd_to_xyz("D:/map.pcd","D:/map.xyz");
	

	//SLAM
	//JluSlamLayer::get_instance()->start_slam("C:\\DataSpace\\LidarDataSpace\\lidar_20180322-1\\gps_data","C:\\DataSpace\\LidarDataSpace\\lidar_20180322-1\\pcd_data");
} 

void byz_test() {
#ifdef WIN32
	SyncCom sync_com = SerialUtil::openSync("COM4", 460800);

	char buffer[512];
	ImuSolver imuSolver;
	std::vector<unsigned char> splits;
	splits.push_back('\r');
	splits.push_back('\n');
	DataBuffer<unsigned char> dataBuffer(splits);
	std::vector<std::vector<unsigned char> > segments;
	
	while (1) {
		//memset(buffer, 0, 512);
		int readByte = sync_com.Read(buffer, 512);
		//std::cout << readByte;
		// printf("readlength=%d\n",ReadByte);
		// Buffer[ReadByte]='\0';
		// SplitString(Buffer, ",")
		if (readByte > 0) {
			// std::cout << readByte;
			dataBuffer.Charge((unsigned char *)buffer, readByte);
			dataBuffer.Discharge(segments);
			//std::cout << segments.size() << std::endl;
			for (std::vector<std::vector<unsigned char> >::iterator it = segments.begin(); it != segments.end(); it++) {
				std::string seg(it->begin(), it->end());
				imuSolver.Solve(*it);
				imuSolver.m_imuPackage.Output();
				// for (std::vector<unsigned char>::iterator i = it->begin(); i != it->end(); i++)
				// {
				//   printf("0x%02x ", *i);
				// }
				// std::cout << std::endl;
				// std::cout << seg << std::endl;
				//std::cout << it->size() << std::endl;
			}
		}
		// printf("%s",buffer);
		//usleep(50000);
		// else printf("Read data failure times=%d\n",j);
	}
	printf("Receive data finished!\n");
#endif
}

