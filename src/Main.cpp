#include "Mammoth.h"

using namespace mammoth::layer;
using Eigen::MatrixXd;

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
	int frame_count = 1;
	std::vector<pcl::PointCloud<PointType>::Ptr> vec;
	for (int i = 0; i < frame_count; i++) {
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
		vec.push_back(cloud);
	}
	PcapTransformLayer::get_instance()->trans_pcap_to_pcd("D:\\Download\\8.pcap", vec, 1);
	printf("%d\n", vec[0]->size());

	PcdUtil::save_pcd_file("D:\\1.pcd", vec[0]);

	system("pause");
}


void gjm_test() {

	//MultipleLidarViewer::showVelodyne16and32Points(0,1);

	//�µ�pcd��ȡ����
	//PointViewer::get_instance()->init_point_viewer();
	/*PCDFILE f;
	PcdUtil::read_pcd_file("C:\\DataSpace\\map\\0406-1.pcd", &f);
	PointViewer::get_instance()->set_point_cloud(f);
	system("pause");*/

	//��¼����
	//PointViewer::get_instance()->init_point_viewer();
	//DataGatherLayer::get_instance()->start_grab("E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\gps_data", "E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\pcd_data", "E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\imu_data", "E:\\DataSpace\\LidarDataSpace\\test_lidar_20180529\\ori_imu_data");
	
	//�����㷨
	PointViewer::get_instance()->init_point_viewer();
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	
	//PcdUtil::read_pcd_file("D:\\t3.pcd", cloud);
	PcdUtil::read_pcd_file("D:\\2.pcd", cloud);
	for (int i = 0; i < cloud->size(); i++) {
		
			(*cloud)[i].g = 255;
			(*cloud)[i].r = 255;
			(*cloud)[i].b = 255;

	}


	//DimensionReductionCluster::start_clusting(cloud);
	//printf("%d\n", cloud->size());
	PointViewer::get_instance()->set_point_cloud(cloud);
	system("pause");
	/*
	pcl::fromPCLPointCloud2();*/
	//PCDתXYZ
	//PcdUtil::trans_pcd_to_xyz("D:/map.pcd","D:/map.xyz");
	

	//SLAM
	//JluSlamLayer::get_instance()->start_slam("E:\\DataSpace\\LidarDataSpace\\new_lidar_20180226-1\\gps_data","E:\\DataSpace\\LidarDataSpace\\new_lidar_20180226-1\\pcd_data");
} 

void byz_test() {
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
}

