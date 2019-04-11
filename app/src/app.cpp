#include "app.h"

using namespace mammoth;

static std::map<std::string, ClassInfo*> *m_classInfoMap = NULL;

std::vector<std::string> AppBase::app_names;
std::unordered_map<std::string, AppBase*> AppBase::app_byname;

bool Reflex::Register(ClassInfo* pCInfo){
    if (!m_classInfoMap){
        m_classInfoMap = new std::map<std::string, ClassInfo*>();
    }
    if (!pCInfo){
        return false;
    }
    if (m_classInfoMap->end() == m_classInfoMap->find(pCInfo->m_className)){
        m_classInfoMap->insert(std::map<std::string, ClassInfo*>::value_type(pCInfo->m_className, pCInfo));
    }
    return true;
}

Reflex* Reflex::create_app(std::string className){
    std::map<std::string, ClassInfo*>::const_iterator c_iter = m_classInfoMap->find(className);
    if (m_classInfoMap->end() != c_iter){
        return c_iter->second->create_app();
    }
    return nullptr;
}

REGISTER_CLASS(MultidataGraberApp)

void MultidataGraberApp::run() {
	printf("MultidataGraberApp is running!\n");
	//MultiDataGraber::get_instance()->start_grab("E:\\DataSpace\\multidata2019\\pcd_create", "E:\\DataSpace\\multidata2019\\gps_create", "E:\\DataSpace\\multidata2019\\pic_create", "E:\\DataSpace\\multidata2019\\imu");
}

void MultidataGraberApp::run(int argc, char ** argv) {
	run();
}

REGISTER_CLASS(Vel32and16ViewerApp)

void Vel32and16ViewerApp::run() {
	printf("Vel32and16ViewerApp is running!\n");
}

void Vel32and16ViewerApp::run(int argc, char ** argv) {
	run();
}

REGISTER_CLASS(KittiBinToPclApp)

void KittiBinToPclApp::run() {
	char temp1[100];
	char temp2[100];
	for (int i = 0; i < 7481; i++) {
		memset(temp1, 0, 100);
		memset(temp2, 0, 100);
		sprintf(temp1, "E:/DataSpace/point/training/velodyne/%06d.bin", i);
		sprintf(temp2, "E:/DataSpace/point/training/velodyne/%06d.pcd", i);
		pcl::PointCloud<pcl::PointXYZI>::Ptr points = PcdUtil::trans_kittibin_to_pcd(string(temp1));
		pcl::io::savePCDFileBinary(string(temp2), *points);
	}
	
}

void KittiBinToPclApp::run(int argc, char ** argv) {
	run();
}

