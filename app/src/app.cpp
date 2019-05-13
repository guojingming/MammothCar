#include "app.h"
#include "mammoth.h"

using namespace mammoth;

static std::map<std::string, ClassInfo*> *m_classInfoMap = NULL;

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
	MultiDataGraber::get_instance()->start_grab("E:\\DataSpace\\can_multidata\\pcd_ori", "", "E:\\DataSpace\\can_multidata\\pic_ori", "");
	//MultiDataGraber::get_instance()->start_grab("D://20190507//pcd", "", "D://20190507//pic", "");
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
		sprintf(temp1, "E:/DataSpace/velodyne_point/training/velodyne/%06d.bin", i);
		sprintf(temp2, "E:/DataSpace/velodyne_point/training/velodyne/%06d.pcd", i);
		pcl::PointCloud<pcl::PointXYZI>::Ptr points = PcdUtil::trans_kittibin_to_pcd(string(temp1));
		pcl::io::savePCDFileBinary(string(temp2), *points);
	}
	
}

void KittiBinToPclApp::run(int argc, char ** argv) {
	run();
}

REGISTER_CLASS(CanMultidataGraberApp)

void CanMultidataGraberApp::run() {
	printf("CanMultidataGraberApp is running!\n");
	CanMultiDataGraber::get_instance()->start_grab("E:\\DataSpace\\can_multidata\\pcd", "E:\\DataSpace\\can_multidata\\pic");
}

void CanMultidataGraberApp::run(int argc, char ** argv) {
	run();
}