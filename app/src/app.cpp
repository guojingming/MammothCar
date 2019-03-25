#include "app.h"

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