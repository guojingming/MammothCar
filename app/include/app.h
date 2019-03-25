#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <map>

namespace mammoth{

    class ClassInfo;
    class Reflex{
    public:
        Reflex() {}
        virtual ~Reflex() {}
        //hash表注册
        static bool Register(ClassInfo* pCInfo);
        static Reflex* create_app(std::string className);
    };

    typedef Reflex* (*ObjConstructorFun)();
    class ClassInfo{
    public:
        ClassInfo(const std::string className, ObjConstructorFun classConstructor)
            :m_className(className), m_objectConstructor(classConstructor)
        {
            //classInfo的构造函数是传入类名和类对应的new函数然后自动注册进map中
            Reflex::Register(this);         
        }
        virtual ~ClassInfo() {}
        Reflex* create_app()const { return m_objectConstructor ? (*m_objectConstructor)() : NULL; }
        bool IsDynamic()const { return NULL != m_objectConstructor; }
        const std::string GetClassName()const { return m_className; }
        ObjConstructorFun GetConstructor()const { return m_objectConstructor; }
    public:
        std::string m_className;
        ObjConstructorFun m_objectConstructor;
    };

    #define DECLARE_CLASS(class_name) \
    public:\
        virtual ClassInfo* get_classinfo() const { return &m_classInfo; }\
        static Reflex* create_app()\
        {\
            return new class_name;\
        }\
    protected:\
        static ClassInfo m_classInfo;

    //新申明类ClassInfo注册
    #define REGISTER_CLASS(class_name)\
        ClassInfo class_name::m_classInfo(#class_name, class_name::create_app);

    //利用自写反射生成类对象
    #define REFLEX_CLASS(class_name)\
        (class_name*)(Reflex::create_app(#class_name))


    class AppBase{
    public:
        virtual void run() = 0;
        virtual void run(int argc, char ** argv) = 0;
        ~AppBase(){
            if(app_names.size()!=0){
                for(auto it = app_byname.begin(); it != app_byname.end(); it++){
                    delete it->second;
                }
            }
        };
        static AppBase* get_app_instance(std::string app_name){
            return app_byname[app_name];
        };
        static std::vector<std::string> get_app_names(){
            return app_names;
        }
    protected:
        AppBase(){
            if(app_names.size() == 0){
                app_names.push_back("MultidataGraberApp");
                app_names.push_back("Vel32and16ViewerApp");
            }
        }
        static std::vector<std::string> app_names;
        static std::unordered_map<std::string, AppBase*> app_byname;
    };

    class MultidataGraberApp : public AppBase, public Reflex{
    public:
        virtual void run();
        virtual void run(int argc, char ** argv){
            run();
        };
    };

    class Vel32and16ViewerApp : public AppBase, public Reflex{
    public:
        virtual void run();
        virtual void run(int argc, char ** argv){
            run();
        };
    };

}