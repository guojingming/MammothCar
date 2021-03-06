#pragma once

#include "header.h"

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
            :m_className(className), m_objectConstructor(classConstructor){
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
		virtual void run() {
			printf("AppBase");
		};
		virtual void run(int argc, char ** argv) {
			run();
		}
        ~AppBase(){

        };
    protected:
        AppBase(){
            
        }
    };

    class MultidataGraberApp : public AppBase, public Reflex{
    public:
        virtual void run();
		virtual void run(int argc, char ** argv);
		DECLARE_CLASS(MultidataGraberApp)
    };

    class Vel32and16ViewerApp : public AppBase, public Reflex{
    public:
		virtual void run();
		virtual void run(int argc, char ** argv);
		DECLARE_CLASS(Vel32and16ViewerApp)
    };

	class KittiBinToPclApp : public AppBase, public Reflex {
	public:
		virtual void run();
		virtual void run(int argc, char ** argv);
		DECLARE_CLASS(KittiBinToPclApp)
	};

	class CanMultidataGraberApp : public AppBase, public Reflex {
	public:
		virtual void run();
		virtual void run(int argc, char ** argv);
		DECLARE_CLASS(CanMultidataGraberApp)
	};
}