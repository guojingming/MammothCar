#include "mammoth.h"
 
using namespace mammoth;

//#define APP MultidataGraberApp
//#define APP_NAME "MultidataGraberApp"

//#define APP Vel32and16ViewerApp
//#define APP_NAME "Vel32and16ViewerApp"

//#define APP KittiBinToPclApp
//#define APP_NAME "KittiBinToPclApp"

#define APP CanMultidataGraberApp
#define APP_NAME "CanMultidataGraberApp"

int main(int argc, char** argv){
	AppBase* app = (APP*)Reflex::create_app(APP_NAME);
	app->run(argc, argv);
 	system("pause");
} 