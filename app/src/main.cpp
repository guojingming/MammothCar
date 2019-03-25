#include <iostream>
#include "app.h"

using namespace mammoth;

int main(int argc, char** argv){
    std::vector<std::string> app_names = AppBase::get_app_names();
    for(int i = 0; i < app_names.size(); i++){
        std::cout << i << ": " << app_names[i] << std::endl; 
    }
    int app_index = -1;
    while(app_index < 0 || app_index >= app_names.size()){
        std::cout<< "Please input the app's index whitch you want to run: ";
        std::cin >> app_index;
    }
    std::string app_name = app_names[app_index];
    std::cout<< "Creating " << app_name << " app obj." << std::endl;
    AppBase* app = AppBase::get_app_instance(app_name);
    if(app == nullptr){
        std::cout<< "Creating " << app_name << " failed! No registered app class named " << app_name << "." << std::endl;
        system("pause");
    }else{
        std::cout<< "Start running " << app_name << "." << std::endl;
        app->run(argc, argv);
        system("pause");
        delete app;
    }
}