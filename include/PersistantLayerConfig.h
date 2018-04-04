#pragma once

#include "UnionConfig.h"

//#include <mysql_connection.h>  
//#include <mysql_driver.h>  
//#include <statement.h>  

namespace mammoth {
	namespace config {
		class PersistantLayerConfig {
		public:
			static std::string mysql_address;
			static std::string mysql_user;
			static std::string mysql_password;
			static std::string mysql_database;
		};
	}
}