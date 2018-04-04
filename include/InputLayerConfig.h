#pragma once

#include "UnionConfig.h"

#include "StdUtilLayer.h"
#include <iomanip>


namespace mammoth
{
	namespace layer
	{
		struct GPSPackage;
		class GnssEthernetInput;
		typedef bool (GnssEthernetInput::*SolverFunction)(std::vector<std::string>& token, GPSPackage & package);
		typedef std::map<std::string, SolverFunction> SolverFunctionMap;

#define RegisterSolver(header) \
{ \
    m_solverFunctionMap[#header] = &GnssEthernetInput::_##header##Solver; \
}
	}
}



struct StringFinder {
	StringFinder(const std::string& str) : m_str(str) {}
	bool operator()(const std::string& pattern) {
		return (m_str.find(pattern) != std::string::npos);
	}
	std::string m_str;
};



namespace mammoth {
	namespace config {
		class InputLayerConfig {
		public:
			static float gyroscope_x_diff;
			static float gyroscope_y_diff;
			static float gyroscope_z_diff;
			static float gyroscope_x_factor;
			static float gyroscope_y_factor;
			static float gyroscope_z_factor;
		};
	}
}