#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h> 

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp" 

#include "UnionConfig.h"

#include "glViewer.h"
#include "DataFormat.h"
#include "StdUtilLayer.h"

#define PointType pcl::PointXYZRGBA

#include <vector>

#define USE_GLVIEWER

#include<iostream>

using namespace std;

namespace mammoth {
	namespace layer {
		struct FiledDesc {
			char Name[8];
			uint32_t Size = 0;
			uint32_t DataType = 0;
			uint32_t Count = 0;
			uint32_t Offset = 0;
			FiledDesc() {
				memset(Name, 0, 8);
			}
		};

		struct PCDHEADER {
			uint32_t MajorVersion = 0;
			uint32_t SubVersion = 0;

			size_t Width = 0;
			size_t Height = 0;
			size_t Points = 0;

			uint32_t FieldCount = 0;
			FiledDesc* Fields = NULL;

			uint32_t StructSize = 0;
			enum {
				UNKNOWN, BINARY, ASCII
			}DataFormat = UNKNOWN;
		};

		struct PCDFILE {
			PCDHEADER header;
			void* pData;
		};

		struct tagPCD {
			FILE* fp;
			bool DataWrited = false;
			glviewer::DataFormatDesc Desc;
			size_t TotalPoints;
			size_t WOffset, POffset;
		};
		struct tagPCD;
		typedef struct tagPCD* HPCD;

		class PcdUtil {
		public:
			static void pcdAllocHeader(uint32_t FiledCount, PCDHEADER* pHeader);
			static void pcdReleaseHeader(PCDHEADER* pHeader);
			static void pcdReadHeader(FILE* fp, PCDHEADER* pHeader);
			static void pcdLoad(FILE* fp, PCDFILE* File);
			static void pcdRelease(PCDFILE* f);
			static FiledDesc* pcdContains(PCDHEADER* pHeader, const char* field);

			static int read_pcd_file(const std::string pcd_file_path, pcl::PointCloud<PointType>::Ptr & cloud);
			static int read_pcd_file(const std::string pcd_file_path, PCDFILE* pcd_file);

			static void save_pcd_file(const std::string pcd_file_path, const pcl::PointCloud<PointType>::Ptr & cloud, short mode = 0);
			static void save_pcd_file(const std::string pcd_file_path, const PCDFILE * pcd_file, short mode = 0);
			template<typename CustomType>
			static void save_pcd_file(const std::string pcd_file_path, const PCDFILE * pcd_file, CustomType type, short mode = 0);

			static void trans_pcd_to_xyz(const std::string pcd_file_path, const std::string  xyz_file_path);


		
			static HPCD pcdOpen(const char* filename);
			static void pcdWrite(HPCD hpcd, glviewer::DataFormatDesc dsc, void* Arr, size_t Count);
			static void pcdClose(HPCD hp);
			template<typename T>
			static inline void pcdWrite(HPCD hpcd, T* arr, size_t count) { pcdWrite(hpcd, glviewer::GetDataFormat<T>(), arr, count); }
			static inline void pcdSave(PCDFILE* pcdFile, const char* fn) {
				HPCD hp = pcdOpen(fn);
				pcdWrite(hp, GetDataFormatDescFromPCD(&pcdFile->header), pcdFile->pData, pcdFile->header.Points);
				pcdClose(hp);
			}

		private:
			template<size_t sz>
			static void readLine(FILE* fp, char(&buffer)[sz]);
			static const char* skip_white(const char* line);
			static bool checkSig(const char * Line, const char* sig);
			template<typename UT>
			static const char* parse_int(const char* str, UT* val);
			static const char* parse_strid(const char* str, char* buf);
			static const char* parse_version(const char* str, uint32_t* Major, uint32_t* Sub);
			static const char* PcdUtil::parse_datatype(const char* str, int size, uint32_t* val);
			static uint32_t PcdUtil::parse_field_count(const char* str);
			static void calc_offset(PCDHEADER* pHeader);
			static bool check_header(PCDHEADER* pHeader);
			static void read_binary(FILE* fp, PCDFILE* File);
			static void read_ascii(FILE* fp, PCDFILE* File);
			static glviewer::DataFormatDesc GetDataFormatDescFromPCD(PCDHEADER* pHeader);
		};

		class PointViewer {
		public:
			void init_point_viewer();
			void set_point_cloud(const pcl::PointCloud<PointType>::Ptr & cloud);
			void set_point_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);
			void set_point_cloud(PCDFILE scene);
			void set_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
			template<typename CustomType>
			void set_point_cloud(PCDFILE scene, CustomType point_type);
			uint32_t add_cube(PointType * cube_points);
			void remove_cubes(std::vector<uint32_t>& cube_handles);
			static PointViewer* get_instance();
			size_t add_text(const char * str, int start_x, int start_y, float scale_rate, glviewer::Color4F color);
			void set_text(size_t id, const char * str, int start_x, int start_y, float scale_rate, glviewer::Color4F color);
			void print_camera_data();
		
		private:
			static glviewer::TextNode* p_node;
			static std::vector<size_t> text_ids;
			static PointViewer* p_viewer;
			static glviewer::GLDevice * p_glviewer;
			static void key_pressed(char key, bool state, void* ctx);
			static void selectResultHandle(glviewer::SelectResult<void*>* _Rx);
		};
	}
}