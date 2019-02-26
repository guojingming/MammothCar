#include "pcdutil.h"

using namespace mammoth::util;
using namespace glviewer;

int PcdUtil::read_pcd_file(const std::string pcd_file_path, pcl::PointCloud<PointType>::Ptr & cloud) {
	cloud->clear();
	if (pcl::io::loadPCDFile<PointType>(pcd_file_path, *cloud) == -1) {
		PCL_ERROR("can't read the pcd file\n");
		return (-1);
	}
	return 0;
}

int PcdUtil::read_pcd_file(const std::string pcd_file_path, PCDFILE* pcd_file) {
	FILE * p;
	fopen_s(&p, pcd_file_path.c_str(), "rb");
	PcdUtil::pcdLoad(p, pcd_file);
	fclose(p);
	return 0;
}

//0 binary 1 ascii
void PcdUtil::save_pcd_file(const std::string pcd_file_path, const pcl::PointCloud<PointType>::Ptr & cloud, short mode) {
	if (cloud->size() == 0) {
		printf("Ҫд�� %s ��PCD���������СΪ0��д�����ȡ��\n", pcd_file_path.c_str());
		return;
	}
	if (mode == 0) {
		pcl::io::savePCDFileBinary(pcd_file_path, *cloud);
	} else if (mode == 1) {
		pcl::io::savePCDFile(pcd_file_path, *cloud);
	}
}

template<typename CustomType>
static void save_pcd_file(const std::string pcd_file_path, const PCDFILE * pcd_file, CustomType type, short mode) {
	if (pcd_file.header.Points == 0) {
		printf("Ҫд�� %s ��PCD���������СΪ0��д�����ȡ��\n", pcd_file_path.c_str());
		return;
	}
	if (mode == 0) {
		//binary
	} else if (mode == 1) {
		//ascii
	}
}

void PcdUtil::save_pcd_file(const std::string pcd_file_path, const PCDFILE * pcd_file, short mode) {
	if (pcd_file->header.Points == 0) {
		printf("Ҫд�� %s ��PCD���������СΪ0��д�����ȡ��\n", pcd_file_path.c_str());
		return;
	}
	if (mode == 0) {
		//binary
		HPCD hpcd = PcdUtil::pcdOpen(pcd_file_path.c_str());
		PcdUtil::pcdWrite(hpcd, (XYZRGBA *)(pcd_file->pData), pcd_file->header.Points);
		PcdUtil::pcdClose(hpcd);
	} else if (mode == 1) {
		//ascii
	}
}


void PcdUtil::trans_pcd_to_xyz(const std::string pcd_file_path, const std::string xyz_file_path) {
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	read_pcd_file(pcd_file_path, cloud);
	char str[100];
	FileUtil file(xyz_file_path.c_str(), 2);
	for (int i = 0; i < cloud->size(); i++) {
		float x = (*cloud)[i].x;
		float y = (*cloud)[i].y;
		float z = (*cloud)[i].z;
		memset(str, 0, 100);
		sprintf(str, "%f %f %f", x, y, z);
		file.write_line(str);
	}
}

glviewer::GLDevice * PointViewer::p_glviewer = nullptr;
PointViewer* PointViewer::p_viewer = nullptr;

void PointViewer::print_camera_data() {
	CameraData d;
	float * data = (float *)d.unknown_data;
	p_glviewer->GetCameraData(&d);
	for (auto & i : d.unknown_data) {
		printf("0x%02X,",(unsigned int)i);
	}
}

void PointViewer::init_point_viewer() {
#ifdef USE_GLVIEWER
	p_glviewer->SetParam(glviewer::DeviceParams::Wave_ZPlane, -1.8f);
	p_glviewer->SetParam(glviewer::DeviceParams::POINT_SIZE, 1.0f);
	//float camera_data[9] = { 0.074241, 0.990729, -0.113753, 0.008500, 0.113435,0.993508, 0.015301,2.194579, -1.845818 };

	unsigned char b[64] = { 0xFF,0x4B,0x9B,0x3B,0x8D,0xF7,0x7C,0x3F,0xC7,0x12,0x1D,0xBE,0x0B,0xDD,0x40,0x3A,0x4F,0x12,0x1D,0x3E,0x39,0xF8,0x7C,0x3F,0xED,0x8A,0x93,0x3D,0x18,0x12,0x9C,0x40,0x81,0x21,0xF0,0xBF,0x00,0x00,0x80,0x40,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC };
	p_glviewer->SetCameraData((CameraData *)b);

	//p_glviewer->AddWavePlane(2, 0, 50);
	p_glviewer->SetOnPickingCallback(selectResultHandle);
	p_glviewer->RegisterCallback(' ', key_pressed);

	//init text
	for (int i = 0; i < 17; i++) {
		text_ids.push_back(add_text("0", 0, 0, 0.4f, {0.0f, 0.0f, 0.0f, 0.0f}));
	}

#endif
}

glviewer::TextNode* PointViewer::p_node;
std::vector<size_t> PointViewer::text_ids;

size_t PointViewer::add_text(const char * str, int start_x, int start_y, float scale_rate, glviewer::Color4F color) {
#ifdef USE_GLVIEWER
	if (p_node == nullptr) {
		p_node = p_glviewer->CreateTextNode();
	}
	size_t id = p_node->AddText(str, start_x, start_y, scale_rate, color);
	return id;
#else
	return 0;
#endif
}

void PointViewer::set_text(size_t id, const char * str, int start_x, int start_y, float scale_rate, glviewer::Color4F color) {
#ifdef USE_GLVIEWER
	if (p_node == nullptr) {
		p_node = p_glviewer->CreateTextNode();
	}
	if (id >= text_ids.size()) {
		id = text_ids.size() - 1;
	}
	p_node->UpdateText(text_ids[id], str);
	p_node->UpdateColor(text_ids[id], color);
	p_node->UpdatePosition(text_ids[id], start_x, start_y);
#endif
}


void PointViewer::set_point_cloud(PCDFILE scene) {
#ifdef USE_GLVIEWER
	struct point_type {
		float x;
		float y;
		float z;
		uint8_t b;
		uint8_t g;
		uint8_t r;
		uint8_t a;
	};
	p_glviewer->SetPointCloud((point_type *)scene.pData, scene.header.Points);
#endif
}

template<typename CustomType>
void PointViewer::set_point_cloud(PCDFILE scene, CustomType point_type) {
#ifdef USE_GLVIEWER
	p_glviewer->SetPointCloud((point_type *)scene.pData, scene.header.Points);
#endif
}

void PointViewer::set_point_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud) {
#ifdef USE_GLVIEWER
	p_glviewer->SetPointCloud(&((*cloud)[0]), cloud->size());
#endif
}

void PointViewer::set_point_cloud(const pcl::PointCloud<PointType>::Ptr & cloud) {
#ifdef USE_GLVIEWER
	p_glviewer->SetPointCloud(&((*cloud)[0]), cloud->size());
#endif
}


void PointViewer::set_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud) {
#ifdef USE_GLVIEWER
	p_glviewer->SetPointCloud(&((*cloud)[0]), cloud->size());
#endif
}

void PointViewer::selectResultHandle(glviewer::SelectResult<void*>* _Rx) {
	auto * _R = (glviewer::SelectResult<PointType>*)_Rx;
	int count = 0;
	std::cout << std::endl << "----------------------------------------------" << std::endl;
	float min_z = 100;
	float average_z = 0;
	float max_z = -100;
	for (glviewer::SelectResult<PointType>::iterator it = _R->begin(); it != _R->end(); it++) {
		if (count >= 1) {
			if (it->z <= min_z) {
				min_z = it->z;
			}
			if (it->z >= max_z) {
				max_z = it->z;
			}
			average_z += it->z;
		}

		if (count < 10 && count >= 1) {
			std::cout << "[" << count << "]   " << "x:" << it->x << " y:" << it->y << " z:" << it->z << std::endl;
		} else if (count >= 10 && count < 100) {
			std::cout << "[" << count << "]  " << "x:" << it->x << " y:" << it->y << " z:" << it->z << std::endl;
		} else if (count >= 100) {
			std::cout << "[" << count << "] " << "x:" << it->x << " y:" << it->y << " z:" << it->z << std::endl;
		}
		count++;
	}
	/*average_z /= (count - 1);
	std::cout << "minZ: " << min_z << std::endl;
	std::cout << "maxZ: " << max_z << std::endl;
	std::cout << "averageZ: " << average_z << std::endl;
	std::cout << "----------------------------------------------" << std::endl;*/
	//std::cout << _R->size() << std::endl;
}


void PointViewer::key_pressed(char key, bool state, void* ctx) {
	if (state == true) {
		//printf("down %d\n", key);
	} else {
		//printf("up %d\n", key);
		/*cv::imshow("show", img);
		cv::waitKey(1);*/
	}
}

uint32_t PointViewer::add_cube(PointType * cube_points) {
#ifdef USE_GLVIEWER
	return p_glviewer->AddCubeRenderer(cube_points);
#else
	return 0;
#endif
}

void PointViewer::remove_cubes(std::vector<uint32_t>& cube_handles) {
#ifdef USE_GLVIEWER
	for (int i = 0; i < cube_handles.size(); i++) {
		p_glviewer->RemoveRenderer(cube_handles[i]);
	}
	cube_handles.clear();
#endif
}

PointViewer* PointViewer::get_instance() {
#ifdef USE_GLVIEWER
	if (p_glviewer == nullptr) {
		p_glviewer = GetGLDevice(0, NULL, "Point Cloud Window");
	}
#endif
	if (p_viewer == nullptr) {
		p_viewer = new PointViewer();
	}
	return p_viewer;
}

////////////////////////////////////////////////
template<size_t sz>
void PcdUtil::readLine(FILE* fp, char(&buffer)[sz]) {
	size_t n = 0;
	size_t rd = fread(buffer, 1, sz - 1, fp);
	while (buffer[n] != '\n' && n < rd)n++;
	if (n == rd) {
		buffer[0] = 0;
		n = 0;
	} else {
		n++;
	}
	fseek(fp, (long)(n - rd), SEEK_CUR);
	buffer[n] = 0;
}

const char* PcdUtil::skip_white(const char* line) {
	while ((*line) == ' ' || (*line) == '\t' || (*line) == '\r')line++;
	return line;
}

bool PcdUtil::checkSig(const char * Line, const char* sig) {
	return strncmp(Line, sig, strlen(sig)) == 0;
}

template<typename UT>
const char* PcdUtil::parse_int(const char* str, UT* val) {
	*val = 0;
	while ((*str) >= '0' && (*str) <= '9') {
		*val = *val * 10 + (*str) - '0';
		str++;
	}
	return str;
}

const char* PcdUtil::parse_strid(const char* str, char* buf) {
	while (*str >= 'a' && *str <= 'z' || *str >= 'A' && *str <= 'Z' || *str == '_') {
		*buf++ = *str++;
	}
	*buf = 0;
	return str;
}
const char* PcdUtil::parse_version(const char* str, uint32_t* Major, uint32_t* Sub) {
	str = parse_int(str, Major);
	str++;//skip '.'
	str = parse_int(str, Sub);
	return str;
}
const char* PcdUtil::parse_datatype(const char* str, int size, uint32_t* val) {
	switch (*str) {
	case 'U':
		switch (size) {
		case 1:
			*val = _helpers::_DataType<unsigned char>::value; break;
		case 2:
			*val = _helpers::_DataType<unsigned short>::value; break;
		case 4:
			*val = _helpers::_DataType<unsigned int>::value; break;
		default:
			*val = 0;
			break;
		}
		break;
	case 'F':
		*val = _helpers::_DataType<float>::value;
		break;
	case 'I':
		switch (size) {
		case 1:
			*val = _helpers::_DataType<char>::value; break;
		case 2:
			*val = _helpers::_DataType<short>::value; break;
		case 4:
			*val = _helpers::_DataType<int>::value; break;
		default:
			*val = 0;
			break;
		}
		break;
	default:
		*val = 0;
		break;
	}
	return str + 1;
}
uint32_t PcdUtil::parse_field_count(const char* str) {
	str = skip_white(str);
	char buf[128];
	const char* res = parse_strid(str, buf);
	int n = 0;
	while (res != str) {
		n++;
		str = skip_white(res);
		res = parse_strid(str, buf);
	}
	return n;
}
void PcdUtil::calc_offset(PCDHEADER* pHeader) {
	using T = FiledDesc;
	uint32_t Offset = 0;
	for (T* i = pHeader->Fields; i != pHeader->Fields + pHeader->FieldCount; i++) {
		i->Offset = 0;
		Offset += i->Size * i->Count;
	}
	pHeader->StructSize = Offset;
}

bool PcdUtil::check_header(PCDHEADER* pHeader) {
	if (pHeader->DataFormat == PCDHEADER::UNKNOWN) {
		return false;
	}
	if (pHeader->Points == 0 || pHeader->Width * pHeader->Height != pHeader->Points) {
		return false;
	}
	if (pHeader->FieldCount == 0 || pHeader->Fields == NULL)
		return false;
	for (uint32_t i = 0; i < pHeader->FieldCount; i++) {
		if (pHeader->Fields[i].Count == 0 || pHeader->Fields[i].DataType == 0 || pHeader->Fields[i].Size == 0)
			return false;
	}
	return true;
}

void PcdUtil::pcdAllocHeader(uint32_t FiledCount, PCDHEADER* pHeader) {
	if (pHeader != NULL) {
		pHeader->MajorVersion = 0;
		pHeader->SubVersion = 7;
		pHeader->FieldCount = FiledCount;
		if (FiledCount != 0)
			pHeader->Fields = new FiledDesc[FiledCount];
		else
			pHeader->Fields = NULL;
		pHeader->Height = 0;
		pHeader->Points = 0;
		pHeader->Width = 0;

		for (uint32_t i = 0; i < FiledCount; i++) {
			pHeader->Fields[i].Count = 0;
			pHeader->Fields[i].DataType = 0;
			pHeader->Fields[i].Name[0] = 0;
			pHeader->Fields[i].Size = 0;
		}
	}
}
void PcdUtil::pcdReleaseHeader(PCDHEADER* pHeader) {
	if (pHeader && pHeader->Fields)
		delete[] pHeader->Fields;
	memset(pHeader, 0, sizeof(PCDHEADER));
}

void PcdUtil::pcdReadHeader(FILE* fp, PCDHEADER* pHeader) {
	char buffer[128];
	while (true) {
		readLine(fp, buffer);
		const char* buf = skip_white(buffer);
		//ignore
		if (checkSig(buf, "#")) {
			continue;
		}

		if (checkSig(buf, "VERSION")) {
			//read Version
			buf += 8;
			buf = skip_white(buf);
			parse_version(buf, &pHeader->MajorVersion, &pHeader->SubVersion);
			continue;
		}

		if (checkSig(buf, "FIELDS")) {
			buf = skip_white(buf + 7);
			//?????
			int nCount = parse_field_count(buf);
			if (nCount == 0)// No Data Field,invalid
				return;
			pHeader->FieldCount = nCount;
			pHeader->Fields = new FiledDesc[nCount];

			for (int i = 0; i < nCount; i++) {
				buf = skip_white(parse_strid(buf, pHeader->Fields[i].Name));
			}
			continue;
		}

		if (checkSig(buf, "SIZE")) {
			buf = skip_white(buf + 4);
			for (uint32_t i = 0; i < pHeader->FieldCount; i++) {
				buf = skip_white(parse_int(buf, &pHeader->Fields[i].Size));
			}
			continue;
		}

		if (checkSig(buf, "TYPE")) {
			buf = skip_white(buf + 4);
			for (uint32_t i = 0; i < pHeader->FieldCount; i++) {
				buf = skip_white(parse_datatype(buf, pHeader->Fields[i].Size, &pHeader->Fields[i].DataType));
			}
			continue;
		}
		if (checkSig(buf, "COUNT")) {
			buf = skip_white(buf + 5);
			for (uint32_t i = 0; i < pHeader->FieldCount; i++) {
				buf = skip_white(parse_int(buf, &pHeader->Fields[i].Count));
			}
			continue;
		}
		if (checkSig(buf, "WIDTH")) {
			buf = skip_white(buf + 5);
			parse_int(buf, &pHeader->Width);
			continue;
		}
		if (checkSig(buf, "HEIGHT")) {
			buf = skip_white(buf + 6);
			parse_int(buf, &pHeader->Height);
			continue;
		}
		if (checkSig(buf, "VIEWPOINT")) {
			continue;
		}
		if (checkSig(buf, "POINTS")) {
			buf = skip_white(buf + 6);
			parse_int(buf, &pHeader->Points);
			continue;
		}
		if (checkSig(buf, "DATA")) {
			buf = skip_white(buf + 4);
			if (checkSig(buf, "binary")) {
				pHeader->DataFormat = PCDHEADER::BINARY;
			} else if (checkSig(buf, "ascii")) {
				pHeader->DataFormat = PCDHEADER::ASCII;
			}
			break;
		}
	}
	if (check_header(pHeader)) {
		calc_offset(pHeader);
	} else {
		pcdReleaseHeader(pHeader);
	}
}

void PcdUtil::read_binary(FILE* fp, PCDFILE* File) {
	size_t fsz = File->header.StructSize * File->header.Points;
	File->pData = malloc(fsz);
	size_t frd = fread(File->pData, fsz, 1, fp);
	if (frd != 1) {
		//gg
	}
}
void PcdUtil::read_ascii(FILE* fp, PCDFILE* File) {
}

void PcdUtil::pcdLoad(FILE* fp, PCDFILE* File) {
	pcdReadHeader(fp, &File->header);
	switch (File->header.DataFormat) {
	case PCDHEADER::BINARY:
		//read binary;
		read_binary(fp, File);
		break;
	case PCDHEADER::ASCII:
		read_ascii(fp, File);
		break;
	default:
		break;
	}
}

void PcdUtil::pcdRelease(PCDFILE* f) {
	free(f->pData);
	pcdReleaseHeader(&f->header);
}

FiledDesc* PcdUtil::pcdContains(PCDHEADER* pHeader, const char* field) {
	using T = FiledDesc;
	for (T* i = pHeader->Fields; i != pHeader->Fields + pHeader->FieldCount; i++) {
		if (strcmp(field, i->Name) == 0)return i;
	}
	return NULL;
}

HPCD PcdUtil::pcdOpen(const char* filename) {
	HPCD p = new tagPCD;
	fopen_s(&p->fp, filename, "wb");
	p->DataWrited = false;
	p->TotalPoints = 0;
	return p;
}

void PcdUtil::pcdWrite(HPCD hpcd, glviewer::DataFormatDesc dsc, void* Arr, size_t Count) {
	if (!hpcd->DataWrited) {
		//build header
		const char* comment = "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH                        \nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS                        \nDATA binary\n";
		fwrite(comment, strlen(comment), 1, hpcd->fp);
		hpcd->Desc = dsc;
		hpcd->POffset = 119;
		hpcd->WOffset = 183;
		hpcd->DataWrited = true;
	}
	if (dsc != hpcd->Desc) {
		return;
	}
	fwrite(Arr, dsc.StructSize, Count, hpcd->fp);
	hpcd->TotalPoints += Count;
}


void PcdUtil::pcdClose(HPCD hp) {
	char W[24];
	sprintf_s(W, "%llu", hp->TotalPoints);
	fseek(hp->fp, hp->POffset, SEEK_SET);
	fwrite(W, strlen(W), 1, hp->fp);
	fseek(hp->fp, hp->WOffset, SEEK_SET);
	fwrite(W, strlen(W), 1, hp->fp);
	fclose(hp->fp);
	delete hp;
}


glviewer::DataFormatDesc PcdUtil::GetDataFormatDescFromPCD(PCDHEADER* pHeader) {
	glviewer::DataFormatDesc desc;
	FiledDesc* fdesc;
	if (NULL == (fdesc = pcdContains(pHeader, "r"))) {
		desc.bColorChannel = true;
		desc.ColorDataFormat = fdesc->DataType;
		desc.ColorOffset = fdesc->Offset;
	} else if (NULL == (fdesc = pcdContains(pHeader, "rgb"))) {
		desc.bColorChannel = true;
		desc.ColorDataFormat = _helpers::_DataType<unsigned char>::value;
		desc.ColorOffset = fdesc->Offset;
	} else {
		desc.bColorChannel = false;
		desc.ColorDataFormat = 0;
		desc.ColorOffset = 0;
	}
	fdesc = pcdContains(pHeader, "x");
	desc.PosDataFormat = fdesc->DataType;
	desc.PositionOffset = fdesc->Offset;
	desc.StructSize = pHeader->StructSize;
	return desc;
}

