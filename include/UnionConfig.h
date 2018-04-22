#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <map>
#include <algorithm>
#include <sstream>
#include <sys/types.h>
#include <chrono>
#include <fstream>
#include <xmmintrin.h>

#define PI 3.14159265357

namespace mammoth {
	namespace config {
		typedef struct {
			int x;
			int y;
		}MyPoint2D;
		typedef struct {
			float x;
			float y;
			float z;
		}MyPoint3D;

		typedef struct {
			float x;
			float y;
			float z;
			uint8_t b;
			uint8_t g;
			uint8_t r;
			uint8_t a;
		}XYZRGBA;
	}
}