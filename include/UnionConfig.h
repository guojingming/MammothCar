#pragma once

#ifdef WIN32
	#include <windows.h>
#else
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
	#include <sys/time.h>
	#include <unistd.h>
#endif


#define PI 3.1415926535 

namespace mammoth {
	namespace config {
		struct MyPoint2D{
			MyPoint2D(){ x = 0; y = 0; };
			MyPoint2D(const float in_x, const float in_y){ x = in_x; y = in_y; };
			float x;
			float y;
			MyPoint2D& operator = (const MyPoint2D& point){
				MyPoint2D newpoint(point.x, point.y);
				return newpoint;
			};

			MyPoint2D& operator + (const MyPoint2D& point){
				MyPoint2D newpoint(x + point.x, y + point.y);
				return newpoint;
			};
			
			float distance(MyPoint2D other){
				return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
			}
		};

	    struct MyPoint3D{
			MyPoint3D(){
				x = 0;
				y = 0;
				z = 0;
			}
			MyPoint3D(float input_x, float input_y, float input_z){
				x = input_x;
				y = input_y;
				z = input_z;
			}
			float x;
			float y;
			float z;
		};

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