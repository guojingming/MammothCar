#pragma once

#include "api.h"

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <string>

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