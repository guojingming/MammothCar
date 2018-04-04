#pragma once
#ifndef GG_h__
#define GG_h__

typedef unsigned char ubyte;
void Initialize(int argc,char* argv[],const char* title);

size_t AddObject(const char* ObjFileName);

void SetRotate(size_t ObjID,float Yaw, float Roll, float Pitch);
void SetTranslate(size_t ObjID, float dx, float dy, float dz);

void SetColor(size_t ObjID, ubyte R, ubyte G, ubyte B);

void SetCamera(
	float EyeAtX, float EyeAtY, float EyeAtZ,
	float LookAtX, float LookAtY, float LookAtZ,
	float UpX, float UpY, float UpZ
);

#endif // GG_h__