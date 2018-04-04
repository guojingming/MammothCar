#pragma once
#ifndef glViewer_h__
#define glViewer_h__
#include <stdint.h>
#include <functional>
#include <stdio.h>
#include "DataFormat.h"

namespace glviewer {
	template<typename T>
	struct SelectResult {
	private:
		uint32_t* _Index;
		uint32_t _Count;

		T* _Values;
		uint32_t Stride;
	public:

		struct iterator {
		private:
			uint32_t* _Pt;
			T* _Value;
			uint32_t Stride;
		public:
			iterator(uint32_t *_pt, T* _value, uint32_t stride) :_Pt(_pt), _Value(_value), Stride(stride) {}

			const iterator& operator++() { _Pt++; return *this; }
			const iterator& operator++(int) { _Pt++; return *this; }
			const iterator& operator--(int) { _Pt--; return *this; }
			const iterator& operator-() { _Pt--; return *this; }
			T* operator->() { return (T*)((char*)_Value + (*_Pt) * Stride); }
			T& operator*() { return ((T*)((char*)_Value + Stride * *_Pt))[0]; }
			bool operator==(const typename SelectResult<T>::iterator& _It) const {
				return _Value == _It._Value && _Pt == _It._Pt;
			}
			bool operator!=(const typename SelectResult<T>::iterator& _It) const { return !(*this == _It); }
		};

		SelectResult(uint32_t* _index, uint32_t _count, uint32_t stride, void* _value) :_Index(_index), _Count(_count), _Values((T*)_value), Stride(stride) {}
		iterator begin() {
			return iterator{ _Index, _Values, Stride };
		}

		iterator end() {
			return iterator{ _Index + _Count, _Values, Stride };
		}

		size_t size() const {
			return _Count;
		}
	};

	enum DeviceParams {
		Camera_State,//float[6] Forward X Y Z Up X Y Z
		Camera_Position,//float[3] X,Y,Z

		Wave_Color,//normalized float[3] R G B
		Wave_ZPlane,// float[1] Z
	};


	class GLDevice {
	public:
		template<typename T>
		inline void SetPointCloud(T* Arr, uint32_t Count) {
			SetPointCloud(GetDataFormat<T>(), Arr, Count);
		}

		virtual void SetPointCloud(DataFormatDesc desc, void * Arr, uint32_t Count) = 0;

		virtual void Release() = 0;

		template<typename T>
		inline void SetOnPickingCallback(void(*_callback)(SelectResult<T>*)) {
			SetOnPickingCallback((void(*)(SelectResult<void*>* _R))_callback);
		}

		virtual void SetOnPickingCallback(void(*_C)(SelectResult<void*>* _R)) = 0;

		//key : 'a' - 'z' ' '(Space) F1 = 0x01 F2 = 0x02 ... F12 = 0x0C

		//state = false : KeyUp
		//state = true  : Key Down

		virtual void RegisterCallback(char key, void(*)(char key, bool state, void* ctx), void* ctx = NULL) = 0;

		template<typename T>
		uint32_t AddCubeRenderer(T* Arr) {
			return AddCubeRenderer(GetDataFormat<T>(), Arr);
		}

		//
		//        V7         V6
		//		   ----------
		//        |\        |\
				//        | \_______|_\ V5
		//        | V4      | |
		//      V3|_|_ _ _ _|V2
		//        \ |       \ |
		//         \|________\|
		//         V0         V1
		//
		virtual uint32_t AddCubeRenderer(DataFormatDesc desc, void* Data) = 0;

		template<typename T>
		void AppendCube(uint32_t _Rid, T* _Data) {
			_AppendCube(_Rid, GetDataFormat<T>(), _Data);
		}

		virtual void _AppendCube(uint32_t _Rid, DataFormatDesc _Format, void* _Data) = 0;

		virtual void RemoveRenderer(uint32_t Rid) = 0;

		virtual uint32_t AddWavePlane(float _Step, float _StartX, int _Count) = 0;
		virtual void Shutdown() = 0;

		virtual void Redraw() = 0;

		template<typename... _ValueTypes>
		inline void SetParam(DeviceParams params, const _ValueTypes& ... _Args) {
			float _XV[] = { (float)_Args... };
			SetParam(params, _XV);
		}

		template<size_t sz>
		void SetParam(DeviceParams params, float(&V)[sz]) {
			SetParam(params, V, sz);
		}

		virtual void SetParam(DeviceParams params, float _Value[], size_t C) = 0;

	};

}

glviewer::GLDevice* GetGLDevice(int argc, char** argv, const char* title);

#endif // glViewer_h__