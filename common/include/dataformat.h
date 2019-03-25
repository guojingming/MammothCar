#pragma once
#ifndef DataFormat_h__
#define DataFormat_h__

#include <stdint.h>
#include <vector>

#ifndef _offsetof
#define _offsetof(T,x) ((size_t)(&((T*)0)->x))
#endif
namespace glviewer
{
	struct DataFormatDesc
	{
		size_t StructSize = 0;
		size_t PositionOffset = 0;
		size_t ColorOffset = 0;
		size_t ColorDataFormat = 0;
		size_t PosDataFormat = 0;
		bool bColorChannel = false;
		bool bIsBGR = false;
		bool operator>(const DataFormatDesc& _Right) const {
#define CMP(x) if (_Right.x != x)return x > _Right.x;
			CMP(StructSize);
			CMP(PosDataFormat);
			CMP(ColorOffset);
			CMP(ColorDataFormat);
			CMP(PosDataFormat);
			CMP(bColorChannel);
			CMP(bIsBGR);
#undef CMP
			return false;
		}

		bool operator==(const DataFormatDesc& _Right) const {
#define CMP(x) if (_Right.x != x)return false;
		CMP(StructSize);
		CMP(PosDataFormat);
		CMP(ColorOffset);
		CMP(ColorDataFormat);
		CMP(PosDataFormat);
		CMP(bColorChannel);
		CMP(bIsBGR);
#undef CMP
		return true;
		}
		bool operator!=(const DataFormatDesc& _Right) const {
			return !operator==(_Right);
		}
	};

	namespace _helpers {
		template<typename> struct _DataType { static const size_t value = 0x00000000; };
		template<> struct _DataType<float> { static const size_t value = 0x00040001; };
		template<> struct _DataType<char> { static const size_t value = 0x00010002; };
		template<> struct _DataType<int> { static const size_t value = 0x00040002; };
		template<> struct _DataType<unsigned char> { static const size_t value = 0x00010012; };
		template<> struct _DataType<unsigned int> { static const size_t value = 0x00040012; };
		template<> struct _DataType<double> { static const size_t value = 0x00080002; };
		template<> struct _DataType<unsigned short> { static const size_t value = 0x00020011; };
		template<> struct _DataType<short> { static const size_t value = 0x00020001; };
		template<typename T>
		struct is_member_contains_r
		{
			template<typename U> struct matcher;
			template<typename V> static char test_func(matcher<decltype(V::r)>*) { return ' '; }
			template<typename V> static int test_func(...) { return 0; }
			static const bool value = (sizeof(test_func<T>(0)) == sizeof(char));
		};

		template<typename T, bool _False_Type>
		struct _DataFormat_Helper{
			static DataFormatDesc Get() {
				DataFormatDesc desc;
				desc.bColorChannel = false;
				desc.ColorOffset = 0;
				desc.ColorDataFormat = 0;
				desc.bIsBGR = false;
				return desc;
			}
		};
		template<typename T>
		struct _DataFormat_Helper<T, true>{
			static DataFormatDesc Get() {
				DataFormatDesc desc;
				desc.bColorChannel = true;
				desc.ColorDataFormat = _helpers::_DataType<decltype(T::r)>::value;
				desc.bIsBGR = _offsetof(T, b) < _offsetof(T, r);
				if (desc.bIsBGR)
					desc.ColorOffset = _offsetof(T, b);
				else
					desc.ColorOffset = _offsetof(T, r);
				return desc;
			}
		};
	}
	inline size_t _Get_size(size_t _Type) { return _Type >> 16; }
	template<typename _FwdIt, typename F>
	void _FillColor(_FwdIt  _First, _FwdIt _End, F R, F G, F B) {
		for (_FwdIt _It = _First; _It != _End; _It++) {
			(*_It).r = R;
			(*_It).g = G;
			(*_It).b = B;
		}
	}
	template<typename T>
	DataFormatDesc GetDataFormat() {
		DataFormatDesc desc = _helpers::_DataFormat_Helper<T, _helpers::is_member_contains_r<T>::value>::Get();
		desc.StructSize = sizeof(T);
		desc.PositionOffset = _offsetof(T, x);
		desc.PosDataFormat = _helpers::_DataType<decltype(T::x)>::value;
		return desc;
	}
}
#endif // DataFormat_h__