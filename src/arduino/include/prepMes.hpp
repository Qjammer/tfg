#pragma once

namespace std{
#ifndef _GLIBCXX_TYPE_TRAITS
	template<typename _Tp, _Tp __v>
	struct integral_constant{
		static constexpr _Tp value = __v;
		constexpr operator _Tp() const noexcept{return value;}
	};

	typedef integral_constant<bool, false> false_type;
	typedef integral_constant<bool, true> true_type;

	template<typename,typename> struct is_same:public false_type{};
	template<typename T> struct is_same<T,T>:public true_type{};
#endif

#ifndef _GLIBCXX_STRING
	class string:public String{
		public:
		unsigned long size(){
			return this->length();
		}
		unsigned char concat(const char *cstr, unsigned int length){
			unsigned int newlen = len + length;
			if (!cstr) return 0;
			if (length == 0) return 1;
			if (!reserve(newlen)) return 0;
			memcpy(buffer + len, cstr,length);
			buffer[newlen]='\0';
			len = newlen;
			return 1;
		
		}
		
		string& copy(const char *cstr, unsigned int length){
			if (!reserve(length)) {
				invalidate();
				return *this;
			}
			len = length;
			memcpy(buffer, cstr,length);
			return *this;
		}
	};
#endif
}
//#include"../../include/protocolHandler.hpp"
#include"protocolHandler.hpp"

