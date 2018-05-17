#pragma once
#include<sys/socket.h>
#include<sys/un.h>
#include<string>
#include<vector>
#include<iostream>
#include<variant>
#include<unistd.h>
#include<fcntl.h>

#define ALLOWED_TYPES void*,bool,char,int,long,float,double
template<typename ...>
struct is_one_of{static constexpr bool value=false;};

template<typename F,typename S, typename...T>
struct is_one_of<F,S,T...>{
	static constexpr bool value=std::is_same<F,S>::value||is_one_of<F,T...>::value;
};


template<typename T>
static constexpr char typeChar(){
	static_assert(is_one_of<T,ALLOWED_TYPES>::value,"Type not supported");
	return
	std::is_same<T,void*>::value?'a':
	std::is_same<T,bool>::value?'b':
	std::is_same<T,char>::value?'c':
	std::is_same<T,int>::value?'d':
	std::is_same<T,long>::value?'e':
	std::is_same<T,float>::value?'f':
	std::is_same<T,double>::value?'g':
	'z';
	}

//static char typeChars[]={'a', 'b', 'c', 'd', 'e', 'f', 'g'};
typedef std::pair<char,std::variant<ALLOWED_TYPES>> myvari;
typedef std::vector<myvari> vecvar;

struct varmes{
	std::string sender;
	std::string purpose;
	vecvar vars;
};

class BaseSocket{
	public:
	struct sockaddr_un addr;
	int fd;
	char buf[2048];

	BaseSocket(const std::string& addr);
	
	void handleSocketErr(int er);

	int setFlagFD(int fd,int flag);
	int unsetFlagFD(int fd,int flag);
	
	std::vector<std::string> receiveFD(int fd);

	virtual void handleReadErr(int er);

	int sendsFD(const std::string& msg,int fd);

	virtual int init()=0;

	template<typename... V>
	std::string prepareMessage(const char sender[2], const char type[2],V... v) const{
		std::string ss;
		ss+=sender[0];
		ss+=sender[1];
		ss+=type[0];
		ss+=type[1];
		this->prepareMessageRec(ss,v...);
		return ss;
	}

	template<typename T,typename... V>
	int prepareMessageRec(std::string& ss,T t,V...v)const{
		return prepareMessageType(ss,t)+prepareMessageRec(ss,v...);
	}

	template<typename T>
	int prepareMessageRec(std::string& ss,T t) const{
		return prepareMessageType(ss,t);
	}

	template<typename T>
	int prepareMessageTypeGen(std::string& ss, T& t,char c)const{
		ss+=c;
		char* cp=reinterpret_cast<char*>(&t);
		for(unsigned int i=0;i<sizeof(T);++i){
			ss+=cp[i];
		}
		return 1+sizeof(T);
	}

	template <typename T>
	int prepareMessageType(std::string& ss, T t) const{
		return prepareMessageTypeGen(ss,t,typeChar<T>());
	
	}

	varmes processMessage(const std::string& msg);
};

