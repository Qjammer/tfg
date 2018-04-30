#pragma once
#include<sys/socket.h>
#include<sys/un.h>
#include<string>
#include<vector>
#include<iostream>
#include<map>
#include<sstream>
#include<unistd.h>
#include<fcntl.h>

template<typename ...>
struct is_one_of{static constexpr bool value=false;};

template<typename F,typename S, typename...T>
struct is_one_of<F,S,T...>{
	static constexpr bool value=std::is_same<F,S>::value||is_one_of<F,T...>::value;
};

template<typename T>
static constexpr char typeChar(){
	static_assert(is_one_of<T,void*,bool,char,int,long,float,double>::value,"Type not supported");
	return
	std::is_same<T,void*>::value?'a':
	std::is_same<T,bool>::value?'b':
	std::is_same<T,char>::value?'c':
	std::is_same<T,int>::value?'d':
	std::is_same<T,long>::value?'e':
	std::is_same<T,float>::value?'f':
	std::is_same<T,double>::value?'g':
	'z';
	};

class Socket{
	public:
	struct sockaddr_un addr;
	int fd;
	char buf[2048];

	Socket(const std::string& addr);
	
	void handleSocketErr(int er);

	int setFlagFD(int fd,int flag);
	int unsetFlagFD(int fd,int flag);
	
	std::vector<std::string> receiveFD(int fd);

	virtual void handleReadErr(int er);

	int sendsFD(const std::string& msg,int fd);

	virtual int init()=0;

	template<typename... V>
	std::string prepareMessage(const char sender[2], const char type[2],V... v){
		std::string ss;
		ss+=sender[0];
		ss+=sender[1];
		ss+=type[0];
		ss+=type[1];
		int cd=prepareMessageRec(ss,v...);
		return ss;
	}

	template<typename T,typename... V>
	int prepareMessageRec(std::string& ss,T t,V...v){
		return prepareMessageType(ss,t)+prepareMessageRec(ss,v...);
		
	}

	template<typename T>
	int prepareMessageRec(std::string& ss,T t){
		return prepareMessageType(ss,t);
	}

	template<typename T>
	int prepareMessageTypeGen(std::string& ss, T& t,char c){
		ss+=c;
		char* cp=reinterpret_cast<char*>(&t);
		for(int i=0;i<sizeof(T);++i){
			ss+=cp[i];
		}
		return 1+sizeof(T);
	}

	template <typename T>
	int prepareMessageType(std::string& ss, T t){
		return prepareMessageTypeGen(ss,t,typeChar<T>());
	
	}
};

/*
int main2(){

	std::cout<<typeChar<long>()<<std::endl;

}

int main4(){
	std::string addr="./sock.sock";
	SrvSocket s(addr);
	CliSocket c1(addr);
	CliSocket c2(addr);

	std::cout<<s.fd<<std::endl;
	std::cout<<s.addr.sun_path<<std::endl;
	std::cout<<s.br<<std::endl;
	std::cout<<s.lr<<std::endl;
	std::cout<<c1.cr<<std::endl;
		
	s.accepts();
	//s.accepts();

	std::string msg1="hello world";
	std::string msg2=std::string(100,'0')+std::string(100,'1')+std::string(5,'2');
	std::string msg3=s.prepareMessage("ag","ag",'2',1020);
	for(auto i:msg3){
		std::cout<<+i<<" ";
	}
	std::cout<<std::endl;
	
	c1.sends(msg3);
	//c1.sends(msg2);
	std::vector<std::string> msgr1=s.receiveAccp(0);
	for(auto i:msgr1[0]){
		std::cout<<+i<<" ";
	}
	std::cout<<std::endl;
	for(auto i:msgr1){
		std::cout<<i<<std::endl;
	}
	auto p=reinterpret_cast<int*>(&(msgr1[0][8]));
	std::cout<<"Received:"<<msgr1[0][8]<<std::endl;
	std::cout<<"Received:"<<*p<<std::endl;
	//s.sends(msg2,0);
	std::vector<std::string> msgr2=c1.receive();

	for(auto i:msgr2){
		std::cout<<i<<std::endl;
	}
	
	s.accepts();
	for(auto i:s.accepted){
	std::cout<<i<<std::endl;
	}
}
*/
