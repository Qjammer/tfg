#include"ArduinoHndlr.hpp"
#include<cstring>
#include<fcntl.h>
#include<unistd.h>
#include<cerrno>
#include<linux/serial.h>

ArduinoHandler::ArduinoHandler(const std::string& addr):addr(addr){
	this->fd=open(addr.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC|O_SYNC);
	int er=errno;
	if(this->fd==-1){
		std::cerr<<"Error when opening arduino stream: "<<strerror(er)<<std::endl;
	}
	struct termios& opts=this->options;
	tcgetattr(this->fd,&opts);
	//Baud rate=115200
	cfsetispeed(&opts,B115200);
	cfsetospeed(&opts,B115200);
	opts.c_cflag|=(CLOCAL|CREAD);
	//Data bit number
	opts.c_cflag&=~CSIZE;
	opts.c_cflag|=CS8;
	//No Parity check
	opts.c_cflag&=~PARENB;
	//One Stop bit
	opts.c_cflag&=~CSTOPB;
	tcsetattr(this->fd,TCSANOW,&opts);
	tcflush(this->fd,TCIOFLUSH);
}

std::vector<std::string> ArduinoHandler::receive(){
	std::string s;
	int sz;
	do{
		memset(this->buf,0,sizeof(this->buf));
		sz=read(this->fd,this->buf,sizeof(this->buf));
		int er=errno;
		if(sz==-1){
			//Error
		} else {
			s+=std::string(this->buf,sz);
		}
	}while(sz>0);
	this->partMes+=s;
	return this->processMessages();
}

std::vector<std::string> ArduinoHandler::processMessages(){
	std::vector<std::string> v;
	int initpos;
	std::string& pm=this->partMes;

	while(pm.size()>0){
		initpos=pm.find("ar");
		if(initpos==-1){
			pm.clear();
		} else {
			pm.erase(pm.begin(),pm.begin()+initpos);
			const char* c=pm.c_str();
			const uint32_t* p=reinterpret_cast<const uint32_t*>(c+4);
			unsigned int sz=4+sizeof(uint32_t)+(*p);
			if(pm.size()>=sz){
				v.push_back(pm.substr(0,sz));
				pm.erase(pm.begin(),pm.begin()+sz);
			} else if(pm.size()>256){//Artificial limit to make sure no endless loops appear
				pm=pm.substr(2);
				initpos=pm.find("ar");
				pm.erase(pm.begin(),pm.begin()+initpos);
			} else {
				break;
			}
		}
	}
	return v;
}

int ArduinoHandler::sends(const std::string& msg){
	int sr=write(this->fd,msg.c_str(),msg.size());
	int er=errno;
	if(sr==-1){
		std::cerr<<"Error when writing to arduino: "<<strerror(er)<<std::endl;
	}
	return sr;
}

