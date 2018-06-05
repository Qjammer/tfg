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
	//Disable canonical mode
	opts.c_lflag&=~ICANON;
	//opts.c_lflag|=ICANON;
	tcsetattr(this->fd,TCSANOW,&opts);
	tcflush(this->fd,TCIOFLUSH);
}

void ArduinoHandler::retrieve(){
	std::string s;
	int sz;
	do{
		memset(this->buf,0,sizeof(this->buf));
		sz=read(this->fd,this->buf,sizeof(this->buf));
		int er=errno;
		if(sz==-1){
			//Error
			switch(er){
				case EAGAIN:
					std::cerr<<"Serial Warning: would block"<<std::endl;
					break;

				default:
					std::cerr<<"Serial Error "<<er<<": "<<strerror(er)<<std::endl;
					break;
			}
		} else {
			s+=std::string(this->buf,sz);
		}
	}while(sz>0);
	this->partMes+=s;
}

std::vector<std::string> ArduinoHandler::receive(){
	this->retrieve();
	return this->processMessages();
}

std::vector<std::string> ArduinoHandler::processMessages(){
	std::vector<std::string> v;
	int initpos;
	std::string& pm=this->partMes;

	while(pm.size()>0){
		//std::cout<<pm.size()<<" ";
		initpos=pm.find("ar");
		if(initpos==-1){
			pm.clear();
		} else {
			pm.erase(pm.begin(),pm.begin()+initpos);
			
			const char* c=pm.c_str();
			const uint32_t* p=reinterpret_cast<const uint32_t*>(c+4);
			unsigned int sz=4+sizeof(uint32_t)+(*p);
			int secondpos=pm.substr(2).find("ar");
			if(secondpos==-1){//Next message hasnt arrived
				if(pm.size()>=sz){
					v.push_back(pm.substr(0,sz));
					pm.erase(pm.begin(),pm.begin()+sz);
				} else if(sz>256){//Artificial limit to make sure no endless loops appear
					pm=pm.substr(2);
				} else {
					break;
				}
			} else {//next message has arrived already
				if(secondpos+2==sz){//computed size is correct
					v.push_back(pm.substr(0,secondpos+2));
					pm.erase(pm.begin(),pm.begin()+secondpos+2);
				} else {
					pm.erase(pm.begin(),pm.begin()+1);
				}
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

