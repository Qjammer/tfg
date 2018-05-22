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

std::string ArduinoHandler::receive(){
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
	return s;
}

int ArduinoHandler::sends(const std::string& msg){
	int sr=write(this->fd,msg.c_str(),msg.size());
	int er=errno;
	if(sr==-1){
		std::cerr<<"Error when writing to arduino: "<<strerror(er)<<std::endl;
	}
	return sr;
}

