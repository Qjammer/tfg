#pragma once
#include<map>
#include<set>
#include<eigen3/Eigen/Eigen>
#include<linux/serial.h>
#include<termios.h>
#include"Module.hpp"
#include"Socket.hpp"

class ArduinoHandler{
public:
	ArduinoHandler(const std::string& addr):addr(addr){
		this->fd=open(addr.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC|O_SYNC);
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

	struct termios options;
	std::string addr;
	int fd;
	char buf[256];

	std::string receive(){
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

	int sends(const std::string& msg){
		int sr=write(this->fd,msg.c_str(),msg.size());
		int er=errno;
		if(sr==-1){
			std::cerr<<strerror(er)<<std::endl;
		}
		return sr;
	
	}
};

class Sens:public Module{
public:
	std::vector<Eigen::Vector3d> gyro;
	std::vector<Eigen::Vector3d> accel;

	Sens(const std::string& srvaddr);

	void handleVarMessage(varmes& mv);

	virtual void handleOutComms();
	std::vector<std::string> prepareMesGyros();
	std::string prepareMesGyro(const Eigen::Vector3d& v) const;
	std::vector<std::string> prepareMesAccels();
	std::string prepareMesAccel(const Eigen::Vector3d& v) const;

	virtual void process();
};
