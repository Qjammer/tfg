#pragma once
#include<sys/socket.h>
#include<sys/un.h>
#include<string>
#include<vector>
#include<iostream>
#include<variant>
#include<unistd.h>
#include<fcntl.h>
#include"protocolHandler.hpp"

class BaseSocket:public protocolHandler{
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
};

