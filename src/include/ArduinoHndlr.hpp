#pragma once
#include<iostream>
#include<vector>
#include<termios.h>
#include"protocolHandler.hpp"

class ArduinoHandler:public protocolHandler{
public:
	struct termios options;
	std::string addr;
	int fd;
	char buf[2048];
	std::string partMes;

	ArduinoHandler(const std::string& addr);

	void retrieve();
	std::vector<std::string> receive();
	std::vector<std::string> processMessages();
	int sends(const std::string& msg);
};

