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
	char buf[256];
	std::string partMes;

	ArduinoHandler(const std::string& addr);

	std::vector<std::string> receive();
	std::vector<std::string> processMessages();
	int sends(const std::string& msg);
};
