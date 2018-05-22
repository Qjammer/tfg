#pragma once
#include<iostream>
#include<termios.h>

class ArduinoHandler{
public:
	struct termios options;
	std::string addr;
	int fd;
	char buf[256];

	ArduinoHandler(const std::string& addr);

	std::string receive();
	int sends(const std::string& msg);
};

