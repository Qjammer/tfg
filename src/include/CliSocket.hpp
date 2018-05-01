#pragma once
#include"BaseSocket.hpp"

class CliSocket:public BaseSocket{
	public:
	int cr;

	CliSocket(const std::string& addr);

	int connects();
	void handleConnectErr(int er);

	virtual int init();
	int sends(const std::string& msg);

	std::vector<std::string> receive();
};

