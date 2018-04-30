#pragma once
#include"Socket.hpp"

class SrvSocket:public Socket{
	public:
	int br;
	int lr;
	std::vector<int> accepted;

	SrvSocket(std::string& addr);
		
	int binds();
	void handleBindErr(int er);

	int listens();
	void handleListenErr(int er);

	virtual int init();


	int accepts();
	void handleAcceptErr(int er);

	std::vector<std::string> receiveAccp(int accp);

	void sends(const std::string& msg,int accp);
	void sendsToAll(const std::string& msg);
};


