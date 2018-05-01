#pragma once
#include"BaseSocket.hpp"

class SrvSocket:public BaseSocket{
	public:
	int br;
	int lr;
	std::vector<int> accepted;

	SrvSocket(const std::string& addr);
		
	int binds();
	void handleUnlinkErr(int er);
	void handleBindErr(int er);

	int listens();
	void handleListenErr(int er);

	virtual int init();


	int accepts();
	void acceptsAll();
	void handleAcceptErr(int er);

	std::vector<std::string> receiveAccp(int accp);

	void sends(const std::string& msg,int accp);
	void sendsToAll(const std::string& msg);
};
