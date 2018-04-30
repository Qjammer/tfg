#include"CliSocket.hpp"

CliSocket::CliSocket(const std::string& addr):Socket(addr),cr(-1){
	this->init();
}

int CliSocket::connects(){
	this->cr=connect(this->fd,(struct sockaddr*)&this->addr,sizeof(this->addr));
	int er=errno;
	if(this->cr==-1){this->handleConnectErr(er);}
	return this->cr;
}

void CliSocket::handleConnectErr(int er){
	switch(er){
		case EINPROGRESS:
			break;
		case EINTR:
			break;
		case EISCONN:
			break;
		case ETIMEDOUT:
			break;
		default:
			std::cerr<<"Unhandled error in connect:"<<er<<strerror(er)<<std::endl;
			break;
	}
}

int CliSocket::init(){
	this->connects();
	return this->cr;
}

int CliSocket::sends(const std::string& msg){
	return this->sendsFD(msg,this->fd);
}

std::vector<std::string> CliSocket::receive(){
	return this->receiveFD(this->fd);
}

