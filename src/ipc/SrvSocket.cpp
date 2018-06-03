#include"SrvSocket.hpp"

SrvSocket::SrvSocket(const std::string& addr):BaseSocket(addr),br(-1),lr(-1){
	this->init();
}


int SrvSocket::binds(){
	int ur=unlink(this->addr.sun_path);
	int er=errno;
	if(ur==-1){this->handleUnlinkErr(er);}

	this->br=bind(this->fd,(struct sockaddr*)&this->addr,sizeof(addr));
	er=errno;
	if(this->br==-1){this->handleBindErr(er);}
	return this->br;
}

void SrvSocket::handleUnlinkErr(int er){
	switch(er){
		default:
			std::cerr<<"Unhandled error in unlink:"<<er<<strerror(er)<<std::endl;
			break;
	}
}
	
void SrvSocket::handleBindErr(int er){
	switch(er){
		case EACCES:
			break;
		case EADDRINUSE:
			break;
		case EBADF:
			break;
		case EINVAL:
			break;
		default:
			std::cerr<<"Unhandled error in bind:"<<er<<strerror(er)<<std::endl;
			break;
	}
}

int SrvSocket::listens(){
	this->lr=listen(this->fd,5);
	int er=errno;
	if(this->lr==-1){this->handleListenErr(er);}
	return this->lr;
}

void SrvSocket::handleListenErr(int er){
	switch(er){
		case EBADF:
			//Invalid file descriptor. Might need to create a new socket
			break;
		default:
			std::cerr<<"Unhandled error in listen:"<<er<<strerror(er)<<std::endl;
			break;
	}
}

int SrvSocket::init(){
	this->binds();
	this->listens();
	return this->lr;
}

int SrvSocket::accepts(){
	int a=accept(this->fd,NULL,NULL);
	int er=errno;
	if(a==-1){
		//Accept error
		this->handleAcceptErr(er);
	} else {
		this->accepted.push_back(a);
		this->setFlagFD(a,O_NONBLOCK);
	}
	return a;
}

void SrvSocket::acceptsAll(){
	int r;
	do{
	r=this->accepts();
	}while(r!=-1);
}

void SrvSocket::handleAcceptErr(int er){
	switch(er){
		case EWOULDBLOCK:
			//std::cout<<"No connection to be accepted"<<std::endl;
			break;
		case EBADF:
			break;
		case EINTR:
			break;
		case EINVAL:
			break;
		case ENOBUFS:
		default:
			std::cerr<<"Unhandled error in accept:"<<er<<strerror(er)<<std::endl;
			break;
	}
}

std::vector<std::string> SrvSocket::receiveAccp(int accp){
	return this->receiveFD(this->accepted[accp]);
}

void SrvSocket::sends(const std::string& msg,int accp){
	this->sendsFD(msg,this->accepted[accp]);
}

void SrvSocket::sendsToAll(const std::string& msg){
	for(unsigned int i=0;i<this->accepted.size();++i){
		this->sends(msg,i);
	}
}
