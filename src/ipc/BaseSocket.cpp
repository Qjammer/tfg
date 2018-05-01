#include<BaseSocket.hpp>

BaseSocket::BaseSocket(const std::string& addr){
	this->fd=socket(AF_UNIX,SOCK_SEQPACKET,0);
	//this->fd=socket(AF_UNIX,SOCK_STREAM,0);
	int er=errno;
	if(this->fd==-1){this->handleSocketErr(er);}

	this->addr.sun_family=AF_UNIX;
	strncpy(this->addr.sun_path, addr.c_str(), sizeof(this->addr.sun_path)-1);
	this->setFlagFD(this->fd,O_NONBLOCK);
}

void BaseSocket::handleSocketErr(int er){
	switch(er){
		case EACCES:
			break;
		case EINVAL:
			break;
		case ENOBUFS:
			break;
		default:
			std::cerr<<"Unhandled error in socket creation:"<<er<<strerror(er)<<std::endl;
			break;
	}
}

int BaseSocket::setFlagFD(int fd,int flag){
	int flags=fcntl(fd,F_GETFL,0);
	return fcntl(fd,F_SETFL,flags|flag);
}

int BaseSocket::unsetFlagFD(int fd,int flag){
	int flags=fcntl(fd,F_GETFL,0);
	return fcntl(fd,F_SETFL,flags&~flag);
}


std::vector<std::string> BaseSocket::receiveFD(int fd){
	std::vector<std::string> v;
	std::string strbuf;
	int sz;
	do{
		memset(this->buf,0,sizeof(this->buf));
		sz=read(fd,this->buf,sizeof(this->buf));
		int er=errno;
		if(sz==-1){
			this->handleReadErr(er);
		}else{
			strbuf=std::string(this->buf,sz);
			v.push_back(strbuf);

		}
	}while(sz!=-1);
	return v;
}

void BaseSocket::handleReadErr(int er){
	switch(er){
		case EWOULDBLOCK:
			std::cerr<<"Nothing more to read"<<std::endl;
			break;
		case EINVAL:
			std::cerr<<"Invalid fd in read:"<<er<<strerror(er)<<std::endl;
			break;
		default:
			std::cerr<<"Unhandled error in read:"<<er<<strerror(er)<<std::endl;
			break;
	}
}

int BaseSocket::sendsFD(const std::string& msg,int fd){
	int sr=send(fd,msg.c_str(),msg.size(),0);
	return sr;
}

template<typename T>
T processType(const std::string& msg,unsigned int& pos){
	char ar[sizeof(T)];
	pos++;
	for(unsigned int i=0;i<sizeof(T);++i){
	ar[i]=msg[pos+i];
	}
	pos+=sizeof(T);
	T* t = reinterpret_cast<T*>(ar);
	return *t;
}

vecvar BaseSocket::processMessage(const std::string& msg){
	vecvar vec;
	unsigned int pos=4;//Ignore header info for now
	while(pos<msg.size()){
		switch (msg[pos]){
	#define processTypeSwitch(U) case typeChar<U>():{vec.push_back(myvari(typeChar<U>(),processType<U>(msg,pos)));break;}
			processTypeSwitch(void*);
			processTypeSwitch(bool);
			processTypeSwitch(int);
			processTypeSwitch(char);
			processTypeSwitch(long);
			processTypeSwitch(float);
			processTypeSwitch(double);
		}
	}
	return vec;

}
