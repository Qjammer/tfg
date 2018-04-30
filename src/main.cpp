#include<iostream>
#include"SrvSocket.hpp"
#include"CliSocket.hpp"

int main(){
	std::cout<<"Hello World"<<std::endl;
	std::string addr="sock.sock";
	SrvSocket s(addr);

}
