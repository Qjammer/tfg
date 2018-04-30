#include<iostream>
#include"SrvSocket.hpp"
#include"CliSocket.hpp"
#include"envrec.hpp"

int main(){
	bucket b;

	b.pQueue.push_back({1.1, 2.2, 3.3});
	b.pQueue.push_back({0.1, 2.2, 3.3});
	b.pQueue.push_back({0.1, 1.2, 2.3});
	b.processPoints();
	std::cout<<b.FFo<<std::endl;
	std::cout<<b.FZo<<std::endl;
	std::cout<<b.beta<<std::endl;

}
/*
void printVarVec(vecvar v);
int main3(){
	std::string addr="sock.sock";
	SrvSocket s(addr);
	CliSocket c1(addr);
	s.accepts();
	std::string msg3=s.prepareMessage("ag","ag",'2',1020,(float) 1.23,'c',false);
	std::cout<<msg3<<std::endl;
	c1.sends(msg3);

	std::vector<std::string> msgr1=s.receiveAccp(0);
	std::string ms1=msgr1[0];
	std::cout<<ms1<<std::endl;
	vecvar vv=s.processMessage(ms1);
	printVarVec(vv);
	
}

void printVarVec(vecvar v){
	for(auto i:v){
		switch(i.first){
	#define printCase(U,i) case typeChar<U>():{std::cout<<std::get<U>(i.second)<<" ";break;}
			printCase(void*,i);
			printCase(bool,i);
			printCase(char,i);
			printCase(int,i);
			printCase(long,i);
			printCase(float,i);
			printCase(double,i);
		}
	}
	std::cout<<std::endl;
}


int main4(){
	std::string addr="./sock.sock";
	SrvSocket s(addr);
	CliSocket c1(addr);
	CliSocket c2(addr);

	std::cout<<s.fd<<std::endl;
	std::cout<<s.addr.sun_path<<std::endl;
	std::cout<<s.br<<std::endl;
	std::cout<<s.lr<<std::endl;
	std::cout<<c1.cr<<std::endl;
		
	s.accepts();
	//s.accepts();

	std::string msg1="hello world";
	std::string msg2=std::string(100,'0')+std::string(100,'1')+std::string(5,'2');
	std::string msg3=s.prepareMessage("ag","ag",'2',1020);
	for(auto i:msg3){
		std::cout<<+i<<" ";
	}
	std::cout<<std::endl;
	
	c1.sends(msg3);
	//c1.sends(msg2);
	std::vector<std::string> msgr1=s.receiveAccp(0);
	for(auto i:msgr1[0]){
		std::cout<<+i<<" ";
	}
	std::cout<<std::endl;
	for(auto i:msgr1){
		std::cout<<i<<std::endl;
	}
	auto p=reinterpret_cast<int*>(&(msgr1[0][8]));
	std::cout<<"Received:"<<msgr1[0][8]<<std::endl;
	std::cout<<"Received:"<<*p<<std::endl;
	//s.sends(msg2,0);
	std::vector<std::string> msgr2=c1.receive();

	for(auto i:msgr2){
		std::cout<<i<<std::endl;
	}
	
	s.accepts();
	for(auto i:s.accepted){
	std::cout<<i<<std::endl;
	}
}
*/
