#include<iostream>
#include"Socket.hpp"
#include"envrec.hpp"

void mapTest();
void envrecTest();

int main(){
	envrecTest();
	//mapTest();
	return 0;
}

void envrecTest(){
	srand((unsigned int) time(0));
	
	std::string tsock="testsock.sock";
	SrvSocket ss(tsock);
	EnvRec er("envrec.sock");
	er.clis.emplace_back(tsock);
	std::string msg1=ss.prepareMessage(modStr<MOD_TYPE::STATE>(),"or",1.0,0.0,0.0,0.0);
	std::string msg2=ss.prepareMessage(modStr<MOD_TYPE::STATE>(),"ps",0.0,0.0,0.0);
	ss.acceptsAll();
	ss.sendsToAll(msg1);
	ss.sendsToAll(msg2);
	std::cout<<"or"<<std::endl<<er.ori.vec()<<std::endl;
	std::cout<<"ps"<<std::endl<<er.pos<<std::endl;
	er.handleInComms();
	std::cout<<"or"<<std::endl<<er.ori.vec()<<std::endl;
	std::cout<<"ps"<<std::endl<<er.pos<<std::endl;
#define PTS 1000
	Eigen::Matrix<double,2,PTS> randM=Eigen::Matrix<double,2,PTS>::Random();
	auto vC=Eigen::Vector2d(0.1,0.2);
	auto randZ=vC.transpose()*randM;
	for(int i=0;i<randM.cols();++i){
		point p(randM.col(i).x(),randM.col(i).y(),2+randZ.col(i)[0]);
		er.unp.push_back(p);
	}
	er.process();
	for(auto b:er.bm.m){
		auto k=b.first;
		std::cout<<k.first<<" "<<k.second<<" "<<b.second.w<<std::endl;
	
	}
}

/*
void mapTest(){
	srand((unsigned int) time(0));
	buckMap bm({2.0,2.0});
	Eigen::Matrix<double,2,10> randM=Eigen::Matrix<double,2,10>::Random();
	auto vC=Eigen::Vector2d(1.1,2.2);
	auto randZ=vC.transpose()*randM;
	for(int i=0;i<randM.cols();++i){
		point p(randM.col(i).x(),randM.col(i).y(),2+randZ.col(i)[0]);
		bm.insertPoint(p);
	}
	key k=bm.calcKey({0.0,0.0,0.0});
	bm.processPendingBuckets();
	std::cout<<bm.m.find(k)->second.FFo<<std::endl;
	std::cout<<bm.m.find(k)->second.FZo<<std::endl;
	std::cout<<bm.m.find(k)->second.beta<<std::endl;
	
}

void bucketTest(){
	bucket b({0.6,1.7});

	b.pQueue.push_back({1.1, 2.2, 3.3});
	b.pQueue.push_back({0.1, 2.2, 3.3});
	b.pQueue.push_back({0.1, 1.2, 2.3});
	b.processPoints();
	std::cout<<b.FFo<<std::endl;
	std::cout<<b.FZo<<std::endl;
	std::cout<<b.beta<<std::endl;
	std::cout<<b.z<<std::endl;
}
*/
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
