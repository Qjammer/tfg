#include<iostream>
#include"Socket.hpp"
#include"sensor.hpp"
#include"ArduinoHndlr.hpp"

//void mapTest();
//void envrecTest();
//void contrTest();
//void pathfTest();
void stateTest();
void arduinoTest();

int main(){
	arduinoTest();
	//stateTest();
	//pathfTest();
	//contrTest();
	//envrecTest();
	//mapTest();
	return 0;
}

void arduinoTest(){

	ArduinoHandler ino("/dev/ttyACM0");
	while(true){
		std::vector<std::string> rcv=ino.receive();
		std::cout<<"Receive: "<<std::endl;
		for(auto i:rcv){
			std::cout<<i<<std::endl;
			if(i[0]=='a'){
				const char* b=rcv[0].c_str();
				const int32_t* p=reinterpret_cast<const int32_t*>(b+9);
				std::cout<<*p<<std::endl;
			}
		}
		std::string msg="bcd";
		int rv=ino.sends(msg);
		std::cout<<"sends retval: "<<rv<<std::endl;
		usleep(30000);
	}

}
/*
void stateTest(){
	std::string tsock="testsock.sock";
	SrvSocket ss(tsock);
	State st("state.sock");
	st.clis.emplace_back(tsock);
	st.tprev=std::chrono::high_resolution_clock::now();
	st.dt=std::chrono::milliseconds(10);
	st.rotvel<<0.0,0.0,0.1;
	st.gyro<<0.0,0.0,0.1;
	st.vel<<0.0,0.0,0.0;
	st.vel<<0.0,0.1,0.0;
	double angle=0.0*M_PI;
	st.ori=Eigen::Quaterniond(cos(angle/2),0,0,sin(angle/2));
	st.Pk=0.01*Eigen::Matrix<double,STATE_N,STATE_N>::Identity();
	st.Rk=0.01*Eigen::Matrix<double,SENSOR_N,SENSOR_N>::Identity();
	st.accelSens<<0.0,0.0,9.81;
	st.accelSens<<-0.01,0.0,9.81;
	st.accelState<<0.0,0.0,0.0;
	st.accelState<<-0.01,0.0,0.0;

	int i=0;
	auto begin=std::chrono::high_resolution_clock::now();
	auto now=begin;
	double dts=std::chrono::duration_cast<std::chrono::duration<double>>(now-begin).count();
	unsigned int max=-1;
	Eigen::IOFormat fmt(6,Eigen::DontAlignCols,"\t");
	do{
		st.process();
		if(++i%1000==0){std::cout<<dts<<"\t"<<st.pos.transpose().format(fmt)<<std::endl;}
		now=std::chrono::high_resolution_clock::now();
		dts=std::chrono::duration_cast<std::chrono::duration<double>>(now-begin).count();
	}while(dts<10.0&&i<max);

	std::cout<<"xk"<<std::endl<<st.xk<<std::endl<<std::endl;
	std::cout<<"Fk"<<std::endl<<st.Fk<<std::endl<<std::endl;
	std::cout<<"Pk"<<std::endl<<st.Pk<<std::endl<<std::endl;
	std::cout<<"zk"<<std::endl<<st.zk<<std::endl<<std::endl;
	std::cout<<"Hk"<<std::endl<<st.Hk<<std::endl<<std::endl;
	std::cout<<"Kk"<<std::endl<<st.Kk<<std::endl<<std::endl;
	std::cout<<"accelSens"<<std::endl<<st.accelSens<<std::endl<<std::endl;
	std::cout<<"accelState"<<std::endl<<st.accelState<<std::endl<<std::endl;
	std::cout<<"vel"<<std::endl<<st.vel<<std::endl<<std::endl;
	std::cout<<"Loops:"<<i<<std::endl<<std::endl;
	std::cout<<"testquat:"<<std::endl<<(st.ori*Eigen::Quaterniond(0,1,0,0)*st.ori.inverse()).vec()<<std::endl<<std::endl;

}
*/
/*
void pathfTest(){
	std::string tsock="testsock.sock";
	SrvSocket ss(tsock);
	Pathf pf("controller.sock");
	pf.clis.emplace_back(tsock);
	std::string msgp1=ss.prepareMessage(modStr<MOD_TYPE::STATE>(),"ps",-1.2,-0.2,1.2);
	std::string msgs1=ss.prepareMessage(modStr<MOD_TYPE::SUPER>(),"ng",2,2);
	std::string msgw1=ss.prepareMessage(modStr<MOD_TYPE::ENVREC>(),"nw",0,0,1.0);
	std::string msgw2=ss.prepareMessage(modStr<MOD_TYPE::ENVREC>(),"nw",1,1,4.0);
	std::string msgw3=ss.prepareMessage(modStr<MOD_TYPE::ENVREC>(),"nw",2,2,1.0);
	ss.acceptsAll();
	ss.sendsToAll(msgp1);
	ss.sendsToAll(msgs1);
	ss.sendsToAll(msgw1);
	//ss.sendsToAll(msgw2);
	ss.sendsToAll(msgw3);

	pf.handleInComms();
	for(auto i:pf.nmap){
		std::cout<<"key:"<<i.first.first<<","<<i.first.second<<"  \tw:"<<i.second.w<<std::endl;
	}
	std::cout<<"Current Node: "<<pf.curNode.first<<","<<pf.curNode.second<<std::endl;

	pf.process();
	ss.sendsToAll(msgw2);
	pf.handleInComms();
	pf.process();

	std::cout<<"Number of nodes: "<<pf.nmap.size()<<std::endl;
	for(auto i:pf.nmap){
		std::cout<<"key: "<<i.first.first<<","<<i.first.second<<"\tg: "<<i.second.g<<"\trhs: "<<i.second.rhs<<std::endl;
	}
	//std::cout<<"Open Queue Remaining Size:"<<pf.openQueue.size()<<std::endl;
}
*/
/*
void contrTest(){
	std::string tsock="testsock.sock";
	SrvSocket ss(tsock);
	Contr ct("controller.sock");
	ct.clis.emplace_back(tsock);
	double t=0.499*M_PI;
	std::string msg1=ss.prepareMessage(modStr<MOD_TYPE::STATE>(),"or",cos(t/2),0.0,0.0,sin(t/2));
	std::string msg2=ss.prepareMessage(modStr<MOD_TYPE::STATE>(),"ps",1.2,1.2,1.2);
	std::string msg3=ss.prepareMessage(modStr<MOD_TYPE::PATHF>(),"np",1.2,2.2);
	ss.acceptsAll();
	ss.sendsToAll(msg1);
	ss.sendsToAll(msg2);
	ss.sendsToAll(msg3);

	std::cout<<"ori:"<<ct.ori.vec()<<std::endl;
	ct.handleInComms();
	
	std::cout<<"vr:"<<ct.vr<<std::endl;
	std::cout<<"omega:"<<ct.omega<<std::endl;
	ct.process();
	std::cout<<"ori:"<<ct.ori.vec()<<std::endl;
	std::cout<<"vr:"<<ct.vr<<std::endl;
	std::cout<<"omega:"<<ct.omega<<std::endl;
}
*/
/*
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
*/
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
