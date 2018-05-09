#include"state.hpp"

State::State(const std::string& srvaddr):Module(MOD_TYPE::CONTR,srvaddr){}

void State::handleMesAccel(const varmes& mv){
	if(varCond<double,double,double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		makeMesVar(double,z,2);
		this->accel=Eigen::Vector3d(x,y,z);
	}
}

void State::handleMesGyro(const varmes& mv){
	if(varCond<double,double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		makeMesVar(double,z,2);
		this->gyro=Eigen::Vector3d(x,y,z);
	}
}

void State::handleMesTacho(const varmes& mv){
	if(varCond<double,double,double,double>(mv)){
		makeMesVar(double,w1,0);
		makeMesVar(double,w2,1);
		makeMesVar(double,w3,2);
		makeMesVar(double,w4,3);
		this->tacho=Eigen::Vector4d(w1,w2,w3,w4);
	}
}

void State::handleVarMessage(const varmes& mv){
	if(mv.sender==modStr<MOD_TYPE::SENS>()){
		if(mv.purpose=="ac"){
			this->handleMesAccel(mv);
		}else if(mv.purpose=="gy"){
			this->handleMesGyro(mv);
		}else if(mv.purpose=="ta"){
			this->handleMesTacho(mv);
		}
	}
}

std::string State::prepareMesOri(){
	return this->srvs.prepareMessage(modStr<MOD_TYPE::STATE>(),"or",this->ori.w(),this->ori.vec().x(),this->ori.vec().y(),this->ori.vec().z());
}

std::string State::prepareMesPos(){
	return this->srvs.prepareMessage(modStr<MOD_TYPE::STATE>(),"ps",this->pos[0],this->pos[1],this->pos[2]);
}

void State::handleOutComms(){
	this->srvs.acceptsAll();
	std::string msgori=this->prepareMesOri();
	std::string msgpos=this->prepareMesPos();
	this->srvs.sendsToAll(msgori);
	this->srvs.sendsToAll(msgpos);
}

void predict(){


}

void update(){

}
void State::process(){

}
