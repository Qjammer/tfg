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

void State::calcFk(){

}

void State::calcHk(){

}

void State::assemblezk(){
	this->zk.segment<3>(0)=this->accel;
	this->zk.segment<3>(3)=this->gyro;
	

}
void State::disassemblexk(){
	this->pos=this->xk.segment<3>(0);
	this->vel=this->xk.segment<3>(3);
	this->ori.w()=this->xk(6);
	this->ori.vec()=this->xk.segment<3>(7);
	this->rotvel=this->xk.segment<3>(10);
}

void State::predict(){
	this->calcFk();

	this->xk=this->Fk*this->xk;
	this->Pk=this->Fk*this->Pk*this->Fk.transpose();
}

void State::update(){
	this->calcHk();

	this->yk=this->zk-this->Hk*this->xk;
	this->Sk=this->Rk+this->Hk*this->Pk*this->Hk.transpose();
	this->Kk=this->Pk*this->Hk.transpose()*this->Sk.inverse();
	this->xk=this->xk+this->Kk*this->yk;
	this->Pk=(Eigen::Matrix<double,STATE_N,STATE_N>::Identity()-this->Kk*this->Hk)*this->Pk;
}

void State::process(){

}
