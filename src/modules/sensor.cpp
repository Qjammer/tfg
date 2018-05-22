#include"sensor.hpp"

Sens::Sens(const std::string& srvaddr):Module(MOD_TYPE::SENS,srvaddr){}

void Sens::handleVarMessage(varmes& mv){
}

void Sens::handleArduino(int i){


}

std::string Sens::prepareMesGyro(const Eigen::Vector3d& v) const{
	return this->srvs.prepareMessage(modStr<MOD_TYPE::SENS>(),"gy",v.x(),v.y(),v.z());
}

std::vector<std::string> Sens::prepareMesGyros(){
	std::vector<std::string> v;
	for(auto i:this->gyro){
		v.push_back(this->prepareMesGyro(i));
	}
	this->gyro.clear();
	return v;
}

std::string Sens::prepareMesAccel(const Eigen::Vector3d& v) const{
	return this->srvs.prepareMessage(modStr<MOD_TYPE::SENS>(),"ac",v.x(),v.y(),v.z());
}


std::vector<std::string> Sens::prepareMesAccels(){
	std::vector<std::string> v;
	for(auto i:this->accel){
		v.push_back(this->prepareMesAccel(i));
	}
	this->accel.clear();
	return v;
}

void Sens::handleOutComms(){
	this->srvs.acceptsAll();
	std::vector<std::string> msg1=this->prepareMesGyros();
	for(auto i:msg1){
		this->srvs.sendsToAll(i);
	}
	std::vector<std::string> msg2=this->prepareMesAccels();
	for(auto i:msg2){
		this->srvs.sendsToAll(i);
	}
}

void Sens::process(){
}
