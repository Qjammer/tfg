#include"extcomms.hpp"

ExtComms::ExtComms(const std::string& srvaddr):Module(MOD_TYPE::EXTCOM,srvaddr){}

void ExtComms::handleInComms(){
	//Handle Arduino Messages
	for(int i=0;i<this->ardhndls.size();i++){
		auto v=this->handleInArduino(i);
		for(auto&m:v){
			this->handleVarMessage(m);
		}
	}
	//Handle Kernel Messages
	this->Module::handleInComms();
}

void ExtComms::handleVarMessage(const varmes& mv){
	std::cout<<mv.sender<<" "<<mv.purpose<<" ";
	if(mv.sender=="ar"){
		if(mv.purpose=="ac"){
			this->handleMesAccel(mv);
		} else if(mv.purpose=="gy"){
			this->handleMesGyro(mv);
		} else if(mv.purpose=="li"){
			this->handleMesLIDARPoint(mv);
		} else if(mv.purpose=="ta"){
			//this->handleMesTacho(mv);
		} else if(mv.purpose=="gp"){
			//this->handleMesGPS(mv);
		}
	} else if(mv.sender==modStr<MOD_TYPE::CONTR>()){
		if(mv.purpose=="ws"){
			this->handleMesWheels(mv);
		} else if(mv.purpose=="ra"){
			//this->handleMesArm(mv);
		} else if(mv.purpose=="lt"){
			//this->handleMesLIDARTower(mv);
		}
	}
}

void ExtComms::handleMesAccel(const varmes& mv){
	if(varCond<float,float,float>(mv)){
		makeMesVar(float,x,0);
		makeMesVar(float,y,1);
		makeMesVar(float,z,2);
		std::cout<<x<<" "<<y<<" "<<z<<std::endl;
		this->accel.emplace_back(x,y,z);
	}
}

void ExtComms::handleMesGyro(const varmes& mv){
	if(varCond<float,float,float>(mv)){
		makeMesVar(float,x,0);
		makeMesVar(float,y,1);
		makeMesVar(float,z,2);
		this->gyro.emplace_back(x,y,z);
	}
}

void ExtComms::handleMesLIDARPoint(const varmes& mv){
	if(varCond<float,float,float>(mv)){
		makeMesVar(float,x,0);
		makeMesVar(float,y,1);
		makeMesVar(float,z,2);
		this->lidarpts.emplace_back(x,y,z);
	}
}

void ExtComms::handleMesWheels(const varmes& mv){
	if(varCond<double,double,double,double,double,double>(mv)){
		makeMesVar(double,w1,0);
		makeMesVar(double,w2,1);
		makeMesVar(double,w3,2);
		makeMesVar(double,w4,3);
		//makeMesVar(double,w5,4);
		//makeMesVar(double,w6,5);
		//this->wheels.emplace_back(w1,w2,w3,w4,w5,w6);
		this->wheels.emplace_back(w1,w2,w3,w4);
	}
}

std::vector<varmes> ExtComms::handleInArduino(int i){
	auto msgv=this->ardhndls[i].receive();
	std::vector<varmes> vmv;
	for(auto&mg:msgv){
		vmv.push_back(this->ardhndls[i].processMessage(mg));
	}
	return vmv;
}

std::string ExtComms::prepareMesWheels(const Eigen::Vector4f& v) const{
	return this->srvs.prepareMessage(modStr<MOD_TYPE::CONTR>(),"ws",v[0],v[1],v[2],v[3]);

}

std::string ExtComms::prepareMesGyro(const Eigen::Vector3d& v) const{
	return this->srvs.prepareMessage(modStr<MOD_TYPE::EXTCOM>(),"gy",v.x(),v.y(),v.z());
}

std::vector<std::string> ExtComms::prepareMesGyros(){
	std::vector<std::string> v;
	for(auto i:this->gyro){
		v.push_back(this->prepareMesGyro(i));
	}
	this->gyro.clear();
	return v;
}

std::string ExtComms::prepareMesAccel(const Eigen::Vector3d& v) const{
	return this->srvs.prepareMessage(modStr<MOD_TYPE::EXTCOM>(),"ac",v.x(),v.y(),v.z());
}

std::vector<std::string> ExtComms::prepareMesAccels(){
	std::vector<std::string> v;
	for(auto i:this->accel){
		v.push_back(this->prepareMesAccel(i));
	}
	this->accel.clear();
	return v;
}

std::string ExtComms::prepareMesLIDARPt(const Eigen::Vector3d& v) const{
	return this->srvs.prepareMessage(modStr<MOD_TYPE::EXTCOM>(),"li",v.x(),v.y(),v.z());
}

std::vector<std::string> ExtComms::prepareMesLIDARPts(){
	std::vector<std::string> v;
	for(auto i:this->lidarpts){
		v.push_back(this->prepareMesLIDARPt(i));
	}
	this->lidarpts.clear();
	return v;
}

void ExtComms::handleOutComms(){
	//Handle External Messages
	
	
	//Handle Kernel Messages
	this->srvs.acceptsAll();
	std::vector<std::string> msgaccel=this->prepareMesAccels();
	for(auto i:msgaccel){
		this->srvs.sendsToAll(i);
	}
	std::vector<std::string> msggyro=this->prepareMesGyros();
	for(auto i:msggyro){
		this->srvs.sendsToAll(i);
	}
	std::vector<std::string> msglidarpt=this->prepareMesLIDARPts();
	for(auto i:msglidarpt){
		this->srvs.sendsToAll(i);
	}
}

void ExtComms::process(){
}
