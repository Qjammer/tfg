#include"extcomms.hpp"

ExtComms::ExtComms(const std::string& srvaddr):Module(MOD_TYPE::EXTCOM,srvaddr){
	this->ardhndls.emplace_back("/dev/ttyArdUNO");
	this->ardhndls.emplace_back("/dev/ttyArdMEGA");
}

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
	if(mv.sender=="ar"){
		if(mv.purpose=="ac"){
			//this->handleMesAccel(mv);
		} else if(mv.purpose=="gy"){
			this->handleMesGyro(mv);
		} else if(mv.purpose=="li"){
			this->handleMesLIDARPoint(mv);
		} else if(mv.purpose=="ta"){
			this->handleMesTacho(mv);
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

void ExtComms::handleMesTacho(const varmes& mv){
	if(varCond<float,float,float,float>(mv)){
		makeMesVar(float,w0,0);
		makeMesVar(float,w1,1);
		makeMesVar(float,w2,2);
		makeMesVar(float,w3,3);
		std::cout<<w0<<" "<<w1<<" "<<w2<<" "<<w3<<std::endl;
		this->tacho.emplace_back(w0,w1,w2,w3);
	}
}

void ExtComms::handleMesWheels(const varmes& mv){
	if(varCond<double,double,double,double>(mv)){
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

std::string ExtComms::prepareMesWheel(const Eigen::Vector4f& v) const{
	return std::string("wheels ")+std::to_string(v[0])+" "+std::to_string(v[1])+" "+std::to_string(v[2])+" "+std::to_string(v[3])+" end ";
	//return this->srvs.prepareMessage(modStr<MOD_TYPE::CONTR>(),"ws",v[0],v[1],v[2],v[3]);
}

std::vector<std::string> ExtComms::prepareMesWheels(){
	std::vector<std::string> v;
	for(auto i:this->wheels){
		v.push_back(this->prepareMesWheel(i.cast<float>()));
	}
	this->wheels.clear();
	return v;
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
	return this->srvs.prepareMessage(modStr<MOD_TYPE::EXTCOM>(),"li",double(v.x()),double(v.y()),double(v.z()));
}

std::vector<std::string> ExtComms::prepareMesLIDARPts(){
	std::vector<std::string> v;
	for(auto i:this->lidarpts){
		v.push_back(this->prepareMesLIDARPt(i));
	}
	this->lidarpts.clear();
	return v;
}

std::string ExtComms::prepareMesTacho(const Eigen::Vector4d& v) const{
	return this->srvs.prepareMessage(modStr<MOD_TYPE::EXTCOM>(),"ta",double(v[0]),double(v[1]),double(v[2]),double(v[3]));
}

std::vector<std::string> ExtComms::prepareMesTachos(){
	std::vector<std::string> v;
	for(auto i:this->tacho){
		v.push_back(this->prepareMesTacho(i));
	}
	this->tacho.clear();
	return v;
}

void ExtComms::handleOutComms(){
	//Handle External Messages
	std::vector<std::string> msgwheels=this->prepareMesWheels();
	for(auto i:msgwheels){
		this->srvs.sendsToAll(i);
		for(auto ard:ardhndls){
			ard.sends(i);
		}
	}
	
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
	std::vector<std::string> msgtacho=this->prepareMesTachos();
	for(auto i:msgtacho){
		this->srvs.sendsToAll(i);
	}
}

void ExtComms::process(){
}
