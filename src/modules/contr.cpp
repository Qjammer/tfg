#include"contr.hpp"

Contr::Contr(const std::string& srvaddr):Module(MOD_TYPE::CONTR,srvaddr){}

void Contr::handleMesOri(varmes& mv){
	if(varCond<double,double,double,double>(mv)){
		makeMesVar(double,w,0);
		makeMesVar(double,x,1);
		makeMesVar(double,y,2);
		makeMesVar(double,z,3);
		this->ori=Eigen::Quaterniond(w,x,y,z);
	}
}

void Contr::handleMesPos(varmes& mv){
	if(varCond<double,double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		makeMesVar(double,z,2);
		this->pos=Eigen::Vector3d(x,y,z);
	}
}

void Contr::handleMesNextPos(varmes& mv){
	if(varCond<double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		this->nPos=Eigen::Vector2d(x,y);
	}
}

void Contr::handleVarMessage(varmes& mv){
	if(mv.sender==modStr<MOD_TYPE::STATE>()){
		if(mv.purpose=="or"){
			this->handleMesOri(mv);
		}else if(mv.purpose=="ps"){
			this->handleMesPos(mv);
		}
	}
	if(mv.sender==modStr<MOD_TYPE::PATHF>()){
		if(mv.purpose=="np"){
			this->handleMesNextPos(mv);
		}
	}
}

std::string Contr::prepareMesVel(){
	return this->srvs.prepareMessage(modStr<MOD_TYPE::CONTR>(),"vo",this->vr,this->omega);
}
std::string Contr::prepareMesWheels(){
	return this->srvs.prepareMessage(modStr<MOD_TYPE::CONTR>(),"ws",this->ws[0],this->ws[1],this->ws[2],this->ws[3]);
}

void Contr::handleOutComms(){
	this->srvs.acceptsAll();
	std::string msgw=this->prepareMesWheels();
	this->srvs.sendsToAll(msgw);
	std::string msgv=this->prepareMesVel();
	this->srvs.sendsToAll(msgv);
}

double symSat(double f,double th){
	return f>0?std::min(f,th):std::max(f,th);
}

void Contr::process(){
	Eigen::Vector2d relDist=this->nPos-this->pos.head(2);
	Eigen::Vector3d ydir=rotQuat(Eigen::Vector3d(0.0,1.0,0.0),this->ori.inverse());
	ydir.z()=0;
	double vn=relDist.norm()*ydir.norm();
	Eigen::Vector3d rd3;
	rd3<<relDist,0;

	double cost=ydir.dot(rd3)/vn;
	double sint=ydir.cross(rd3).z()/vn;
	double tant=sint/(1+cost);//Tangent of half angle. Period=2pi
	this->vr=this->maxV*cost;
	this->omega=symSat(tant,this->maxO);//Saturate here or in controller?
	for(int i=0;i<this->wheelpos.size();i++){
		this->ws[i]=this->calcwheelspeed(this->wheelpos[i]);
	
	}

}

double Contr::calcwheelspeed(const Eigen::Vector3d& wp){
	Eigen::Vector3d vw=Eigen::Vector3d(0,this->vr,0)+Eigen::Vector3d(0,0,this->omega).cross(wp);
	return vw.norm()/vw[1];
}
