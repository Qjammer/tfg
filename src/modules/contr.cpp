#include"contr.hpp"


Contr::Contr(const std::string& srvaddr):Module(MOD_TYPE::CONTR,srvaddr){}

void Contr::handleMesOri(varmes& mv){
	if(mv.vars.size()==4){
		if(mv.vars[0].first==typeChar<double>()&&
			mv.vars[1].first==typeChar<double>()&&
			mv.vars[2].first==typeChar<double>()&&
			mv.vars[3].first==typeChar<double>()){

			double w=std::get<double>(mv.vars[0].second);
			double x=std::get<double>(mv.vars[1].second);
			double y=std::get<double>(mv.vars[2].second);
			double z=std::get<double>(mv.vars[3].second);
			this->ori=Eigen::Quaterniond(w,x,y,z);
		}
	}
}

void Contr::handleMesPos(varmes& mv){
	if(mv.vars.size()==3){
		if(mv.vars[0].first==typeChar<double>()&&
			mv.vars[1].first==typeChar<double>()&&
			mv.vars[2].first==typeChar<double>()){

			double x=std::get<double>(mv.vars[0].second);
			double y=std::get<double>(mv.vars[1].second);
			double z=std::get<double>(mv.vars[2].second);
			this->pos=Eigen::Vector3d(x,y,z);
		}
	}
}

void Contr::handleMesNextPos(varmes& mv){
	if(mv.vars.size()==2){
		if(mv.vars[0].first==typeChar<double>()&&
		   mv.vars[1].first==typeChar<double>()){

			double x=std::get<double>(mv.vars[0].second);
			double y=std::get<double>(mv.vars[1].second);
			this->nPos=Eigen::Vector2d(x,y);
		}
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

void Contr::handleInComms(){
	for(auto cli:this->clis){
		std::vector<std::string> msgs=cli.receive();
		for(auto msg:msgs){
			std::cout<<msg<<std::endl;
			varmes vm=cli.processMessage(msg);
			this->handleVarMessage(vm);
		}
	}
}

std::string Contr::prepareMesVel(){
	return this->srvs.prepareMessage(modStr<MOD_TYPE::CONTR>(),"vo",this->vr,this->omega);
}

void Contr::handleOutComms(){
	this->srvs.acceptsAll();
	std::string msg=this->prepareMesVel();
	this->srvs.sendsToAll(msg);
}

double symSat(double f,double th){
	return f>0?std::min(f,th):std::max(f,th);
}

void Contr::process(){
	Eigen::Vector2d relDist=this->nPos-this->pos.head(2);
	Eigen::Vector3d ydir=rotQuat(Eigen::Vector3d(0.0,1.0,0.0),this->ori);
	ydir.z()=0;
	double vn=relDist.norm()*ydir.norm();
	Eigen::Vector3d rd3;
	rd3<<relDist,0;

	double cost=ydir.dot(rd3)/vn;
	double sint=ydir.cross(rd3).z()/vn;
	double tant=sint/(1+cost);//Tangent of half angle. Period=2pi
	this->vr=this->maxV*cost;
	this->omega=symSat(tant,this->maxO);//Saturate here or in controller?
}
