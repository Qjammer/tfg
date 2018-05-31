#include"envrec.hpp"

bucket::bucket(Eigen::Vector2d c):
	c(c),pQueue(0),pCounter(0),
	FWFo(Eigen::Matrix3d::Zero()),
	FWZo(Eigen::Vector3d::Zero()),
	beta(Eigen::Vector3d::Zero()),
	z(0),w(2)
{

}
bool bucket::addPoint(pointw p){
	this->pQueue.push_back(p);
	return this->calcNeeded();
}

bool bucket::addPoints(const std::vector<pointw>& pts){
	this->pQueue.insert(this->pQueue.end(),pts.begin(),pts.end());
	return this->calcNeeded();
}

bool bucket::calcAllowed(){
	return this->pCounter+this->pQueue.size()>3;//Returns true if a recalculation can be done and won't return an underdetermined system
}

bool bucket::calcNeeded(){
	return this->calcAllowed()&&this->pQueue.size()>2;
}

void bucket::processPoints(){
	if(!this->calcAllowed()){return;}
	Eigen::MatrixXd F(3,this->pQueue.size());
	Eigen::VectorXd Z(this->pQueue.size());
	Eigen::SparseMatrix<double> W(this->pQueue.size(),this->pQueue.size());
	for(unsigned int i=0;i<this->pQueue.size();++i){
		F.col(i)<<1,this->pQueue[i].x(),this->pQueue[i].y();
		Z.row(i)<<this->pQueue[i].z();
		W.insert(i,i)=(this->pQueue[i][3]);
	}
	W.makeCompressed();
	this->pCounter+=this->pQueue.size();
	this->pQueue.clear();

	Eigen::Matrix3d FWFn=F*W*F.transpose();
	Eigen::Vector3d FWZn=F*W*Z;

	this->FWFo=FWFn+this->FWFo;
	this->FWZo=FWZn+this->FWZo;

	this->beta= this->FWFo.llt().solve(this->FWZo);
	this->z=this->beta.dot(Eigen::Vector3d(1,this->c.x(),this->c.y()));

	this->calcWeight();
}

double bucket::calcWeight(){
	Eigen::Vector3d n(this->beta[1],this->beta[2],1);
	double t2=(pow(n.x(),2)+pow(n.y(),2))/(pow(n.z(),2));
	this->w=1+2*pow(t2,0.625);
	return this->w;
}

buckMap::buckMap(Eigen::Vector2d nd):nd(nd){}

void buckMap::processPendingBuckets(){
	for(auto i:this->pb){
		this->m.find(i)->second.processPoints();
		this->ub.insert(i);
	}
}

void buckMap::insertPoint(pointw p){
	key k=this->calcKey(p.head<3>());
	if(this->insertPointToKey(p,k)){
		this->pb.insert(k);
	}
}

bool buckMap::insertPointToKey(pointw p,key k){
	auto f=this->m.find(k);
	if(f==this->m.end()){
		this->m.emplace(k,Eigen::Vector2d(k.first*this->nd.x(),k.second*this->nd.y()));
	}
	return this->m.find(k)->second.addPoint(p);
}

key buckMap::calcKey(point p){
	int kx=std::floor(p.x()/this->nd.x()+0.5);
	int ky=std::floor(p.y()/this->nd.y()+0.5);
	return key(kx,ky);
}

EnvRec::EnvRec(const std::string& srvaddr):Module(MOD_TYPE::ENVREC,srvaddr),bm(Eigen::Vector2d(1.0,1.0)){}

void EnvRec::preprocessPoints(){
		for(auto i:this->unp){
			Eigen::Quaterniond q1;
			q1.w()=cos(i[0]/2);
			q1.vec()=Eigen::Vector3d::Zero();
			q1.vec().x()=sin(i[0]/2);

			Eigen::Quaterniond q2;
			q2.w()=cos(i[1]/2);
			q2.vec()=Eigen::Vector3d::Zero();
			q2.vec().z()=sin(i[1]/2);

			Eigen::Quaterniond qtotal=this->ori.inverse()*q2*q1;

			Eigen::Quaterniond v;
			v.w()=0;
			v.vec()=Eigen::Vector3d::Zero();
			v.vec().y()=i[2];
			Eigen::Vector3d rel=(qtotal*v*qtotal.inverse()).vec();

			pointw xyzw;
			xyzw.head<3>()=this->pos+rel;
			const double varr=0.01;//meters
			const double vartheta=0.017;//radians
			const double varphi=0.017;//radians
			double variance=varr+i[2]*i[2]*(vartheta+varphi);
			xyzw.tail<1>()[0]=1/variance;
			std::cout<<"xyzw"<<std::endl<<xyzw<<std::endl<<std::endl;
			std::cout<<"variance"<<std::endl<<variance<<std::endl<<std::endl;
			this->bm.insertPoint(xyzw);
		}
		this->unp.clear();
	}

void EnvRec::handleMesOri(const varmes& mv){
	if(varCond<double,double,double,double>(mv)){
		makeMesVar(double,w,0);
		makeMesVar(double,x,1);
		makeMesVar(double,y,2);
		makeMesVar(double,z,3);
		this->ori=Eigen::Quaterniond(w,x,y,z);
	}
}

void EnvRec::handleMesPos(const varmes& mv){
	if(varCond<double,double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		makeMesVar(double,z,2);
		this->pos=Eigen::Vector3d(x,y,z);
	}
}

void EnvRec::handleLIDARPoint(const varmes& mv){
	if(varCond<double,double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		makeMesVar(double,z,2);
		this->unp.emplace_back(x,y,z);
	}
}

void EnvRec::handleVarMessage(const varmes& mv){
	std::cout<<mv.sender<<" "<<mv.purpose<<std::endl;
	if(mv.sender==modStr<MOD_TYPE::STATE>()){
		if(mv.purpose=="or"){
			this->handleMesOri(mv);
		}else if(mv.purpose=="ps"){
			this->handleMesPos(mv);
		}
	} else if(mv.sender==modStr<MOD_TYPE::EXTCOM>()){
		if(mv.purpose=="li"){
			this->handleLIDARPoint(mv);
		}
	}
}

std::string EnvRec::prepareMesBucket(key k){
	int kx=k.first;
	int ky=k.second;
	double w=this->bm.m.find(k)->second.w;
	return this->srvs.prepareMessage(modStr<MOD_TYPE::ENVREC>(),"nw",kx,ky,w);
}

void EnvRec::handleOutComms(){
	this->srvs.acceptsAll();
	for(auto k:this->bm.ub){
		std::string msg=this->prepareMesBucket(k);
		this->srvs.sendsToAll(msg);
	}
}

void EnvRec::process(){
	this->preprocessPoints();
	this->bm.processPendingBuckets();
}
