#include"envrec.hpp"

bucket::bucket(Eigen::Vector2d c):
	c(c),pQueue(0),pCounter(0),
	FFo(Eigen::Matrix3d::Zero()),
	FZo(Eigen::Vector3d::Zero()),
	beta(Eigen::Vector3d::Zero()),
	z(0),w(0)
{

}
bool bucket::addPoint(point p){
	this->pQueue.push_back(p);
	return this->calcNeeded();
}

bool bucket::addPoints(const std::vector<point>& pts){
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
	for(unsigned int i=0;i<this->pQueue.size();++i){
		//TODO:Add weights
		F.col(i)<<1,this->pQueue[i].x(),this->pQueue[i].y();
		Z.row(i)<<this->pQueue[i].z();
	}
	this->pCounter+=this->pQueue.size();
	this->pQueue.clear();

	Eigen::Matrix3d FFn=F*Eigen::Transpose(F);
	Eigen::Vector3d FZn=F*Z;

	this->FFo=FFn+this->FFo;
	this->FZo=FZn+this->FZo;

	this->beta= this->FFo.llt().solve(this->FZo);
	this->z=this->beta.dot(Eigen::Vector3d(1,this->c.x(),this->c.y()));
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

bool buckMap::insertPoint(point p){
	key k=this->calcKey(p);
	if(this->insertPointToKey(p,k)){
		this->pb.insert(k);
	}
}

bool buckMap::insertPointToKey(point p,key k){
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
