#include"pathf.hpp"

typedef std::map<key,dNode>::iterator mapIt;

double nCost(dNode l,dNode r){
	return 0.5*(l.w+r.w)*(l.pos-r.pos).norm();
}

Pathf::Pathf(const std::string& srvaddr):Module(MOD_TYPE::CONTR,srvaddr){}

void Pathf::handleMesPos(varmes& mv){
	if(varCond<double,double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		makeMesVar(double,z,2);
		this->pos=Eigen::Vector3d(x,y,z);
	}
}

void Pathf::handleMesWeight(varmes& mv){
	if(varCond<int,int,double>(mv)){
		makeMesVar(int,x,0);
		makeMesVar(int,y,1);
		makeMesVar(double,w,2);
		//TODO: Call function create node if necessary and add to changed nodes
		key k(x,y);
		mapIt it=this->nmap.find(k);
		if(it==this->nmap.end()){
			insertNewNode(k);
			it=this->nmap.find(k);
		}
		it->second.w=w;
	}
}

void Pathf::handleVarMessage(varmes& mv){
	if(mv.sender==modStr<MOD_TYPE::STATE>()){
		if(mv.purpose=="ps"){
			this->handleMesPos(mv);
		}
	}
	if(mv.sender==modStr<MOD_TYPE::ENVREC>()){
		if(mv.purpose=="nw"){
			this->handleMesWeight(mv);
		}
	}
}

std::string Pathf::prepareMesNextPos(){
	return this->srvs.prepareMessage(modStr<MOD_TYPE::PATHF>(),"np",this->nextPos.x(),this->nextPos.y());
}

void Pathf::handleOutComms(){
	this->srvs.acceptsAll();
	std::string msg=this->prepareMesNextPos();
	this->srvs.sendsToAll(msg);
}

Eigen::Vector2d Pathf::calcCenter(key k){
	double cx=k.first*this->nd.x();
	double cy=k.second*this->nd.y();
	return {cx,cy};
}

void Pathf::insertNewNode(key k){
	if(this->nmap.find(k)==this->nmap.end()){
		this->nmap.emplace(k,dNode(k,this->calcCenter(k)));
		std::cout<<"Number of nodes: "<<this->nmap.size()<<std::endl;
	}
}

void Pathf::updateRhs(key k){
	double prhs=HUGE_VAL;
	key km;
	mapIt it=this->nmap.find(k);
	if(it==this->nmap.end()){
		this->insertNewNode(k);
		it=this->nmap.find(k);
	}
	for(auto i:it->second.neigh()){
		mapIt neig=this->nmap.find(i);
		if(neig==this->nmap.end()){
			this->insertNewNode(i);
			neig=this->nmap.find(i);
		}
		double c=nCost(it->second,neig->second);
		if(c+it->second.g<prhs){
			prhs=c+neig->second.g;
			km=i;
		}
	}
	it->second.rhs=prhs;
}

void Pathf::updateVertex(key k){
	if(this->goal!=k){
		this->updateRhs(k);
	}
	std::map<dKey,key>::iterator dk=this->openQueue.find(k);
	if(dk!=this->openQueue.end()){
		this->openQueue.erase(dk);
	}
	mapIt it=this->nmap.find(k);
	if(it==this->nmap.end()){
		this->insertNewNode(k);
		it=this->nmap.find(k);
	}
	if(it->second.g!=it->second.rhs){
		this->openQueue.emplace(it->second.calcdKey(this->nmap.find(this->curNode)->second.pos),k);
	}
}

void Pathf::computeShortestPath(){
	mapIt it=this->nmap.find(this->curNode);
	if(it==this->nmap.end()){
		this->insertNewNode(this->curNode);
		it=this->nmap.find(this->curNode);
	}
	dNode& strt=it->second;
	dKey skey=strt.calcdKey(strt.pos);
	while(this->nmap.begin()->first<skey||strt.rhs!=strt.g){
		auto u=this->nmap.begin();
		if(u->second.g>u->second.rhs){
			u->second.g=u->second.rhs;
			for(auto i:u->second.neigh()){
				this->updateVertex(i);
			}
		}else{
			u->second.g=HUGE_VAL;
			this->updateVertex(u->first);
			for(auto i:u->second.neigh()){
				this->updateVertex(i);
			}
		}
	}
}

void Pathf::process(){

}
