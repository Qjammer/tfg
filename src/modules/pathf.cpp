#include"pathf.hpp"

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
		//this->wmap[key(x,y)]=w;
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

void Pathf::updateRhs(key k){
	double prhs=HUGE_VAL;
	key km;
	for(auto i:this->nmap[k].neigh()){
		double c=nCost(this->nmap[k],this->nmap[i]);
		if(c+this->nmap[i].g<prhs){
			prhs=c+this->nmap[i].g;
			km=i;
		}
	}
	this->nmap[k].rhs=prhs;

}

void Pathf::updateVertex(key k){
	if(this->goal!=k){
		this->updateRhs(k);
	}
	std::map<dKey,key>::iterator dk;
	if((dk=this->openQueue.find(k))!=this->openQueue.end()){
		this->openQueue.erase(dk);
	}
	if(this->nmap[k].g!=this->nmap[k].rhs){
		this->openQueue.emplace(this->nmap[k].calcdKey(this->nmap[this->curNode].pos),k);
	}
}

void Pathf::computeShortestPath(){
	dNode& strt=this->nmap[this->curNode];
	dKey skey=strt.calcdKey(strt.pos);
	while(this->nmap.begin()->first<skey||strt.rhs!=strt.g){
		auto u=this->nmap.begin();
		if(u->second.g>u->second.rhs){
			u->second.g=u->second.rhs;
			for(auto i:u->second.neigh()){
				this->updateVertex(i);
			}

		} else{
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
