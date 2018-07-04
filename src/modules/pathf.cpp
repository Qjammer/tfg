#include"pathf.hpp"

typedef std::map<key,dNode>::iterator mapIt;

double nCost(const dNode& l,const dNode& r){
	return 0.5*(l.w+r.w)*(l.pos-r.pos).norm();
	//return (l.w+r.w)*(l.pos-r.pos).norm();
}

Pathf::Pathf(const std::string& srvaddr):Module(MOD_TYPE::CONTR,srvaddr){}

key Pathf::calcKey(Eigen::Vector2d p){
	int kx=std::floor(p.x()/this->nd.x()+0.5);
	int ky=std::floor(p.y()/this->nd.y()+0.5);
	return key(kx,ky);
}

void Pathf::handleMesPos(const varmes& mv){
	if(varCond<double,double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		makeMesVar(double,z,2);
		this->pos=Eigen::Vector3d(x,y,z);
		key newkey=this->calcKey(this->pos.head<2>());
		if(newkey!=this->curNode){
			auto it1=this->nmap.find(this->curNode);
			if(it1==this->nmap.end()){
				this->insertNewNode(this->curNode);
				it1=this->nmap.find(this->curNode);
			}
			auto it2=this->nmap.find(newkey);
			if(it2==this->nmap.end()){
				this->insertNewNode(newkey);
				it2=this->nmap.find(newkey);
			}
			this->km+=it1->second.heur(it2->second.pos);
			this->curNode=newkey;
		}
	}
}

void Pathf::handleMesWeight(const varmes& mv){
	if(varCond<int,int,double>(mv)){
		makeMesVar(int,x,0);
		makeMesVar(int,y,1);
		makeMesVar(double,w,2);
		key k(x,y);
		mapIt it=this->nmap.find(k);
		if(it==this->nmap.end()){
			insertNewNode(k);
			it=this->nmap.find(k);
		}
		it->second.w=w;
		this->newWeights.insert(k);
	}
}

void Pathf::handleMesGoal(const varmes& mv){
	if(varCond<int,int>(mv)){
		makeMesVar(int,x,0);
		makeMesVar(int,y,1);
		//TODO: Call function create node if necessary and add to changed nodes
		key k(x,y);
		this->goal=k;
		this->openQueue.clear();
		this->km=0;
		for(auto& n:this->nmap){
			n.second.rhs=HUGE_VAL;
			n.second.g=HUGE_VAL;
		}
		auto goal=this->nmap.find(k);
		if(goal==this->nmap.end()){
			this->insertNewNode(k);
			goal=this->nmap.find(k);
		}
		goal->second.rhs=0;
		auto start=this->nmap.find(this->curNode);
		if(start==this->nmap.end()){
			this->insertNewNode(this->curNode);
			start=this->nmap.find(this->curNode);
		}
		this->openQueue.emplace(goal->second.calcdKey(start->second.pos,this->km),k);
	}
}

void Pathf::handleVarMessage(const varmes& mv){
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
	if(mv.sender==modStr<MOD_TYPE::SUPER>()){
		if(mv.purpose=="ng"){
			this->handleMesGoal(mv);
		}
	}
}

std::string Pathf::prepareMesNextPos() const{
	return this->srvs.prepareMessage(modStr<MOD_TYPE::PATHF>(),"np",this->nextPos.x(),this->nextPos.y());
}

void Pathf::handleOutComms(){
	this->srvs.acceptsAll();
	std::string msg=this->prepareMesNextPos();
	this->srvs.sendsToAll(msg);
}

Eigen::Vector2d Pathf::calcCenter(const key& k) const{
	double cx=k.first*this->nd.x();
	double cy=k.second*this->nd.y();
	return {cx,cy};
}

void Pathf::insertNewNode(const key& k){
	if(this->nmap.find(k)==this->nmap.end()){
		this->nmap.emplace(k,dNode(k,this->calcCenter(k)));
	}
}

void Pathf::updateRhs(const key& k){
	double prhs=HUGE_VAL;
	key kmin;
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
		if((c+neig->second.g)<prhs){
			prhs=c+neig->second.g;
			kmin=i;
		}
	}
	it->second.rhs=prhs;
}

void Pathf::updateVertex(const key& k){
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
		mapIt ss=this->nmap.find(this->curNode);
		if(ss==this->nmap.end()){
			this->insertNewNode(this->curNode);
			ss=this->nmap.find(this->curNode);
		}
		this->openQueue.emplace(it->second.calcdKey(ss->second.pos,this->km),k);
	}
}

void Pathf::computeShortestPath(){
	mapIt cur=this->nmap.find(this->curNode);
	if(cur==this->nmap.end()){
		this->insertNewNode(this->curNode);
		cur=this->nmap.find(this->curNode);
	}
	dNode& strt=cur->second;
	auto u=this->openQueue.begin();
	while((u=this->openQueue.begin())!=this->openQueue.end()&&(u->first<strt.calcdKey(strt.pos,this->km)||strt.rhs!=strt.g)){
		dKey kold=u->first;
		key k=u->second;
		this->openQueue.erase(u);
		mapIt it=this->nmap.find(k);
		dKey knew=it->second.calcdKey(strt.pos,this->km);
		if(kold<knew){
			std::cout<<"Updating queue"<<std::endl;
			this->openQueue.emplace(knew,k);
		}else if(it->second.g>it->second.rhs){
			it->second.g=it->second.rhs;
			for(auto i:it->second.neigh()){
				this->updateVertex(i);
			}
		}else{
			it->second.g=HUGE_VAL;
			this->updateVertex(u->first);
			for(auto i:it->second.neigh()){
				this->updateVertex(i);
			}
		}
	}
}

std::vector<key> Pathf::getPath(){
	key cur=this->curNode;
	std::vector<key> v;
	v.push_back(cur);
	double cost=this->nmap.find(cur)->second.g;

	while(cur!=this->goal){
		double nextcost=HUGE_VAL;
		key nextkey;

		for(auto&n:this->nmap.find(cur)->second.neigh()){
			auto nit=this->nmap.find(n);
			if (nit->second.g<nextcost){
				nextcost=nit->second.g;
				nextkey=nit->second.k;
			}
		
		}
		cur=nextkey;
		v.push_back(cur);
	}
	return v;


}

void Pathf::process(){
	//update vertex that have changed
	std::cout<<"Updated Weights: "<<this->newWeights.size()<<std::endl;
	for(auto n:this->newWeights){
		this->updateVertex(n);
	}
	this->newWeights.clear();
	//compute shortest path
	this->computeShortestPath();
}
