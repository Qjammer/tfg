#pragma once
#include<map>
#include<set>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"

typedef Eigen::Vector3d point;
typedef std::pair<int,int> key;


class bucket{
	public:
	Eigen::Vector2d c;
	std::vector<point> pQueue;
	int pCounter;
	Eigen::Matrix3d FFo;
	Eigen::Vector3d FZo;
	Eigen::Vector3d beta;
	double z;
	double w;

	bucket(Eigen::Vector2d c);

	bool addPoints(const std::vector<point>& pts);
	bool addPoint(point p);
	void processPoints();
	double calcWeight();

	bool calcAllowed();
	bool calcNeeded();
};

class buckMap{
public:
	std::map<key,bucket> m;
	Eigen::Vector2d nd;
	std::set<key> pb;//Pending buckets to process
	std::set<key> ub;//Updated buckets

	buckMap(Eigen::Vector2d nd);

	void processPendingBuckets();

	void insertPoint(point p);
	bool insertPointToKey(point p,key k);

	key calcKey(point p);
};

typedef point LIDARPoint;

class EnvRec:public Module{
public:
	std::vector<LIDARPoint> unp;//Unprocessed points
	buckMap bm;
	Eigen::Quaterniond ori=Eigen::Quaterniond::Identity();
	Eigen::Vector3d pos=Eigen::Vector3d::Zero();

	EnvRec(const std::string& srvaddr):Module(MOD_TYPE::ENVREC,srvaddr),bm(Eigen::Vector2d(1.0,1.0)){}

	void preprocessPoints(){
		for(auto i:this->unp){
			//TODO:Transform

			//Stuff with quaternions
			Eigen::Quaterniond q;
			q.w()=0;
			q.vec()=i;
			auto r=this->pos+(this->ori*q*this->ori.inverse()).vec();
			this->bm.insertPoint(r);
		}
		this->unp.clear();
	}

	void handleMesOri(varmes& mv){
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

	void handleMesPos(varmes& mv){
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

	void handleVarMessage(varmes& mv){
		if(mv.sender==modStr<MOD_TYPE::STATE>()){
			if(mv.purpose=="or"){
				this->handleMesOri(mv);
			}else if(mv.purpose=="ps"){
				this->handleMesPos(mv);
			}
		
		}
	
	}

	void handleInComms(){
		for(auto cli:this->clis){
			std::vector<std::string> msgs=cli.receive();
			for(auto msg:msgs){
				std::cout<<msg<<std::endl;
				varmes vm=cli.processMessage(msg);
				this->handleVarMessage(vm);
			}
			
		}
	}

	std::string prepareMesBucket(key k){
		int kx=k.first;
		int ky=k.second;
		double w=this->bm.m.find(k)->second.w;
		return this->srvs.prepareMessage(modStr<MOD_TYPE::ENVREC>(),"nw",kx,ky,w);
	}

	void handleOutComms(){
		this->srvs.acceptsAll();
		for(auto k:this->bm.ub){
			std::string msg=this->prepareMesBucket(k);
			this->srvs.sendsToAll(msg);
		}
	}

	void loop(){
		while(true){
			this->handleInComms();
			this->process();
			this->handleOutComms();
		}
	}
	
	void process(){
		this->preprocessPoints();
		this->bm.processPendingBuckets();
	}
	

};
