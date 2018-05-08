#pragma once
#include<map>
#include<set>
#include<algorithm>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"

typedef std::pair<int,int> key;
typedef std::pair<double,double> dKey;

template<typename T,typename U>
std::pair<T,U> operator+(const std::pair<T,U>& l,const std::pair<T,U>& r){
	return {l.first+r.first,l.second+r.second};
}

inline bool operator<(const dKey& l,const dKey& r){
	return(l.first<r.first||(l.first==r.first&&l.second<r.second));
}

class dNode{
	public:
	dNode(key k,Eigen::Vector2d pos):k(k),pos(pos){}
	key k;
	Eigen::Vector2d pos;
	std::vector<key> neigh() const{
		std::vector<key> v;
		v.push_back(this->k+key{1,0});
		v.push_back(this->k+key{1,1});
		v.push_back(this->k+key{0,1});
		v.push_back(this->k+key{-1,1});
		v.push_back(this->k+key{-1,0});
		v.push_back(this->k+key{-1,-1});
		v.push_back(this->k+key{0,-1});
		v.push_back(this->k+key{1,-1});
		return v;
	}
	

	double w=2;
	double g=HUGE_VAL;
	double rhs=HUGE_VAL;
	double h=HUGE_VAL;
	double heur(Eigen::Vector2d n) const{return (this->pos-n).norm();}

	dKey calcdKey(Eigen::Vector2d sst,double km) const{
		return dKey(std::min(this->g,this->rhs)+this->heur(sst)+km,std::min(this->g,this->rhs));
	}
};

double nCost(const dNode& l,const dNode& r);

class Pathf:public Module{
public:
	Eigen::Vector3d pos;
	Eigen::MatrixXd path;
	key curNode;
	key goal;
	Eigen::Vector2d nextPos;
	Eigen::Vector2d nd={0.5,0.5};
	std::map<key,dNode> nmap;
	std::map<dKey,key> openQueue;
	std::set<key> newWeights;
	double km=0;

	Pathf(const std::string& srvaddr);

	key calcKey(Eigen::Vector2d p);

	void handleVarMessage(varmes& mv);
	void handleMesPos(const varmes& mv);
	void handleMesWeight(const varmes& mv);
	void handleMesGoal(const varmes& mv);

	virtual void handleOutComms();
	std::string prepareMesNextPos() const;

	Eigen::Vector2d calcCenter(const key& k) const;
	void insertNewNode(const key& k);

	void updateRhs(const key& k);
	void updateVertex(const key& k);
	void computeShortestPath();
	virtual void process();
};
