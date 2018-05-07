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
	std::vector<key> neigh(){
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
	

	double w;
	double g;
	double rhs;
	double h;
	double heur(Eigen::Vector2d n){return (this->pos-n).norm();}

	dKey calcdKey(Eigen::Vector2d sst){
		return dKey(std::min(this->g,this->rhs)+this->heur(sst),std::min(this->g,this->rhs));
	}
};

double nCost(dNode l,dNode r);

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

	Pathf(const std::string& srvaddr);

	void handleVarMessage(varmes& mv);
	void handleMesPos(varmes& mv);
	void handleMesWeight(varmes& mv);

	virtual void handleOutComms();
	std::string prepareMesNextPos();

	Eigen::Vector2d calcCenter(key k);
	void insertNewNode(key k);

	void updateRhs(key k);
	void updateVertex(key k);
	void computeShortestPath();
	virtual void process();
};
