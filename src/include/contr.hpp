#pragma once
#include<map>
#include<set>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"

class Contr:public Module{
public:
	Eigen::Quaterniond ori=Eigen::Quaterniond::Identity();
	Eigen::Vector3d pos=Eigen::Vector3d::Zero();
	Eigen::Vector2d nPos=Eigen::Vector2d::Zero();//NextNode
	double vr=0;
	double omega=0;
	const double maxV=5;
	const double maxO=5;

	Contr(const std::string& srvaddr);

	void handleVarMessage(varmes& mv);
	void handleMesOri(varmes& mv);
	void handleMesPos(varmes& mv);
	void handleMesNextPos(varmes& mv);

	virtual void handleOutComms();
	std::string prepareMesVel();

	virtual void process();
};
