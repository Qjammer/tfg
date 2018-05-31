#pragma once
#include<map>
#include<set>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"

class Contr:public Module{
public:
	const std::vector<Eigen::Vector3d> wheelpos={
		{0.1,0.2,0.0},//RF
		{-0.1,0.2,0.0},//LF
		{0.1,-0.2,0.0},//RB
		{-0.1,-0.2,0.0}};//LB
	Eigen::Vector4d ws=Eigen::Vector4d::Zero();
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
	std::string prepareMesWheels();

	virtual void process();
	double calcwheelspeed(const Eigen::Vector3d& wp);
};
