#pragma once
#include<map>
#include<set>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"

class State:public Module{
public:
	Eigen::Vector3d accel;
	Eigen::Vector3d gyro;
	Eigen::Vector4d tacho;
	std::chrono::high_resolution_clock::time_point tprev;


	Eigen::Vector3d pos=Eigen::Vector3d::Zero();
	Eigen::Quaterniond ori=Eigen::Quaterniond::Identity();
	State(const std::string& srvaddr);

	void handleVarMessage(const varmes& mv);
	void handleMesAccel(const varmes& mv);
	void handleMesGyro(const varmes& mv);
	void handleMesTacho(const varmes& mv);

	virtual void handleOutComms();
	std::string prepareMesOri();
	std::string prepareMesPos();

	void predict();
	void update();
	virtual void process();
};
