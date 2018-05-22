#pragma once
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"

class Sens:public Module{
public:
	std::vector<Eigen::Vector3d> gyro;
	std::vector<Eigen::Vector3d> accel;

	Sens(const std::string& srvaddr);

	void handleVarMessage(varmes& mv);

	virtual void handleOutComms();
	std::vector<std::string> prepareMesGyros();
	std::string prepareMesGyro(const Eigen::Vector3d& v) const;
	std::vector<std::string> prepareMesAccels();
	std::string prepareMesAccel(const Eigen::Vector3d& v) const;

	virtual void process();
};
