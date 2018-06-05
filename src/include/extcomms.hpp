#pragma once
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"ArduinoHndlr.hpp"

class ExtComms:public Module{
public:
	std::vector<ArduinoHandler> ardhndls;
	
	//Incoming data
	std::vector<Eigen::Vector3d> accel;
	std::vector<Eigen::Vector3d> gyro;
	std::vector<Eigen::Vector3d> lidarpts;
	std::vector<Eigen::Vector4d> tacho;
	//Outgoing Data
	std::vector<Eigen::Vector4d> wheels;

	ExtComms(const std::string& srvaddr);

	//
	//Receive Messages
	//
	virtual void handleInComms();
	std::vector<varmes> handleInArduino(int i);
	void handleVarMessage(const varmes& mv);
	//External messages
	void handleMesAccel(const varmes& mv);
	void handleMesGyro(const varmes& mv);
	void handleMesLIDARPoint(const varmes& mv);
	void handleMesTacho(const varmes& mv);
	
	//Kernel messages
	void handleMesWheels(const varmes& mv);
	//void handleMesLIDARTower(const varmes& mv);
	//void handleMesArm(const varmes& mv);

	//
	//Propagate Messages
	//
	virtual void handleOutComms();
	//External messages
	std::vector<std::string> prepareMesWheels();
	std::string prepareMesWheel(const Eigen::Vector4f& v) const;
	
	//Kernel messages
	std::vector<std::string> prepareMesGyros();
	std::string prepareMesGyro(const Eigen::Vector3d& v) const;
	std::vector<std::string> prepareMesAccels();
	std::string prepareMesAccel(const Eigen::Vector3d& v) const;
	std::vector<std::string> prepareMesLIDARPts();
	std::string prepareMesLIDARPt(const Eigen::Vector3d& v) const;
	std::vector<std::string> prepareMesTachos();
	std::string prepareMesTacho(const Eigen::Vector4d& v) const;

	virtual void process();
};
