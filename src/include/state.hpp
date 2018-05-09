#pragma once
#include<map>
#include<chrono>
#include<set>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"

#define STATE_N 13
#define SENSOR_N 6


class State:public Module{
public:
	Eigen::Vector3d accel;
	Eigen::Vector3d gyro;
	Eigen::Vector4d tacho;
	std::chrono::high_resolution_clock::time_point tprev;

	Eigen::Matrix<double,STATE_N,1> xk;//State estimate
	Eigen::Matrix<double,SENSOR_N,1> zk;//Sensor data
	
	Eigen::Matrix<double,STATE_N,STATE_N> Fk;//Jacobian of kinematic model
	Eigen::Matrix<double,STATE_N,STATE_N> Pk;//Covariance of kinematic model

	Eigen::Matrix<double,SENSOR_N,STATE_N> Hk;//Jacobian of sensor model
	Eigen::Matrix<double,SENSOR_N,SENSOR_N> Rk;//Covariance of sensor model

	Eigen::Matrix<double,SENSOR_N,1> yk;//Innovation
	Eigen::Matrix<double,SENSOR_N,SENSOR_N> Sk;//Covariance of innovation

	Eigen::Matrix<double,STATE_N,SENSOR_N> Kk;//Kalman Gain


	Eigen::Vector3d pos=Eigen::Vector3d::Zero();
	Eigen::Vector3d vel=Eigen::Vector3d::Zero();
	Eigen::Quaterniond ori=Eigen::Quaterniond::Identity();
	Eigen::Vector3d rotvel=Eigen::Vector3d::Zero();
	State(const std::string& srvaddr);

	void handleVarMessage(const varmes& mv);
	void handleMesAccel(const varmes& mv);
	void handleMesGyro(const varmes& mv);
	void handleMesTacho(const varmes& mv);

	virtual void handleOutComms();
	std::string prepareMesOri();
	std::string prepareMesPos();

	void assemblezk();
	void disassemblexk();
	void calcFk();
	void calcHk();

	void predict();
	void update();
	virtual void process();
};
