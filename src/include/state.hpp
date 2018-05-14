#pragma once
#include<map>
#include<chrono>
#include<set>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"



class State:public Module{
public:
	#define SENSOR_N 6
	Eigen::Vector3d accelSens;
	Eigen::Vector3d gyro;
	Eigen::Vector4d tacho;
	std::chrono::high_resolution_clock::time_point tprev;
	std::chrono::high_resolution_clock::duration dt;

	void updateT();
	#define STATE_N 16
	Eigen::Vector3d pos=Eigen::Vector3d::Zero();
	Eigen::Vector3d vel=Eigen::Vector3d::Zero();
	Eigen::Vector3d accelState=Eigen::Vector3d::Zero();
	Eigen::Quaterniond ori=Eigen::Quaterniond::Identity();
	Eigen::Vector3d rotvel=Eigen::Vector3d::Zero();

	Eigen::Matrix<double,STATE_N,1> xk;//State estimate
	Eigen::Matrix<double,SENSOR_N,1> zk;//Sensor data
	
	Eigen::Matrix<double,STATE_N,STATE_N> Fk;//Jacobian of kinematic model
	Eigen::Matrix<double,STATE_N,STATE_N> Pk;//Covariance of kinematic model

	Eigen::Matrix<double,SENSOR_N,STATE_N> Hk;//Jacobian of sensor model
	Eigen::Matrix<double,SENSOR_N,SENSOR_N> Rk;//Covariance of sensor model

	Eigen::Matrix<double,SENSOR_N,1> yk;//Innovation
	Eigen::Matrix<double,SENSOR_N,SENSOR_N> Sk;//Covariance of innovation

	Eigen::Matrix<double,STATE_N,SENSOR_N> Kk;//Kalman Gain


	State(const std::string& srvaddr);

	void handleVarMessage(const varmes& mv);
	void handleMesAccel(const varmes& mv);
	void handleMesGyro(const varmes& mv);
	void handleMesTacho(const varmes& mv);

	virtual void handleOutComms();
	std::string prepareMesOri();
	std::string prepareMesPos();

	void assemblexk();
	void disassemblexk();
	void assemblezk();
	void calcFk();

	Eigen::Matrix<double,3,4> Jquatrotate(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);
	Eigen::Matrix<double,3,STATE_N> JaccelSens();
	Eigen::Matrix<double,3,STATE_N> JrotvelSens();
	void calcHk();

	void predict();
	void update();
	virtual void process();
};
