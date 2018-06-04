#pragma once
#include<map>
#include<chrono>
#include<set>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"



class State:public Module{
public:
	#define SENSOR_N 10
	Eigen::Vector3d accelSens={0.0,0.0,9.806};
	Eigen::Vector3d gyro={0.0,0.0,0.0};
	Eigen::Vector4d tacho={0.0,0.0,0.0,0.0};
	std::chrono::high_resolution_clock::time_point tprev;
	std::chrono::high_resolution_clock::duration dt;
	double dts;

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
	Eigen::Matrix<double,STATE_N,1> expectedxk();
	void disassemblexk();
	Eigen::Matrix<double,9,STATE_N> linFk();
	Eigen::Matrix<double,3,STATE_N> Jx();
	Eigen::Matrix<double,3,STATE_N> Jv();
	Eigen::Matrix<double,3,STATE_N> Ja();
	Eigen::Matrix<double,7,STATE_N> rotFk();
	Eigen::Matrix<double,4,STATE_N> Jq();
	Eigen::Matrix<double,3,STATE_N> Jw();
	void calcFk();

	void assemblezk();
	Eigen::Matrix<double,SENSOR_N,1> expectedzk();
	Eigen::Matrix<double,3,4> Jquatrotate(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);
	Eigen::Matrix<double,3,STATE_N> JaccelSens();
	Eigen::Matrix<double,3,STATE_N> JrotvelSens();
	std::vector<Eigen::Vector3d> wheelPos={
		{0.1,0.2,0},
		{-0.1,0.2,0},
		{0.1,-0.2,0},
		{-0.1,-0.2,0}};
	Eigen::Matrix<double,1,STATE_N> JtachoSens(Eigen::Vector3d& r);
	void calcHk();

	void predict();
	void update();
	virtual void process();
};
