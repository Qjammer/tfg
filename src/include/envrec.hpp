#pragma once
#include<map>
#include<set>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"

typedef Eigen::Vector3d point;
typedef Eigen::Vector4d pointw;
typedef std::pair<int,int> key;


class bucket{
	public:
	Eigen::Vector2d c;
	std::vector<pointw> pQueue;
	int pCounter;
	Eigen::Matrix3d FWFo;
	Eigen::Vector3d FWZo;
	Eigen::Vector3d beta;
	double z;
	double w=2;

	bucket(Eigen::Vector2d c);

	bool addPoints(const std::vector<pointw>& pts);
	bool addPoint(pointw p);
	void processPoints();
	double calcWeight();

	bool calcAllowed();
	bool calcNeeded();
};

class buckMap{
public:
	std::map<key,bucket> m;
	Eigen::Vector2d nd;
	std::set<key> pb;//Pending buckets to process
	std::set<key> ub;//Updated buckets

	buckMap(Eigen::Vector2d nd);

	void processPendingBuckets();

	void insertPoint(pointw p);
	bool insertPointToKey(pointw p,key k);

	key calcKey(point p);
};

typedef point LIDARPoint;

class EnvRec:public Module{
public:
	std::vector<LIDARPoint> unp;//Unprocessed points
	buckMap bm;
	Eigen::Quaterniond ori=Eigen::Quaterniond::Identity();
	Eigen::Vector3d pos=Eigen::Vector3d::Zero();

	EnvRec(const std::string& srvaddr);

	void preprocessPoints();

	void handleMesOri(const varmes& mv);
	void handleMesPos(const varmes& mv);
	void handleLIDARPoint(const varmes& mv);
	void handleVarMessage(const varmes& mv) override;


	virtual void handleOutComms();
	std::string prepareMesBucket(key k);

	//void loop();
	virtual void process();
};
