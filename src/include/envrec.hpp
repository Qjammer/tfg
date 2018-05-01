#pragma once
#include<map>
#include<set>
#include<eigen3/Eigen/Eigen>
#include"Module.hpp"
#include"Socket.hpp"

typedef Eigen::Vector3d point;
typedef std::pair<int,int> key;


class bucket{
	public:
	Eigen::Vector2d c;
	std::vector<point> pQueue;
	int pCounter;
	Eigen::Matrix3d FFo;
	Eigen::Vector3d FZo;
	Eigen::Vector3d beta;
	double z;
	double w;

	bucket(Eigen::Vector2d c);

	bool addPoints(const std::vector<point>& pts);
	bool addPoint(point p);
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

	void insertPoint(point p);
	bool insertPointToKey(point p,key k);

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

	void handleInComms();
	void handleMesOri(varmes& mv);
	void handleMesPos(varmes& mv);
	void handleVarMessage(varmes& mv);


	void handleOutComms();
	std::string prepareMesBucket(key k);

	void loop();
	void process();
};
