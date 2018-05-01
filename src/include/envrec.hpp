#include<map>
#include<set>
#include<eigen3/Eigen/Eigen>
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

	buckMap(Eigen::Vector2d nd);

	void processPendingBuckets();

	bool insertPoint(point p);
	bool insertPointToKey(point p,key k);

	key calcKey(point p);
};

class envModule{
public:
	buckMap bm;


};
