#include<map>
#include<eigen3/Eigen/Eigen>
typedef Eigen::Vector3d point;

class bucket{

	public:
	std::vector<point> pQueue;
	int pCounter;
	Eigen::Matrix3d FFo;
	Eigen::Vector3d FZo;
	Eigen::Vector3d beta;
	double w;


	bucket():pQueue(0),pCounter(0),FFo(Eigen::Matrix3d::Zero()),FZo(Eigen::Vector3d::Zero()){
	
	}
	void processPoints(){
		Eigen::MatrixXd F(3,this->pQueue.size());
		Eigen::VectorXd Z(this->pQueue.size());
		for(unsigned int i=0;i<this->pQueue.size();++i){
			//TODO:Add weights
			F.col(i)<<1,this->pQueue[i].x(),this->pQueue[i].y();
			Z.row(i)<<this->pQueue[i].z();
		}
		this->pCounter+=this->pQueue.size();
		this->pQueue.clear();

		Eigen::Matrix3d FFn=F*Eigen::Transpose(F);
		Eigen::Vector3d FZn=F*Z;

		this->FFo=FFn+this->FFo;
		this->FZo=FZn+this->FZo;

		this->beta= this->FFo.llt().solve(this->FZo);
	}

};
