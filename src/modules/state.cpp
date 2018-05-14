#include<eigen3/unsupported/Eigen/CXX11/Tensor>
#include"state.hpp"

Eigen::Matrix3d skewSym(Eigen::Vector3d& v){
	Eigen::Matrix3d m;
	m<<
		     0, -v.z(),  v.y(),
		 v.z(),      0, -v.x(),
		-v.y(),  v.x(),      0;
	return m;
}

State::State(const std::string& srvaddr):Module(MOD_TYPE::CONTR,srvaddr){}

void State::handleMesAccel(const varmes& mv){
	if(varCond<double,double,double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		makeMesVar(double,z,2);
		this->accelSens=Eigen::Vector3d(x,y,z);
	}
}

void State::handleMesGyro(const varmes& mv){
	if(varCond<double,double,double>(mv)){
		makeMesVar(double,x,0);
		makeMesVar(double,y,1);
		makeMesVar(double,z,2);
		this->gyro=Eigen::Vector3d(x,y,z);
	}
}

void State::handleMesTacho(const varmes& mv){
	if(varCond<double,double,double,double>(mv)){
		makeMesVar(double,w1,0);
		makeMesVar(double,w2,1);
		makeMesVar(double,w3,2);
		makeMesVar(double,w4,3);
		this->tacho=Eigen::Vector4d(w1,w2,w3,w4);
	}
}

void State::handleVarMessage(const varmes& mv){
	if(mv.sender==modStr<MOD_TYPE::SENS>()){
		if(mv.purpose=="ac"){
			this->handleMesAccel(mv);
		}else if(mv.purpose=="gy"){
			this->handleMesGyro(mv);
		}else if(mv.purpose=="ta"){
			this->handleMesTacho(mv);
		}
	}
}

std::string State::prepareMesOri(){
	return this->srvs.prepareMessage(modStr<MOD_TYPE::STATE>(),"or",this->ori.w(),this->ori.vec().x(),this->ori.vec().y(),this->ori.vec().z());
}

std::string State::prepareMesPos(){
	return this->srvs.prepareMessage(modStr<MOD_TYPE::STATE>(),"ps",this->pos[0],this->pos[1],this->pos[2]);
}

void State::handleOutComms(){
	this->srvs.acceptsAll();
	std::string msgori=this->prepareMesOri();
	std::string msgpos=this->prepareMesPos();
	this->srvs.sendsToAll(msgori);
	this->srvs.sendsToAll(msgpos);
}

void State::calcFk(){

	Eigen::Matrix<double,9,STATE_N> linSubmatrix;

	const Eigen::Matrix3d id3=Eigen::Matrix3d::Identity();
	const Eigen::Matrix3d zeros3=Eigen::Matrix3d::Zero();
	const Eigen::Matrix<double,3,STATE_N-9> zeros3n=Eigen::Matrix<double,3,STATE_N-9>::Zero();
	double dts=std::chrono::duration_cast<std::chrono::duration<double>>(this->dt).count();

	linSubmatrix<<
		   id3, id3*dts, id3*0.5*dts*dts, zeros3n,
		zeros3,     id3,         id3*dts, zeros3n,
		zeros3,  zeros3,             id3, zeros3n;

	Eigen::Matrix<double,7,STATE_N> rotSubmatrix;
	const Eigen::Matrix<double,4,STATE_N-7> zeros4m=Eigen::Matrix<double,4,STATE_N-7>::Zero();
	const Eigen::Matrix<double,3,STATE_N-7> zeros3m=Eigen::Matrix<double,3,STATE_N-7>::Zero();
	const Eigen::Matrix<double,3,4> zeros34=Eigen::Matrix<double,3,4>::Zero();
	const Eigen::Matrix4d id4=Eigen::Matrix4d::Identity();

	Eigen::Vector3d& rv=this->rotvel;
	Eigen::Matrix4d Omega;
	Omega.row(0)<<0,-rv.transpose();
	Omega.col(0)<<0,rv;
	Omega.bottomRightCorner<3,3>()<<-skewSym(rv);

	Eigen::Matrix<double,4,3> Quat;
	double q0=this->ori.w();
	Eigen::Vector3d qv=this->ori.vec();

	Quat<<-qv.transpose(),q0*id3+skewSym(qv);

	rotSubmatrix<<
		zeros4m, id4+0.25*dts*Omega, 0.25*dts*Quat,
		zeros3m,            zeros34,           id3;

	this->Fk<<linSubmatrix,rotSubmatrix;
	//std::cout<<this->Fk<<std::endl;

}

template<typename T> struct TD;

Eigen::Matrix<double,3,4> State::Jquatrotate(const Eigen::Quaterniond& q,const Eigen::Vector3d& v){

	Eigen::Quaterniond vq={0,v.x(),v.y(),v.z()};
	const Eigen::Matrix<Eigen::Quaterniond,1,4> qbasis={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	const Eigen::Matrix<Eigen::Quaterniond,1,4> qbasisinv=qbasis.unaryExpr([](auto&i){return i.conjugate();});

	Eigen::Matrix<Eigen::Quaterniond,1,4> aq1=(qbasis*vq*q.inverse());
	Eigen::Matrix<Eigen::Quaterniond,1,4> aq2=(q*vq*qbasisinv);
	Eigen::Quaterniond am=q*vq*q.inverse();
	Eigen::Vector4d qcoef=q.coeffs();

	Eigen::Matrix<double,3,4> J=Eigen::Matrix<double,3,4>::Zero();
	for(int i=0;i<4;++i){
		J.col(i)=aq1[i].vec()+(aq2[i].vec()-2*am.vec()*qcoef[i])/(std::pow(q.norm(),2));
	}

	return J;

}

Eigen::Matrix<double,3,STATE_N> State::JaccelSens(){

	Eigen::Matrix3d zeros3=Eigen::Matrix3d::Zero();
	Eigen::Matrix3d& Jax=zeros3;

	const Eigen::Quaterniond& q=this->ori;
	Eigen::Matrix3d Rq=q.toRotationMatrix();
	Eigen::Matrix3d& Jaa=Rq;

	Eigen::Matrix3d Jav=Rq*skewSym(this->rotvel);

	Eigen::Vector3d v=this->accelState+this->rotvel.cross(this->vel)+Eigen::Vector3d(0,0,-9.81);
	Eigen::Matrix<double,3,4> Jaq=this->Jquatrotate(q,v);

	Eigen::Matrix3d Jaw=-Rq*skewSym(this->vel);

	Eigen::Matrix<double,3,STATE_N> Ja;
	Ja<<Jax,Jav,Jaa,Jaq,Jaw;
	return Ja;
}



Eigen::Matrix<double,3,STATE_N> State::JrotvelSens(){
	Eigen::Matrix3d zeros3=Eigen::Matrix3d::Zero();
	Eigen::Matrix3d& Jwx=zeros3;
	Eigen::Matrix3d& Jwv=zeros3;
	Eigen::Matrix3d& Jwa=zeros3;

	const Eigen::Quaterniond& q=this->ori;
	Eigen::Matrix<double,3,4> Jwq=this->Jquatrotate(q,this->rotvel);

	Eigen::Matrix3d Jww=q.toRotationMatrix();

	Eigen::Matrix<double,3,STATE_N> Jw;
	Jw<<Jwx,Jwv,Jwa,Jwq,Jww;
	return Jw;

}

void State::calcHk(){

	Eigen::Matrix<double,3,STATE_N> accelSensMat=this->JaccelSens();
	Eigen::Matrix<double,3,STATE_N> rotvelSensMat=this->JrotvelSens();

	this->Hk<<accelSensMat,rotvelSensMat;



}

void State::updateT(){
	std::chrono::high_resolution_clock::time_point now=std::chrono::high_resolution_clock::now();
	this->dt=now-this->tprev;
	this->tprev=now;
}

void State::assemblezk(){
	this->zk.segment<3>(0)=this->accelSens;
	this->zk.segment<3>(3)=this->gyro;
	

}

void State::assemblexk(){
	this->xk.segment<3>(0)=this->pos;
	this->xk.segment<3>(3)=this->vel;
	this->xk.segment<3>(6)=this->accelState;
	this->xk(9)=this->ori.w();
	this->xk.segment<3>(10)=this->ori.vec();
	this->xk.segment<3>(13)=this->rotvel;

}
void State::disassemblexk(){
	this->pos=this->xk.segment<3>(0);
	this->vel=this->xk.segment<3>(3);
	this->accelState=this->xk.segment<3>(6);
	this->ori.w()=this->xk(9);
	this->ori.vec()=this->xk.segment<3>(10);
	this->ori.normalize();
	this->rotvel=this->xk.segment<3>(13);
}

void State::predict(){
	this->calcFk();

	this->xk=this->Fk*this->xk;
	this->Pk=this->Fk*this->Pk*this->Fk.transpose();
}

void State::update(){
	this->calcHk();

	this->yk=this->zk-this->Hk*this->xk;
	this->Sk=this->Rk+this->Hk*this->Pk*this->Hk.transpose();
	this->Kk=this->Pk*this->Hk.transpose()*this->Sk.inverse();
	this->xk=this->xk+this->Kk*this->yk;
	this->Pk=(Eigen::Matrix<double,STATE_N,STATE_N>::Identity()-this->Kk*this->Hk)*this->Pk;
}

void State::process(){

}
