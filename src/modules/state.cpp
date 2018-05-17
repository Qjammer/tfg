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

Eigen::Matrix<double,3,STATE_N> State::Jx(){
	double dts=this->dts;
	auto id3=Eigen::Matrix3d::Identity();
	Eigen::Matrix<double,3,STATE_N> Jx;
	Jx<<id3,dts*id3,0.5*dts*dts*id3,Eigen::Matrix<double,3,STATE_N-9>::Zero();
	return Jx;
}

Eigen::Matrix<double,3,STATE_N> State::Jv(){
	double dts=this->dts;
	auto id3=Eigen::Matrix3d::Identity();
	auto zeros3=Eigen::Matrix3d::Zero();

	Eigen::Matrix<double,3,STATE_N> Jv;
	Jv<<zeros3,id3,dts*id3,Eigen::Matrix<double,3,STATE_N-9>::Zero();
	return Jv;
}

Eigen::Matrix<double,3,STATE_N> State::Ja(){
	auto id3=Eigen::Matrix3d::Identity();
	auto zeros3=Eigen::Matrix3d::Zero();

	Eigen::Matrix<double,3,STATE_N> Ja;
	Ja<<zeros3,zeros3,id3,Eigen::Matrix<double,3,STATE_N-9>::Zero();
	return Ja;
}

Eigen::Matrix<double,9,STATE_N> State::linFk(){
	Eigen::Matrix<double,9,STATE_N> linSubmatrix;
	linSubmatrix<<
		this->Jx(),
		this->Jv(),
		this->Ja();
	return linSubmatrix;
}

Eigen::Matrix<double,4,STATE_N> State::Jq(){
	double dts=this->dts;

	const Eigen::Matrix<double,4,STATE_N-7> zeros4m=Eigen::Matrix<double,4,STATE_N-7>::Zero();

	const Eigen::Quaterniond& q=this->ori;
	const Eigen::Vector3d& w=this->rotvel;
	Eigen::Quaterniond wqdt;
	wqdt.w()=1-std::pow(this->rotvel.norm()*dts/2,2)/2;
	wqdt.vec()=-this->rotvel*dts/2;
	
	const Eigen::Matrix<Eigen::Quaterniond,1,4> qbasis={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	Eigen::Matrix<Eigen::Quaterniond,1,4> Jqqq;
	Jqqq=qbasis*wqdt;
	Eigen::Matrix<double,4,4> Jqq;

	for(int i=0;i<4;i++){
		Jqq.col(i)<<Jqqq[i].w(),Jqqq[i].vec();
	}
	
	Eigen::Matrix<double,4,3> Jqw;
	for(int i=0;i<3;i++){
		Eigen::Quaterniond dwq;
		dwq.w()=-w[i]*dts*dts/4;
		dwq.vec()=Eigen::Vector3d::Zero();
		dwq.vec()[i]=-dts/2;
		Eigen::Quaterniond rs=dwq*q;
		Jqw.col(i)<<rs.w(),rs.vec();
	}

	Eigen::Matrix<double,4,STATE_N> Jq;
	Jq<<zeros4m, Jqq, Jqw;
	return Jq;

}

Eigen::Matrix<double,3,STATE_N> State::Jw(){
	auto id3=Eigen::Matrix3d::Identity();
	auto zeros34=Eigen::Matrix<double,3,4>::Zero();

	Eigen::Matrix<double,3,STATE_N> Jw;
	Jw<<Eigen::Matrix<double,3,STATE_N-7>::Zero(), zeros34, id3;
	return Jw;
}

Eigen::Matrix<double,7,STATE_N> State::rotFk(){
	Eigen::Matrix<double,7,STATE_N> Jrot;
	Jrot<<
		this->Jq(),
		this->Jw();
	return Jrot;
}

void State::calcFk(){
	Eigen::Matrix<double,9,STATE_N> linFk=this->linFk();
	Eigen::Matrix<double,7,STATE_N> rotFk=this->rotFk();
	this->Fk<<linFk,rotFk;

}

template<typename T> struct TD;

Eigen::Matrix<double,3,4> State::Jquatrotate(const Eigen::Quaterniond& q,const Eigen::Vector3d& v){

	Eigen::Quaterniond vq={0,v.x(),v.y(),v.z()};
	const Eigen::Matrix<Eigen::Quaterniond,1,4> qbasis={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	const Eigen::Matrix<Eigen::Quaterniond,1,4> qbasisinv=qbasis.unaryExpr([](auto&i){return i.conjugate();});

	Eigen::Matrix<Eigen::Quaterniond,1,4> aq1=qbasis*vq*q.inverse();
	Eigen::Matrix<Eigen::Quaterniond,1,4> aq2=q*vq*qbasisinv;
	Eigen::Quaterniond am=q*vq*q.inverse();
	//Eigen::Vector4d qcoef=q.coeffs();//Wrong way around: x,y,z,w
	Eigen::Vector4d qcoef(q.w(),q.vec().x(),q.vec().y(),q.vec().z());

	Eigen::Matrix<double,3,4> J=Eigen::Matrix<double,3,4>::Zero();
	for(int i=0;i<4;++i){
		J.col(i)=aq1[i].vec()+(aq2[i].vec()-2*am.vec()*qcoef[i])/(std::pow(q.norm(),2));
	}
	return J;

}

Eigen::Matrix<double,3,STATE_N> State::JaccelSens(){
	Eigen::Matrix3d zeros3=Eigen::Matrix3d::Zero();
	Eigen::Matrix3d& Jax=zeros3;
	Eigen::Matrix3d& Jav=zeros3;
	Eigen::Matrix3d& Jaw=zeros3;

	const Eigen::Quaterniond& q=this->ori;
	Eigen::Matrix3d Jaa=q.toRotationMatrix();

	Eigen::Vector3d v=this->accelState+Eigen::Vector3d(0,0,9.81);
	Eigen::Matrix<double,3,4> Jaq=this->Jquatrotate(q,v);

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
	this->dts=std::chrono::duration_cast<std::chrono::duration<double>>(this->dt).count();
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

Eigen::Matrix<double,STATE_N,1> State::expectedxk(){
	double dts=this->dts;
	Eigen::Vector3d xkx=this->pos+dts*this->vel+0.5*dts*dts*this->accelState;
	Eigen::Vector3d xkv=this->vel+dts*this->accelState;
	Eigen::Vector3d xka=this->accelState;

	Eigen::Quaterniond wqdt;
	wqdt.w()=1-std::pow(this->rotvel.norm()*dts/2,2)/2;
	wqdt.vec()=-this->rotvel*dts/2;
	Eigen::Quaterniond xkq=wqdt*this->ori;
	xkq.normalize();

	Eigen::Vector3d xkw=this->rotvel;

	Eigen::Matrix<double,STATE_N,1> xkexp;
	xkexp<<xkx,xkv,xka,xkq.w(),xkq.vec(),xkw;
	return xkexp;
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

Eigen::Matrix<double,SENSOR_N,1> State::expectedzk(){
	Eigen::Quaterniond& q=this->ori;
//	Eigen::Vector3d f=this->accelState+this->rotvel.cross(this->vel)+Eigen::Vector3d(0,0,9.81);
	Eigen::Vector3d f=this->accelState+Eigen::Vector3d(0,0,9.81);
	Eigen::Quaterniond fq;
	fq.w()=0;fq.vec()=f;
	Eigen::Matrix<double,3,1> amexp=(q*fq*q.inverse()).vec();

	Eigen::Quaterniond wq;
	wq.w()=0;wq.vec()=this->rotvel;
	Eigen::Matrix<double,3,1> wmexp=(q*wq*q.inverse()).vec();

	Eigen::Matrix<double,SENSOR_N,1> zkexp;
	zkexp<<amexp,wmexp;
	return zkexp;
}

void State::predict(){
	this->calcFk();//TODO:Check if Fk has to be calculated at x=k-1 or x=k

	this->xk=this->expectedxk();
	this->Pk=(this->Fk*this->Pk*this->Fk.transpose()).eval();
}

void State::update(){
	this->calcHk();

	auto yk=this->zk-this->expectedzk();
	auto Sk=this->Rk+this->Hk*this->Pk*this->Hk.transpose();
	this->Kk=this->Pk*this->Hk.transpose()*Sk.inverse();

	this->xk=this->xk+this->Kk*yk;
	auto IKH=Eigen::Matrix<double,STATE_N,STATE_N>::Identity()-this->Kk*this->Hk;
	this->Pk=IKH*this->Pk*IKH.transpose()+this->Kk*this->Rk*this->Kk.transpose();
}

void State::process(){
	this->updateT();

	this->assemblexk();
	this->predict();
	this->disassemblexk();

	this->assemblexk();
	this->assemblezk();
	this->update();
	this->disassemblexk();

}

