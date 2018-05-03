#pragma once
#include"Socket.hpp"

inline Eigen::Vector3d rotQuat(const Eigen::Vector3d& v,const Eigen::Quaterniond& q){
	Eigen::Quaterniond p(0,0,0,0);
	p.vec()=v;
	return (q*p*q.inverse()).vec();
}

enum MOD_TYPE:int{
	SENS,
	ENVREC,
	PATHF,
	CONTR,
	ACT,
	STATE,
	SUPER,
	ERR
};

template<int i>
static const constexpr char* modStr(){
	return
	i==MOD_TYPE::SENS?"sn":
	i==MOD_TYPE::ENVREC?"ev":
	i==MOD_TYPE::PATHF?"pf":
	i==MOD_TYPE::CONTR?"cr":
	i==MOD_TYPE::ACT?"at":
	i==MOD_TYPE::STATE?"st":
	i==MOD_TYPE::SUPER?"su":
	"ER";
	};

class Module{
public:
	bool active;
	MOD_TYPE mt;
	SrvSocket srvs;
	std::vector<CliSocket> clis;
	Module(MOD_TYPE mt,const std::string& srvaddr):mt(mt),srvs(srvaddr){}

	virtual void handleInComms(){
		for(auto cli:this->clis){
			std::vector<std::string> msgs=cli.receive();
			for(auto msg:msgs){
				std::cout<<msg<<std::endl;
				varmes vm=cli.processMessage(msg);
				this->handleVarMessage(vm);
			}
		}
	}

	virtual void handleVarMessage(varmes& mv)=0;
	virtual void process()=0;
	virtual void handleOutComms()=0;

	virtual void loop(){
		while(this->active){
			this->handleInComms();
			this->process();
			this->handleOutComms();
		}
	}
};
