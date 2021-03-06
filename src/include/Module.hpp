#pragma once
#include"Socket.hpp"

inline Eigen::Vector3d rotQuat(const Eigen::Vector3d& v,const Eigen::Quaterniond& q){
	Eigen::Quaterniond p(0,0,0,0);
	p.vec()=v;
	return (q*p*q.inverse()).vec();
}

enum MOD_TYPE:int{
	EXTCOM,
	ENVREC,
	PATHF,
	CONTR,
	STATE,
	SUPER,
	ERR
};

template<int i>
static const constexpr char* modStr(){
	return
	i==MOD_TYPE::EXTCOM?"ex":
	i==MOD_TYPE::ENVREC?"ev":
	i==MOD_TYPE::PATHF?"pf":
	i==MOD_TYPE::CONTR?"cr":
	i==MOD_TYPE::STATE?"st":
	i==MOD_TYPE::SUPER?"su":
	"ER";
}

#define makeMesVar(type,name,num) type name=std::get<type>(mv.vars[num].second);

template<int I,typename T,typename...V>
struct varCondRec {
	bool operator()(const varmes& mv){
		return mv.vars[I].first==typeChar<T>()&&varCondRec<I+1,V...>()(mv);
	}
};

template<int I,typename T>
struct varCondRec<I,T>{
	bool operator()(const varmes& mv){
		return mv.vars[I].first==typeChar<T>();
	}
};

template<typename...V>
bool varCond(const varmes& mv){
return (mv.vars.size()==sizeof...(V))&&varCondRec<0,V...>()(mv);
}

class Module{
public:
	bool active;
	MOD_TYPE mt;
	SrvSocket srvs;
	std::vector<CliSocket> clis;
	Module(MOD_TYPE mt,const std::string& srvaddr):active(true),mt(mt),srvs(srvaddr){}

	virtual void handleInComms(){
		for(auto cli:this->clis){
			std::vector<std::string> msgs=cli.receive();
			for(auto msg:msgs){
				varmes vm=cli.processMessage(msg);
				this->handleVarMessage(vm);
			}
		}
	}

	virtual void handleVarMessage(const varmes& mv)=0;
	virtual void process()=0;
	virtual void handleOutComms()=0;
	virtual void doAll(){
		this->handleInComms();
		this->process();
		this->handleOutComms();
	}
	virtual void loop(){
		while(this->active){
			this->doAll();
			usleep(500000);
		}
	}
};
