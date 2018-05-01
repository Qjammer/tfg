#pragma once
#include"Socket.hpp"

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
	MOD_TYPE mt;
	SrvSocket srvs;
	std::vector<CliSocket> clis;
	Module(MOD_TYPE mt,const std::string& srvaddr):mt(mt),srvs(srvaddr){}
};
