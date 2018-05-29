#pragma once
#if __cplusplus>=201703L
#include<variant>
#endif

#define ALLOWED_TYPES void*,bool,char,int,long,float,double
template<typename ...>
struct is_one_of{static constexpr bool value=false;};

template<typename F,typename S, typename...T>
struct is_one_of<F,S,T...>{
	static constexpr bool value=std::is_same<F,S>::value||is_one_of<F,T...>::value;
};

template<typename T>
static constexpr char typeChar(){
	static_assert(is_one_of<T,ALLOWED_TYPES>::value,"Type not supported");
	return
	std::is_same<T,void*>::value?'a':
	std::is_same<T,bool>::value?'b':
	std::is_same<T,char>::value?'c':
	std::is_same<T,int>::value?'d':
	std::is_same<T,long>::value?'e':
	std::is_same<T,float>::value?'f':
	std::is_same<T,double>::value?'g':
	'z';
	}

#ifdef _GLIBCXX_VARIANT
typedef std::pair<char,std::variant<ALLOWED_TYPES>> myvari;
typedef std::vector<myvari> vecvar;

struct varmes{
	std::string sender;
	std::string purpose;
	int32_t size;
	vecvar vars;
};
#endif


class protocolHandler{
public:
	template<typename... V>
	std::string prepareMessage(const char sender[2], const char type[2],V... v) const{
		std::string ss;
		ss+=sender[0];
		ss+=sender[1];

		ss+=type[0];
		ss+=type[1];

		std::string tmp;
		this->prepareMessageRec(tmp,v...);
		int32_t i=tmp.size();
		char* p=reinterpret_cast<char*>(&i);
		for(unsigned int i=0;i<sizeof(int32_t);++i){
			ss+=p[i];
		}
		ss+=tmp;
		return ss;
	}

	template<typename T,typename... V>
	int prepareMessageRec(std::string& ss,T t,V...v)const{
		return prepareMessageType(ss,t)+prepareMessageRec(ss,v...);
	}

	template<typename T>
	int prepareMessageRec(std::string& ss,T t) const{
		return prepareMessageType(ss,t);
	}

	template<typename T>
	int prepareMessageTypeGen(std::string& ss, T& t,char c)const{
		ss+=c;
		char* cp=reinterpret_cast<char*>(&t);
		for(unsigned int i=0;i<sizeof(T);++i){
			ss+=cp[i];
		}
		return 1+sizeof(T);
	}

	template <typename T>
	int prepareMessageType(std::string& ss, T t) const{
		return prepareMessageTypeGen(ss,t,typeChar<T>());
	
	}

	template<typename T>
	T processType(const std::string& msg,unsigned int& pos){
		char ar[sizeof(T)];
		pos++;
		for(unsigned int i=0;i<sizeof(T);++i){
		ar[i]=msg[pos+i];
		}
		pos+=sizeof(T);
		T* t = reinterpret_cast<T*>(ar);
		return *t;
	}
#ifdef _GLIBCXX_VARIANT
	varmes processMessage(const std::string& msg){
		varmes vm;
		vm.sender=std::string(msg.begin(),msg.begin()+2);
		vm.purpose=std::string(msg.begin()+2,msg.begin()+4);
		const char*c=msg.c_str()+4;
		const int32_t* sp=reinterpret_cast<const int32_t*>(c);
		vm.size=*sp;
		//std::cout<<vm.size<<std::endl;
		//for(auto i:msg){std::cout<<uint16_t(i)<<" ";}std::cout<<std::endl;

		vecvar& vec=vm.vars;
		unsigned int pos=4+sizeof(int32_t);//Ignore header info for now
		while(pos<vm.size+4+sizeof(int32_t)){
			//std::cout<<pos<<" "<<msg.size()<<" "<<vm.size<<std::endl;
			switch (msg[pos]){
		#define processTypeSwitch(U) case typeChar<U>():{vec.push_back(myvari(typeChar<U>(),processType<U>(msg,pos)));break;}
				processTypeSwitch(void*);
				processTypeSwitch(bool);
				processTypeSwitch(int);
				processTypeSwitch(char);
				processTypeSwitch(long);
				processTypeSwitch(float);
				processTypeSwitch(double);
				default:
					pos+=512000;//Something wrong happened. skip this message
					vec.clear();
					break;
			}
		}
		return vm;
	}
#endif
};

