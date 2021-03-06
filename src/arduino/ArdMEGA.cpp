#include <TimerOne.h>
#include <Motor.h>
#include<prepMes.hpp>

Motor M[4] = {
  Motor(4, 30, 31, 28, 29, 1.5, 8, 0.4, 0),
  Motor(5, 34, 35, 32, 33, 1.5, 8, 0.4, 0),
  Motor(6, 38, 39, 36, 37, 1.5, 8, 0.4, 0),
  Motor(7, 42, 43, 40, 41, 1.5, 8, 0.4, 0),
};


protocolHandler ph;
void ENCODER();
void setup() {
  Serial.begin(115200);
  Timer1.initialize (300);
  Timer1.attachInterrupt(ENCODER);
  float lspeed=40;
  float rspeed=40;
  M[0].SetSpeed(lspeed);
  M[1].SetSpeed(lspeed);
  M[2].SetSpeed(-rspeed);
  M[3].SetSpeed(-rspeed);
}

void processMes(std::string&m){
	int i0=m.indexOf(" ");
	if(i0!=-1){
		int i1=m.indexOf(" ",i0+1);
		if(i1!=-1){
			int i2=m.indexOf(" ",i1+1);
			if(i2!=-1){
				int i3=m.indexOf(" ",i2+1);
				if(i3!=-1){
					int i4=m.indexOf(" ",i3+1);
					if(i4!=-1){
						//Correct
						const float R=0.06;
						const float F=2*PI*R/60;
						float w0=m.substring(i0,i1).toFloat();
						float w1=m.substring(i1,i2).toFloat();
						float w2=m.substring(i2,i3).toFloat();
						float w3=m.substring(i3,i4).toFloat();
						M[0].SetSpeed(w0/F);
						M[1].SetSpeed(w1/F);
						M[2].SetSpeed(-w2/F);
						M[3].SetSpeed(-w3/F);
						Serial.print("calculated w: ");
						Serial.print(w0);
						Serial.print(" ");
						Serial.print(w1);
						Serial.print(" ");
						Serial.print(w2);
						Serial.print(" ");
						Serial.print(w3);
						Serial.println();
					} // else ignore message
				}
			}
		}
	}
}

std::string partMes;
void processPartMes(){
	int begin=partMes.indexOf('W');
	if(begin==-1){
		//no beginning found
		return;
	} else {
		int end=partMes.indexOf('E');
		if(end==-1){
			//no end found
			return;
		} else {
			if(end<begin){
				partMes.remove(0,end+1);
				return;
			}
			std::string mes=partMes.substring(begin,end+1);
			partMes.remove(0,end+1);
			processMes(mes);
		}
	}
}

void loop() {
	//Sensor
	const float R=0.08;
	const float F=2*PI*R/60;
	float v0=M[0].realspeed;
	float v1=M[1].realspeed;
	float v2=M[2].realspeed;
	float v3=M[3].realspeed;
	float d0=M[0].realspeed*F;
	float d1=M[1].realspeed*F;
	float d2=-M[2].realspeed*F;
	float d3=-M[3].realspeed*F;
	std::string msgtacho=ph.prepareMessage("ar","ta",d0,d1,d2,d3);
	Serial.print(msgtacho);
	/*
	Serial.print(v0);
	Serial.print(" ");
	Serial.print(v1);
	Serial.print(" ");
	Serial.print(v2);
	Serial.print(" ");
	Serial.println(v3);
	*/
	
	//Actuator
	while(Serial.available()>0){
		//partMes+=char(Serial.read());
		Serial.read();
	}

	//processPartMes();
	float lspeed=50;
	float rspeed=30;
	M[0].SetSpeed(lspeed);
	M[1].SetSpeed(lspeed);
	M[2].SetSpeed(-rspeed);
	M[3].SetSpeed(-rspeed);
}

void ENCODER() {
  M[0].ENCODER();
  M[1].ENCODER();
  M[2].ENCODER();
  M[3].ENCODER();
}

