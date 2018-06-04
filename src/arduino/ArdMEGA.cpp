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
  M[0].SetSpeed(25);
  M[1].SetSpeed(25);
  M[2].SetSpeed(-25);
  M[3].SetSpeed(-25);

}

void loop() {
	const float R=0.06;
	const float F=2*PI*R/60;
	float d0=-M[0].realspeed*F;
	float d1=-M[1].realspeed*F;
	float d2=-M[2].realspeed*F;
	float d3=-M[3].realspeed*F;
	std::string msgtacho=ph.prepareMessage("ar","ta",d0,d1,d2,d3);
  Serial.print(msgtacho);
  /*
  Serial.println("ar");
  Serial.print(M[0].speed);
  Serial.print(" ");
  Serial.print(M[1].speed);
  Serial.print(" ");
  Serial.print(M[2].speed);
  Serial.print(" ");
  Serial.println(M[3].speed);
  */
  delay(20);

}

void ENCODER() {
  M[0].ENCODER();
  M[1].ENCODER();
  M[2].ENCODER();
  M[3].ENCODER();
}


