#include<Arduino.h>
#include"prepMes.hpp"
//#include"simpleVariant.hpp"

int x=A0;
int y=A1;
int z=A2;

protocolHandler ph;
void setup() {
	Serial.begin(115200,SERIAL_8N1);
	pinMode(A0,INPUT);
	pinMode(A1,INPUT);
	pinMode(A2,INPUT);
}
char i=-5;
void loop() {
	//while(Serial.available()>0){
	//	i=Serial.read();
	//	Serial.print(i);
		
	//}
	int xi=analogRead(x);
	int yi=analogRead(y);
	int zi=analogRead(z);
	float xf=float(xi-512)*2/512;
	float yf=float(yi-512)*2/512;
	float zf=float(zi-512)*2/512;
	//String msg=prepareMessage("ar","ac",int32_t(xv),int32_t(yv),int32_t(zv));
	std::string msg=ph.prepareMessage("ar","ac",xf,yf,zf);
	Serial.print(msg);
	//Serial.print("\n");
	//delay(1);
}
