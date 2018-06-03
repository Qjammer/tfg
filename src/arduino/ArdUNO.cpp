#include<Arduino.h>
#include<Servo.h>
#include<Wire.h>
#include<LIDARLite.h>
#include"prepMes.hpp"

protocolHandler ph;
LIDARLite lidar;
Servo s1;
float phis,phireal;//Horizontal axis
int pinPhi=5;
const float phimin=60;
const float phimax=100;
Servo s2;
float thetas,thetareal;//Vertical axis
int pinTheta=6;
const float thetamin=70;
const float thetamax=130;
float d;

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t AcXBias=2912,AcYBias=732,AcZBias=-538,GyXBias=-413,GyYBias=-463,GyZBias=63;
float acx,acy,acz;
float gyx,gyy,gyz;

void setup() {
	Serial.begin(115200,SERIAL_8N1);
	//Servos
	s1.attach(pinPhi);
	s2.attach(pinTheta);
	phis=90;
	s1.write(phis);
	thetas=90;
	s1.write(thetas);
	//LIDAR
	lidar.begin(0, true); // default, I2c @ 400 kHz
	lidar.configure(0); // default
	//Sensors
	Wire.begin();
	//Power modes
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6B);
	Wire.write(0);
	Wire.endTransmission(true);
	//Axis standby
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6C);
	Wire.write(0x00);
	Wire.endTransmission(true);
	//Gyro precision
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x1B);
	Wire.write(0x00);
	Wire.endTransmission(true);
	//Accelerometer precision
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x1C);
	Wire.write(0x00);
	Wire.endTransmission(true);
}

double i=0;

void nextPos(){
	phis+=1;
	if(phis>phimax){// Go from up to down
		phis=phimin;
		s2.write(phis);
		thetas+=1;
		s1.write(thetas);
		delay(350);
	}
	s2.write(phis);
	delay(15);
	
	phireal=0+phis*130/140-90;
	phireal=phireal*PI/180;
	if(thetas>thetamax){//go from left to right
		thetas=thetamin;
		s1.write(thetas);
		delay(350);
	}
	thetareal=100+(thetas-100)*(180-90)/(162-100)-90;
	thetareal=thetareal*PI/180;
}

void loop() {
	//Move LIDAR
	nextPos();
	
	//Read sensors
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B); 
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr,14,true);
	//Register read
	AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

	const float fac=9.806/16384;
	acx=(AcX-AcXBias)*fac;
	acy=(AcY-AcYBias)*fac;
	acz=(AcZ-AcZBias)*fac;
	std::string msgaccel=ph.prepareMessage("ar","ac",acx,acy,acz);
	Serial.print(msgaccel);

	const float fgy=PI/(180*131);
	gyx=(GyX-GyXBias)*fgy;
	gyy=(GyY-GyYBias)*fgy;
	gyz=(GyZ-GyZBias)*fgy;
	std::string msggyro=ph.prepareMessage("ar","gy",gyx,gyy,gyz);
	Serial.print(msggyro);
	
	//Read LIDAR
	//delay(50);
	d = lidar.distance()*0.01;
	std::string msglidar=ph.prepareMessage("ar","li",phireal,thetareal,d);
	Serial.print(msglidar);
}
