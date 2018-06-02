#include<Arduino.h>
#include<Servo.h>
#include<Wire.h>
#include<LIDARLite.h>
#include"prepMes.hpp"

protocolHandler ph;
LIDARLite lidar;
Servo s1;
float phi;//Horizontal axis
int pinPhi=5;
const float phimin=70;
const float phimax=110;
Servo s2;
float theta;//Vertical axis
int pinTheta=6;
const float thetamin=70;
const float thetamax=110;
float d;

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float acx,acy,acz;
float gyx,gyy,gyz;

void setup() {
	Serial.begin(115200,SERIAL_8N1);
	//Servos
	s1.attach(pinPhi);
	s2.attach(pinTheta);
	phi=90;
	s1.write(phi);
	theta=90;
	s1.write(theta);
	//LIDAR
	lidar.begin(0, true); // default, I2c @ 400 kHz
	lidar.configure(0); // default
	//Sensors
	Wire.begin();
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6B);
	Wire.write(0);
	Wire.endTransmission(true);
}
double i=0;

void nextPos(){
	phi+=2;
	if(phi>phimax){
		phi=phimin;
		theta+=5;
	}
	s2.write(phi);
	if(theta>thetamax){
		theta=thetamin;
	}
	s1.write(theta);

}
void loop() {
	//Move LIDAR
	delay(50);
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

	Serial.print(AcX);
	Serial.print(" ");
	Serial.print(AcY);
	Serial.print(" ");
	Serial.println(AcZ);
	//std::string msgaccel=ph.prepareMessage("ar","ac",acx,acy,acz);
	//Serial.print(msgaccel);
	//std::string msggyro=ph.prepareMessage("ar","gy",gyx,gyy,gyz);
	//Serial.print(msggyro);
	
	//Read LIDAR
	d = lidar.distance();
	//std::string msglidar=ph.prepareMessage("ar","li",phi,theta,d);
	//Serial.print(msglidar);
	Serial.print(phi);
	Serial.print(" ");
	Serial.print(theta);
	Serial.print(" ");
	Serial.println(d);

}
