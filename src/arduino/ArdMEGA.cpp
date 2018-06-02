#include <TimerOne.h>
#include <Motor.h>

Motor M[4] = {
  Motor(4, 30, 31, 28, 29, 1.5, 8, 0.4, 0),
  Motor(5, 34, 35, 32, 33, 1.5, 8, 0.4, 0),
  Motor(6, 38, 39, 36, 37, 1.5, 8, 0.4, 0),
  Motor(7, 42, 43, 40, 41, 1.5, 8, 0.4, 0),
};


void ENCODER();
void setup() {
  Serial.begin(9600);
  Timer1.initialize (300);
  Timer1.attachInterrupt(ENCODER);
  M[0].SetSpeed(25);
  M[1].SetSpeed(25);
  M[2].SetSpeed(25);
  M[3].SetSpeed(25);

}

void loop() {
  Serial.print(M[0].speed);
  Serial.print(" ");
  Serial.print(M[1].speed);
  Serial.print(" ");
  Serial.print(M[2].speed);
  Serial.print(" ");
  Serial.println(M[3].speed);

}

void ENCODER() {
  M[0].ENCODER();
  M[1].ENCODER();
  M[2].ENCODER();
  M[3].ENCODER();
}


