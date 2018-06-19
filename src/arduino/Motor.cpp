#include "Motor.h"
#include <PID/PID_v1.h>

//Constructor
Motor::Motor (int P, int A, int B, int E1, int E2, double KP, double KI, double KD, double C2) : pid(&speed, &POWER, &setpoint, KP, KI,KD, DIRECT)
{
  PWM = P;
  PIN_A = A;
  PIN_B = B;
  ENC1 = E1;
  ENC2 = E2;
  
  if (C2>0) SetForward();
  else SetBackward();
  setpoint = fabs(C2);
  
  //pid.SetOutputLimits(60, 255);
  pid.SetOutputLimits(0, 255);
  pid.SetSampleTime(38);
  pid.SetMode(AUTOMATIC);
}

void Motor::SetForward() { //Marxa envant
  digitalWrite(PIN_A, 1);
  digitalWrite(PIN_B, 0);
  F=1;
}

void Motor::SetBackward() { //Marxa endarrere
  digitalWrite(PIN_A, 0);
  digitalWrite(PIN_B, 1);
  F=0;
}

void Motor::SetSpeed(double w) { //Defineix una consigna
  pid.SetMode(AUTOMATIC);
  setpoint = w;
}


void Motor::SetDuty(int w) { //Defineix un cicle de treball
  if (w > -256 && w < 256) {
    pid.SetMode(MANUAL);
    if (w>=0) SetForward();
    else SetBackward();
    analogWrite(PWM, abs(w));
  }
}



void Motor::ENCODER() {
  S = (digitalRead(ENC1)^digitalRead(ENC2));
  if (S != S_ant) {
    count++;
    S_ant = S;
  }
  n++;
  READ();
}

void Motor::READ() {
  if (n > 125) { //40 lectures per segon. Si augmentam les lectures, disminuex la precisiÃ³ a baixes RMP'S
    omega = (C / (float(n))) * float(count);
    
    if(F) turns+=float(count)/1721.6; //Si va cap envant, voltes positives
    else turns-=float(count)/1721.6;
    
    count = 0; //Comptador igual a zero
    n = 0;

    speed = int(digitalSmooth(omega*(2*T-1), Array1, i, sorted)); 
    
    pid.Compute();
    
    if(pid.GetMode()){//Només si el PID està en AUTOMATIC
		if(POWER>0){
			SetForward();
		} else {
			SetBackward();
		}
		analogWrite(PWM, POWER);
	} 
    
    if(speed<=10 && speed>=0){ //Quan la velocitat s'apropa a 0, miram si ha de canviar el sentit de gir o no
    	if(F) T=1; //F és el sentit de gir "consigna", T és el sentit de gir real
    	else  T=0;
	}
	//realspeed=speed*(2*int(T)-1); //Velocitat real = velocitat * sentit de gir real (que no és F, sinó T).
	realspeed=speed;
  }
}

double Motor::digitalSmooth(double rawIn, double *sensSmoothArray, int &i, double *sorted) {    // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  const int filterSamples = 10;
  double temp, top, bottom;
  int j, k;
  double total;
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  for (j = 0; j < filterSamples; j++) { // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting
  while (done != 1) {      // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++) {
      if (sorted[j] > sorted[j + 1]) {    // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j + 1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1);
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j < top; j++) {
    total += sorted[j];  // total remaining indices
    k++;
  }
  return total / double(k);    // divide by number of samples
}

