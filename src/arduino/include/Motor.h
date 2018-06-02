#pragma once
#include <Arduino.h>
#include <PID/PID_v1.h>
class Motor {
  public:

    // DEFINICIÓ DE PINS PWM DELS MOTORS
    int PWM;

    // DEFINICIÓ DELS PINS LÒGICS DELS MOTORS
    int PIN_A, PIN_B, ENC1, ENC2;

    //Constants PID
    double setpoint; //Consigna de RPM donada pel LIDAR


    //Variables Encoder
    volatile int n = 0; //Comptador auxiliar
    volatile int count = 0; //Comptador encoder
    volatile bool S = 0;
    volatile bool S_ant = 0;
    volatile bool F=0; //True si ens movem cap envant
    volatile bool T=0; //True si la velocitat real del motor �s +
    volatile float omega = 0; //Velocitat angular del motor (RPM)
    volatile float turns=0; //Voltes del motor 
    double speed= 0; //Velocitat motor amb filtre passabaixos
    volatile double realspeed=0; //Velocitat motor, amb signe

    //Constants PID
    double POWER = 0;

    PID pid;

    Motor (int P, int A, int B, int E1, int E2, double KP, double KI, double KD, double C2);
    void ENCODER();
    void SetForward();
    void SetBackward();
    void SetDuty(int w);
    void SetSpeed(double w);
    void READ();
    double digitalSmooth(double rawIn, double *sensSmoothArray, int &i, double *sorted);

  private:
  	
  	double Array1 [10]; //array
    double sorted[10]; //array sorted
    int i=0; // array counter
    
    const int t = 300; //Temps interrupció lectura 0.3 ms
    const float t_s = (float(t) / 1000000); //Temps en segons
    const float C = 0.03485 / t_s; //Constant del motor

};

