#include <PID_v1.h>
#include <NewPing.h>
#define TRIGGER_PIN  10 
#define ECHO_PIN     9 
#define PWM_PIN  5
#define MAX_DISTANCE 100 
NewPing sonar1(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
float t1 = 0; 
float t2 = 0; 
float distance  = 0;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
//double aggKp=4, aggKi=0.2, aggKd=1;
//double consKp=1, consKi=0.05, consKd=0.25;
//double consKp=2, consKi=0.01, consKd=1.6;
//double consKp=2, consKi=0.001, consKd=0.08;//
//double consKp=500, consKi=5, consKd=50000;//
double consKp=1.75, consKi=0.001, consKd=0.08;//

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(consKp, consKi, consKd);
  Setpoint = 90;
  myPID.SetOutputLimits(0.0,100);
}

void loop()
{
  distance =sonar1.ping_cm();
  delay(1);
  Input = MAX_DISTANCE - distance;
  myPID.Compute();
  t1 = Output ;
  t2 = MAX_DISTANCE - t1;
  digitalWrite(PWM_PIN, HIGH); 
  delay(t1);   
  digitalWrite(PWM_PIN, LOW);  
  delay(t2);
  Serial.println(Output);

}
