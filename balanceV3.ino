

#include <PID_v1.h>
#include <Wire.h>
#include <SFE_MMA8452Q.h>   // Change address in MMA8452Q.h to Ox1C
#include <math.h>
#include <L3G.h>

#define INTENTIONAL_DEAD_ZONE 2
#define MINPWM 10
#define GYRO_PCT  (float)99
//PID
double setpoint = 180;
double PIDinput, output;
double Kp =15;   
double Kd = 0.8;
double Ki = 200;
//double Kp =28;   
//double Kd = 0.3;
//double Ki = 0.2;

PID pid(&PIDinput, &output, &setpoint, Kp, Ki, Kd, AUTOMATIC);
L3G gyro; 
MMA8452Q accel;/// Change adress to 0x1C in SFE_MMA8452Q.h file if not able to read
//float accel_offset = 0.5;
float accel_offset = 2.15;
void motors_off() {
  digitalWrite(3,LOW); 
  digitalWrite(6,LOW);   
  digitalWrite(5,LOW); 
  digitalWrite(9,LOW);
}
void setup() {
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  motors_off();
  Serial.begin(9600);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
 accel.init(); //2g
 gyro.enableDefault();
 pid.SetMode(AUTOMATIC);
 pid.SetSampleTime(10);
 pid.SetOutputLimits(-255, 255);
}

  

float ANGgyro(float Time,float angle){
 ////////////  Détermine la vitesse de rotation (axe x) du Gyroscope en DPS
 gyro.read();
 float GyroDPS = -gyro.g.x*14.286/1000;
 return(angle + GyroDPS*Time/1000);
}

float AngAcc(){
  static int i=0;
  ////////////  Détermine les valeurs trouvé par l'Accéléromètre (Angle)
  //if (i %10 ==0) {
    accel.read();
  //}
  float forceY = accel.cy;
  float forceZ = accel.cz;
  i++;
  return(accel_offset+(57.2958*atan2(forceY,forceZ)));
}

float CorrAng(unsigned long Time){
  static float angle = 180;
  angle = (((GYRO_PCT/100)*ANGgyro(Time,angle)) + ((100-GYRO_PCT)/100*AngAcc())); // 65% gyro 35% Accel

  //  float new_angle = map(angle, -180, 180, 0, 360);
  float new_angle = angle+180;
  return(new_angle);
}

void Moteurs(float pidOUT){
  if (pidOUT > INTENTIONAL_DEAD_ZONE){
    pidOUT = map(pidOUT,1,255,MINPWM,255);
    analogWrite(3,pidOUT);
    analogWrite(6,pidOUT);
    digitalWrite(5,LOW); 
    digitalWrite(9,LOW);   
  } else if (pidOUT < -INTENTIONAL_DEAD_ZONE){
    pidOUT = map(-pidOUT,1,255,MINPWM,255);
    Serial.println(pidOUT);
    analogWrite(5,pidOUT);
    analogWrite(9,pidOUT);
    digitalWrite(3,LOW); 
    digitalWrite(6,LOW); 
  } else {
    motors_off();
  }
}

unsigned long Time = 0; ///temps depuis démarrage
void loop() {
  unsigned long prevTime = Time;
  Time = millis();
  unsigned long dt = Time - prevTime;
//  Serial.println(CorrAng(dt));

  PIDinput = CorrAng(dt);
  pid.Compute();
  Moteurs(output);
  delay(1);
  //Serial.println(output);
}
