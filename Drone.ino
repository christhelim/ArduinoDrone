#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

int pot[4];
float throttle, yaw, roll, pitch;
float pLeft, pRight, pFront, pRear;
float pfr, pfl, prr, prl;
float desiredRoll = 0;
float desiredPitch = 0;
float pitchError, rollError, prevpitchError, prevrollError;

float rollPIDp = 0;
float rollPIDi = 0;
float rollPIDd = 0;
float pitchPIDp = 0;
float pitchPIDi = 0;
float pitchPIDd = 0;

double p = 3.55;
double i = 0.005;
double d = 2.05;
float pPID, rPID;

Servo fr;
Servo fl;
Servo rr;
Servo rl;

int16_t RawAx, RawAy, RawAz, RawGp, RawGr;
float Ap, Ar;
float Gp, Gr;

float elapsedTime, time, timePrev;

float degree = 180/3.1415;



void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  fr.attach(3);
  fl.attach(4);
  rr.attach(5);
  rl.attach(6);

  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  time = millis();
  fr.writeMicroseconds(1000);
  fl.writeMicroseconds(1000);
  rr.writeMicroseconds(1000);
  rl.writeMicroseconds(1000);
}



void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);

  RawAx = Wire.read()<<8|Wire.read();
  RawAy = Wire.read()<<8|Wire.read();
  RawAz = Wire.read()<<8|Wire.read();
  Ap = atan((RawAy/16384.0)/sqrt(pow((RawAx/16384.0),2) + pow((RawAz/16384.0),2)))*degree;
  Ar = atan(-1*(RawAx/16384.0)/sqrt(pow((RawAy/16384.0),2) + pow((RawAz/16384.0),2)))*degree;

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true);

  RawGp=Wire.read()<<8|Wire.read();
  RawGr=Wire.read()<<8|Wire.read();
  Gp = RawGp/131.0;
  Gr = RawGr/131.0;

  pitch = 0.98 *(pitch + Gp*elapsedTime) + 0.02*Ap;     //Pitch Axis
  roll = 0.98 *(roll + Gr*elapsedTime) + 0.02*Ar;       //Roll Axis

  pitchError = pitch - desiredPitch;
  rollError = roll - desiredRoll;



  rollPIDp = p*rollError;                                       //Proportional component
  if(-3 < rollError < 3){                                       //Integral component
    rollPIDi = rollPIDi + (i*rollError);
  }
  rollPIDd = d*((rollError - prevrollError) / elapsedTime);     //Derivative component
  rPID = rollPIDp + rollPIDi + rollPIDd;                         //PID

  if(rPID < -1000){                                              //Max. values
    rPID = -1000;
  }
  if(rPID > 1000){
    rPID = 1000;
  }

  pitchPIDp = p*pitchError;                                       //Proportional component
  if(-3 < pitchError < 3){                                       //Integral component
    pitchPIDi = pitchPIDi + (i*pitchError);
  }
  pitchPIDd = d*((pitchError - prevpitchError) / elapsedTime);     //Derivative component
  pPID = pitchPIDp + pitchPIDi + pitchPIDd;                         //PID

  if(pPID < -1000){                                              //Max. values
    pPID = -1000;
  }
  if(pPID > 1000){
    pPID = 1000;
  }

  pLeft = throttle - rPID;
  pRight = throttle + rPID;
  pFront = throttle - pPID;
  pRear = throttle + pPID;

  pfr = (pFront + pRight) / 2;
  pfl = (pFront + pLeft) / 2;
  prr = (pRear + pRight) / 2;
  prl = (pRear + pLeft) / 2;

  if(pfr < 1000){
    pfr = 1000;
  }
  if(pfr > 2000){
    pfr = 2000;
  }
  if(pfl < 1000){
    pfl = 1000;
  }
  if(pfl > 2000){
    pfl = 2000;
  }
  if(prr < 1000){
    prr = 1000;
  }
  if(prr > 2000){
    prr = 2000;
  }
  if(prl < 1000){
    prl = 1000;
  }
  if(prl > 2000){
    prl = 2000;
  }

  if(throttle < 1030){
    pfr = 1000;
    pfl = 1000;
    prr = 1000;
    prl = 1000;
  }


  Serial.print(throttle);
  Serial.print("   ");
  Serial.print(pfl);
  Serial.print("   ");
  Serial.print(pfr);
  Serial.print("   ");
  Serial.print(prl);
  Serial.print("   ");
  Serial.println(prr);
  delay(50);


  if (radio.available()) {
    radio.read(pot, sizeof(pot));
  }
  throttle = map(pot[0], 0, 1023, 1000, 2000);

  fr.writeMicroseconds(pfr);
  fl.writeMicroseconds(pfl);
  rr.writeMicroseconds(prr);
  rl.writeMicroseconds(prl);
  prevrollError = rollError;
  prevpitchError = pitchError;




}
