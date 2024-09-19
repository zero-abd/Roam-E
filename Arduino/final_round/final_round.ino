/*
 * Coded by Abdullah Al Mahmud (Team Baby Musks)
 * Libraries:
 * Servo
 * MPU6050_light
*/

#include<Servo.h>
#include "MPU6050_light.h"

MPU6050 mpu(Wire);
Servo turn;

#define ll long long
#define S Serial
#define servo 3
#define mf_en 4
#define mb_en 7
#define mf_pwm 5
#define mb_pwm 6
#define pbutton 11

bool strt = false, con = false;
int initial_turn = 80, sangle = 0;

void setup(){
  pinMode(pbutton, INPUT_PULLUP);
  for(int i = 4; i <= 7; ++i) pinMode(i, OUTPUT);
  turn.attach(servo);
  turn.write(initial_turn);
  stop_motor();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  S.begin(9600);
  while(!S){}
  digitalWrite(13, LOW);
  delay(1000);

  Wire.begin();
  while(mpu.begin() != 0){}
  digitalWrite(13, HIGH);
  mpu.calcOffsets();
}

void loop(){
  if(digitalRead(pbutton) == 0 && !con){
    digitalWrite(13, LOW);
    con = true;
  }
  if(con){
    mpu.update();
    if(strt == false) sangle = round(abs(mpu.getAngleZ()));
    else if(round(abs(mpu.getAngleZ())) - sangle >= 1020){
      stop_all();
      con = false;
    }
    
    int s_pred = 999;
    if(S.available()){
      char s_data = S.read();
      s_pred = (int)s_data;
    }
    if(s_pred == 10) s_pred = 999;
  
    if(s_pred != 999){
      turn.write((s_pred+40));
      if(strt == false){
        strt = true;
        reset_motor();
        forward(70);
      }
    }
  }
}

void stop_all(){
  stop_motor();
  turn.detach();
}

void reset_motor(){
  digitalWrite(mb_en, HIGH);
  digitalWrite(mf_en, HIGH);
  analogWrite(mb_pwm, 0);
  analogWrite(mf_pwm, 0);
}

void stop_motor(){
  digitalWrite(mb_en, LOW);
  digitalWrite(mf_en, LOW);
  analogWrite(mb_pwm, 0);
  analogWrite(mf_pwm, 0);
}

void backward(int speed){
  // digitalWrite(mb_en, HIGH);
  // digitalWrite(mf_en, LOW);
  analogWrite(mb_pwm, speed);
  analogWrite(mf_pwm, 0);
}

void forward(int speed){
  // digitalWrite(mb_en, LOW);
  // digitalWrite(mf_en, HIGH);
  analogWrite(mb_pwm, 0);
  analogWrite(mf_pwm, speed);
}
