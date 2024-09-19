/*
 * Coded by Abdullah Al Mahmud (Team Baby Musks)
 * Libraries:
 * Servo
*/

#include<Servo.h>
#include <NewPing.h>


Servo turn;

#define ll long long
#define S Serial
#define servo 3
#define mf_en 4
#define mb_en 7
#define mf_pwm 5
#define mb_pwm 6
#define sonar_front_trig 9
#define sonar_front_echo 9
#define maxd 200
#define pbutton 11

NewPing front_sonar(sonar_front_trig, sonar_front_echo, maxd);

bool strt = false, con = false;
int initial_turn = 80, sangle = 0, f_dis = 0;

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
  digitalWrite(13, HIGH);
}

void loop(){
  if(digitalRead(pbutton) == 0 && !con){
    S.print('S');
    digitalWrite(13, LOW);
    con = true;
  }
  if(con){
    f_dis = front_sonar.ping_cm();
    int s_pred = 999;
    if(S.available()){
      char s_data = S.read();
      s_pred = (int)s_data;
    }
    if(s_pred == 10) s_pred = 999;
    if(s_pred == 111) stop_all();
    if(s_pred == 112) uturn();
  
    if(s_pred != 999){
      turn.write((s_pred+40));
      if(strt == false){
        strt = true;
        reset_motor();
        forward(60);
      }
    }
  }
}

void stop_all(){
  stop_motor();
  con = false;
  strt = false;
  turn.write(initial_turn);
}

void uturn(){
  reset_motor();
  turn.write(40);
  backward(60);
  delay(2500);
  turn.write(110);
  forward(60);
  delay(1000);
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
