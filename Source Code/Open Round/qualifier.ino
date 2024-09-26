/*
 * Coded by Abdullah Al Mahmud (Team Paragon)
 * Libraries:
 * HC-SR04-NewPing.ino
 * 
*/

#include <Servo.h>
#include <NewPing.h>
#include <math.h>
#include <PID_v1.h>

#define ll long long
#define servo 3
#define sonar_right_trig 9
#define sonar_right_echo 9
#define sonar_left_trig 10
#define sonar_left_echo 10
#define mf_en 4
#define mb_en 11
#define mf_pwm 5
#define mb_pwm 6

#define S Serial
#define maxd 250


double min_angle = 45, max_angle = 115, initial_angle=75, pid_target = 30, pid_in, pid_out;
double Kp=4, Ki=0.05, Kd=0.9;
PID sonar_pid(&pid_in, &pid_out, &pid_target, Kp, Ki, Kd, DIRECT);

NewPing right_sonar(sonar_right_trig, sonar_right_echo, maxd);
NewPing left_sonar(sonar_left_trig, sonar_left_echo, maxd);
Servo turn;

ll last_time = -3000, present_time = 0, rotation = 0, final_delay = 1500, gap_time = 2300;
int l_dis, r_dis;

void setup(){
  S.begin(9600);

  for(int i = 4; i <= 7; ++i) pinMode(i, OUTPUT);
  pinMode(11, OUTPUT);

  turn.attach(servo);
  turn.write(initial_angle);
  sonar_pid.SetMode(AUTOMATIC);
  sonar_pid.SetSampleTime(25);
  stop();

  delay(1000);
  turn.write(initial_angle);
  reset_motor();
  forward(120);
  start_the_game();
}

void loop(){

}

void start_the_game(){
  int r_dis_cnt = 0, l_dis_cnt = 0;
  while(1){
    r_dis = right_sonar.ping_cm();
    l_dis = left_sonar.ping_cm();
    if(r_dis == 0) r_dis = maxd;
    if(l_dis == 0) l_dis = maxd;

    if(r_dis > 200) r_dis_cnt++;
    if(l_dis > 200) l_dis_cnt++;
    if(r_dis_cnt >= 5 || l_dis_cnt >= 5) break;
    
    S.print(r_dis);
    S.print("   ");
    delay(10);
    S.print(l_dis);
    S.println("  cm");
    delay(50);
  }  
  if(r_dis_cnt >= 5){
    deviate(max_angle);
    delay(500);
    main_pid(1);
  }
  if(l_dis_cnt >= 5){
    deviate(min_angle);
    delay(500);
    main_pid(0);
  }
}

void main_pid(int dir){
  // for clockwise direction:
  if(dir == 1){
    gap_time = 2600;
    final_delay = 1200;
  }else{
    gap_time = 2300;
    final_delay = 1500;
  }
  while(1){
    r_dis = right_sonar.ping_cm();
    l_dis = left_sonar.ping_cm();
    if(r_dis == 0) r_dis = maxd;
    if(l_dis == 0) l_dis = maxd;

    S.print(r_dis);
    S.print("   ");
    delay(10);
    S.print(l_dis);
    S.println("  cm");
    delay(50);
    

    sonar_pid.SetOutputLimits(min_angle, max_angle);
    // for clockwise direction:
    if(dir == 1){
      sonar_pid.SetControllerDirection(REVERSE);
      pid_in = r_dis;
    }else{
      pid_in = l_dis;
    }
    sonar_pid.Compute();
    pid_out = constrain(pid_out, min_angle, max_angle);
    turn.write(pid_out);
    // S.println(pid_out);
    // S.println("  ");
    

    present_time = millis();
    if(rotation <= 11 && pid_in > 200 && present_time - last_time > gap_time){
      rotation++;
      last_time = millis();
    }
    if(rotation > 11){
      if(present_time - last_time > final_delay){
        stop();
        delay(600000);
      }
    }
  }
}


void reset_motor(){
  digitalWrite(mb_en, HIGH);
  digitalWrite(mf_en, HIGH);
  analogWrite(mb_pwm, 0);
  analogWrite(mf_pwm, 0);
}

void stop(){
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

void deviate(int angle){
  turn.write(angle);
}
