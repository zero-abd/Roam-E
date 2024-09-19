/*
 * Coded by Abdullah Al Mahmud (Team Baby Musks)
 * Libraries:
 * HC-SR04-NewPing.ino
 * 
*/

#include <Servo.h>
#include <NewPing.h>
#include <math.h>
#include <PID_v1.h>

#define ll long long
#define servo 3 // servo motor at pin 10 pwm
#define sonar_right_trig 8
#define sonar_right_echo 8
#define sonar_left_trig 10
#define sonar_left_echo 10
#define mf_en 4
#define mb_en 7
#define mf_pwm 5
#define mb_pwm 6

#define S Serial
#define maxd 250 // maximum distance measured by sonar


//Define Variables we'll be connecting to
double min_angle = 40, max_angle = 150, initial_angle=80, pid_target = 30, pid_in, pid_out;
double Kp=4, Ki=0.05, Kd=0.9;
PID sonar_pid(&pid_in, &pid_out, &pid_target, Kp, Ki, Kd, DIRECT);

NewPing right_sonar(sonar_right_trig, sonar_right_echo, maxd);
NewPing left_sonar(sonar_left_trig, sonar_left_echo, maxd);
Servo turn;

ll last_time = -3000, present_time = 0, rotation = 0, final_delay = 1500, gap_time = 2300;
int l_dis, r_dis;

void setup(){
  S.begin(115200);

  for(int i = 4; i <= 7; ++i) pinMode(i, OUTPUT);
  turn.attach(servo);
  turn.write(initial_angle);
  sonar_pid.SetMode(AUTOMATIC);
  sonar_pid.SetSampleTime(50);
  stop();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  while(!S){}
  digitalWrite(13, LOW);

//  delay(5000);
//  reset_motor();
//  forward(60);
}

void loop(){
  int data = get_data();
  if(data != 999){
//    turn.write((data+40));
  }
}

int get_data(){
  int s_pred = 999;
  if(S.available()){
    char s_data = S.read();
    s_pred = (int)s_data;
    S.print(s_pred);
  }
  if(s_pred == 10) s_pred = 999;
  return s_pred;
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
  }  
  if(r_dis_cnt >= 5) main_pid(1);
  if(l_dis_cnt >= 5) main_pid(0);
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

//     S.print(r_dis);
//     S.print("   ");
//     delay(10);
//     S.print(l_dis);
//     S.println("  cm");
//     delay(500);
    

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

void deviate(int angle){
  turn.write(angle);
}
