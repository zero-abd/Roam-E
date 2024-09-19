/*
 * Coded by Abdullah Al Mahmud (Team ZER0)
 * Libraries:
 * HC-SR04-NewPing.ino
 * 
*/

#include <Servo.h>
#include <NewPing.h>
#include <math.h>
#include <PID_v1.h>
#include <HUSKYLENS.h>

#define ll long long
#define servo 10 // servo motor at pin 10 pwm
#define sonar_right_trig 2
#define sonar_right_echo 3
#define sonar_left_trig 8
#define sonar_left_echo 9
#define sonar_front_trig 11
#define sonar_front_echo 12
#define mb_en 4
#define mf_en 7
#define mb_pwm 5
#define mf_pwm 6

#define S Serial
#define maxd 250 // maximum distance measured by sonar


double min_angle = 91, max_angle = 170, initial_angle=127, pid_target = 0, pid_in, pid_out;
double Kp=4, Ki=0.05, Kd=0.9;
PID sonar_pid(&pid_in, &pid_out, &pid_target, Kp, Ki, Kd, DIRECT);

NewPing right_sonar(sonar_right_trig, sonar_right_echo, maxd);
NewPing left_sonar(sonar_left_trig, sonar_left_echo, maxd);
NewPing front_sonar(sonar_front_trig, sonar_front_echo, maxd);
Servo turn;
HUSKYLENS huskylens;

ll last_time = -3000, present_time = 0, rotation = 0, final_delay = 1500, gap_time = 2300, last_pid_time=-3000, present_pid_time=0;
int mheight = 0, mcenter = 0, l_dis, r_dis, f_dis;  // 320 * 240 pixel husky

void setup(){
  S.begin(9600);
  Wire.begin();
  huskylens.begin(Wire);

  for(int i = 4; i <= 7; ++i) pinMode(i, OUTPUT);
  turn.attach(servo);
  turn.write(initial_angle);
  sonar_pid.SetMode(AUTOMATIC);
  sonar_pid.SetSampleTime(50);
  stop();

  delay(10000);
  turn.write(initial_angle);
  reset_motor();
  forward(100);
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
    f_dis = front_sonar.ping_cm();
    if(r_dis == 0) r_dis = maxd;
    if(l_dis == 0) l_dis = maxd;
    if(f_dis == 0) f_dis = maxd;

    if(dir == 1 && f_dis > 150 && r_dis > 200) r_dis = max(100 - l_dis - 20, 5);
    if(dir == 0 && f_dis > 150 && l_dis > 200) l_dis = max(100 - r_dis - 20, 5);

//     S.print(r_dis);
//     S.print("   ");
//     delay(10);
//     S.print(f_dis);
//     S.print("   ");
//     delay(10);
//     S.print(l_dis);
//     S.println("  cm");
//     delay(1000);

    
//     int box_id = detect_box();
//     if(box_id==0){
//       if(millis() - last_pid_time >= 1500){
//         pid_target = 0;
//       }
//     }else if(box_id == 2 && mheight >= 30 && mcenter >= 40 && mcenter <= 300){
//       pid_target = 15;
//       last_pid_time = millis();
//     }else if(box_id == 1 && mheight >= 35
//     && mcenter >= 20 && mcenter <= 280){
//       pid_target = 50;
//       last_pid_time = millis();
//     }

//     S.print(box_id);
//     S.print(" -id   ");
//     S.print(mheight);
//     S.print(" -height   ");
//     S.print(mcenter);
//     S.println(" -center  ");
//     delay(1000);
    

    sonar_pid.SetOutputLimits(min_angle, max_angle);
    // for clockwise direction:
    if(dir == 1){
      sonar_pid.SetControllerDirection(REVERSE);
      pid_in = (r_dis - l_dis);
    }else{
      pid_in = (r_dis - l_dis);
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
        delay(60000);
      }
    }
  }
}

int detect_box(){
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  
  if(!huskylens.available()){
    return 0;
  }else{
    int height1 = 0, height2 = 0, center1 = 0, center2 = 0;
    while(huskylens.available()){
      HUSKYLENSResult result = huskylens.read();
      if(result.ID == 1){
        height1 = result.height;
        center1 = result.xCenter;
      }
      else if(result.ID == 2){
        height2 = result.height;
        center2 = result.xCenter;
      }
    }
    if(height1 >= height2){
      mheight = height1;
      mcenter = center1;
      return 1;
    }
    else{
      mheight = height2;
      mcenter = center1;
      return 2;
    }
  }
  
  return 0;
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
