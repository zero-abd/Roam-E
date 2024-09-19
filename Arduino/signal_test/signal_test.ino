/*
 * Coded by Abdullah Al Mahmud (Team Baby Musks)
*/

#define S Serial
#define pbutton 11

bool strt = false, con = false;
int initial_turn = 80, sangle = 0;

void setup(){
  pinMode(pbutton, INPUT_PULLUP);
  stop_motor();

  while(!S){}
}

void loop(){
  if(digitalRead(pbutton) == 0 && !con){
    digitalWrite(13, LOW);
    con = true;
  }
}
