#include "MPU6050_light.h"

MPU6050 mpu(Wire);

void setup(){
  Serial.begin(9600);
  Wire.begin();
  while (mpu.begin() != 0){}
  mpu.calcOffsets();
}

int MPUtimer = 0, yawAngle = 0, start = 0, startAngle = 0;

void loop(){
  mpu.update();
    sangle = round(abs(mpu.getAngleZ()));
    Serial.println((round(abs(mpu.getAngleZ())) - sangle);
}
