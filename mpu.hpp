#ifndef _MPU_H
#define _MPU_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

class Mpu
{
public:
  void init()
  {
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
  };
  void update()
  {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//     // Serial.print("ypr\t");
//     // Serial.print(ypr[0] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.print(ypr[1] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.println(ypr[2] * 180 / M_PI);
    }
  };
  MPU6050 mpu;
  uint8_t fifoBuffer[64];
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];  //弧度
};


#endif

