#ifndef _MPU_H
#define _MPU_H
//教程：https://www.jianshu.com/p/d8b04a77158c
//库位置：https://github.com/jrowberg/i2cdevlib

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
struct YPR{
  float y,p,r;
};

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
    updata_time = millis();
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      rec_time = millis();
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      ypr_data.y =ypr[0]*180 / M_PI; ypr_data.p = ypr[1]*180 / M_PI; ypr_data.r = ypr[2]*180 / M_PI;
//     // Serial.print("ypr\t");
//     // Serial.print(ypr[0] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.print(ypr[1] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.println(ypr[2] * 180 / M_PI);
    }
    if(updata_time - rec_time> 1000)  //超过一秒就认为断开了连接
      connected =false;
    else
      connected =true;
  };
  bool isconnected(){return connected;};
  YPR ypr_data;  //单位 角度
private:
  MPU6050 mpu;
  uint8_t fifoBuffer[64];
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];  //弧度
  float updata_time;  //更新运行时刻
  float rec_time;     //接收到数据时刻
  bool connected;     //是否连接上了
};


#endif

