#ifndef _MPU_H
#define _MPU_H
#include <Wire.h>
#include "utils.h"
#define MPU_ADD 0x68  //MPU-6050的I2C地址
const int nValCnt = 7; //一次读取寄存器的数量
const float fRad2Deg = 57.295779513f; //将弧度转为角度的乘数

class Mpu
{
public:
   void init();
   void writeChar(char _cha);
   //从MPU6050读出一个字节的数据
  //指定寄存器地址，返回读出的值
   unsigned char readMPUChar(int nReg);
void update();
  int read_outs[nValCnt];  //读取的数据
  int calibData[nValCnt]; //校准的数据

  void ReadAccGyr(int *pVals);
  float GetRoll(float *pRealVals, float fNorm);
  float GetPitch(float *pRealVals, float fNorm);
  void Rectify(int *pReadout, float *pRealVals);
};


#endif

