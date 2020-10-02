#include "mpu.h"

void Mpu::init()
{
    Wire.begin(); 
    writeChar(0x00);
}
void Mpu::update()
{
    ReadAccGyr(read_outs); //读出测量值
    float real_vals[7];
    Rectify(read_outs, real_vals); //根据校准的偏移量进行纠正

     //计算加速度向量的模长，均以g为单位
  float fNorm = sqrt(real_vals[0] * real_vals[0] + real_vals[1] * real_vals[1] + real_vals[2] * real_vals[2]);
  float fRoll = GetRoll(real_vals, fNorm); //计算Roll角
  if (real_vals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(real_vals, fNorm); //计算Pitch角
  if (real_vals[0] < 0) {
    fPitch = -fPitch;
  }

  
}

void Mpu::writeChar(char _cha)
{
    Wire.beginTransmission(MPU_ADD); //开启MPU6050的传输
    Wire.write(0x6B); //指定寄存器地址
    Wire.write(_cha); //写入一个字节的数据
    Wire.endTransmission(true); //结束传输，true表示释放总线
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char Mpu::readMPUChar(int nReg) {
  Wire.beginTransmission(MPU_ADD);
  Wire.write(nReg);
  Wire.requestFrom(MPU_ADD, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

 //从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void Mpu::ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU_ADD);
  Wire.write(0x3B);
  Wire.requestFrom(MPU_ADD, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

float Mpu::GetRoll(float *pRealVals, float fNorm) {
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//算得Pitch角。算法见文档。
float Mpu::GetPitch(float *pRealVals, float fNorm) {
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Mpu::Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}