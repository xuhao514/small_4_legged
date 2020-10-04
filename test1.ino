#include <Servo.h>                // 声明调用Servo.h库
#include "steering_engine.h"
#include "data_processing.h"
#include "leg.h"
#include "walk.h"
#include "mpu.hpp"

SteeringEngine steering_engine1;
DataProcess data_process;
float ang = 0;

float value;
LegClass leg1;
WalkLegClass walkLegClass1;
Mpu mpu6050;

struct SendValue
{
 float a,b;
} ;
SendValue send_value;
struct YPR{
  float y,p,r;
};
YPR ypr_data;

float pos = 60;    bool dir = false;    
void singleServoControl(){
   
    if(pos > 100) dir = true;
    if(pos<60) dir =false;
    if(dir) pos -= 0.5;
    if(!dir) pos += 0.5;
    //steering_engine1.setAngle(pos);  
    leg1.setPos(11,pos);
}  

void usartRead()
{
  int _dat_len = Serial.available();
  if(_dat_len>0)
  {
    for(int i=0;i<_dat_len;i++)
    {
      char _temp = Serial.read();
      if(data_process.getHeadMsg(_temp))
		  {
        if(data_process.headId() == 1)
        {
          if(data_process.dataDecode<float>(_temp,&ang))
            {
              // steering_engine.setAngle(ang);
              // steering_engine1.setRatio(ang);
              leg1.setPos(0,ang);
            }
        }
        else
        {
          data_process.clearFlag();
        }
      }
    }
    
  }

}

void sendData()
{
  // value = steering_engine.getAng();
  // value = steering_engine1.getRatio();
  ypr_data.y =mpu6050.ypr[0]*180 / M_PI; ypr_data.p = mpu6050.ypr[1]*180 / M_PI; ypr_data.r = mpu6050.ypr[2]*180 / M_PI;
  send_value.a = leg1.get_c1();
  send_value.b = leg1.get_c4();
  int _len;
  char *arr;
  char _id = 11 ;
  arr=data_process.dataEncode<YPR>(&ypr_data, _id , &_len);
  for(int i=0;i<_len;i++)
  {
    Serial.print(arr[i]);
  }
}
double t_now,t_pre;
double dt;

void setup() {
  // put your setup code here, to run once:
 
  leg1.legInit(2,3);
  walkLegClass1.init(leg1,11,85,50,30,1000,1,1500,0,12,1);
  //steering_engine1.init(1,2,544,2400,160.0,0);
  Serial.begin(115200);
  t_now = t_pre = 0;
  walkLegClass1.move2InitPos(2000);
  mpu6050.init();
}

void loop() {
  t_now = millis();
  dt = t_now - t_pre;
  t_pre = t_now;
  mpu6050.update();
  //singleServoControl();
 // usartRead();

  //steering_engine1.updateSteering();  
  //leg1.update(dt);
  walkLegClass1.walkUpdate(dt);
  sendData();
  delay(5);  
}
