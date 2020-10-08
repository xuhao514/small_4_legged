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
LegClass leg_fl,leg_fr,leg_bl,leg_br;
//WalkLegClass walkLegClass1;
//WalkClass walk;
Mpu mpu6050;
double t_now,t_pre;
double dt;
bool inited=false;
float get_time;
struct SteerAngle
{
  float c1[4];
  float c4[4];
};
SteerAngle steer_ang;
 
float pos = 60;    bool dir = false;    
void singleServoControl(){
   
    if(pos > 100) dir = true;
    if(pos<60) dir =false;
    if(dir) pos -= 0.5;
    if(!dir) pos += 0.5;
    //steering_engine1.setAngle(pos);  
    leg_fl.setPos(11,pos);
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
          if(data_process.dataDecode<SteerAngle>(_temp,&steer_ang))
          {
            leg_fr.setSteerRad(steer_ang.c1[0],steer_ang.c4[0]);
            leg_fl.setSteerRad(steer_ang.c1[1],steer_ang.c4[1]);
            leg_br.setSteerRad(steer_ang.c1[2],steer_ang.c4[2]);
            leg_bl.setSteerRad(steer_ang.c1[3],steer_ang.c4[3]);
            get_time = millis();
          }
        }
        else if(data_process.headId() == 2)
        {
          if(data_process.dataDecode<float>(_temp,&ang))
          {
            // steering_engine.setAngle(ang);
            // steering_engine1.setRatio(ang);
            //leg_fl.setPos(0,ang);
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
  int _len;
  char *arr;
  char _id = 11 ;
  arr=data_process.dataEncode<YPR>(&mpu6050.ypr_data, _id , &_len);
  for(int i=0;i<_len;i++)
  {
    Serial.print(arr[i]);
  }
  arr=data_process.dataEncode<float>(&get_time, 12 , &_len);
  for(int i=0;i<_len;i++)
  {
    Serial.print(arr[i]);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mpu6050.init();
 //steering_engine1.init(1,2,544,2400,160.0,80);
  leg_fl.legInit(2,3);leg_fr.legInit(4,5);leg_bl.legInit(6,7);leg_br.legInit(8,9);
  
  // walkLegClass1.init(leg_fl,11,85,50,30,1000,1,1500,0,12,1);
  // walkLegClass1.move2InitPos(2000);
  //walk.walkInit(leg_fr,leg_fl,leg_br,leg_bl,&mpu6050.ypr_data.y);
  //walk.prepWalk();
  t_now = t_pre = millis();
}

void loop() {

  t_now = millis();
  dt = t_now - t_pre;
  t_pre = t_now;
  mpu6050.update();

  //singleServoControl();
  usartRead();

  //steering_engine1.updateSteering();  
  leg_fl.updateByRad();leg_fr.updateByRad();leg_bl.updateByRad();leg_br.updateByRad();
  //walkLegClass1.walkUpdate(dt);
  //walk.update(dt);
  sendData();
  delay(8);  
}
