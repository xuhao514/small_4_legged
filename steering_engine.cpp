#include "steering_engine.h"

void SteeringEngine::init(int _id ,int _pin, float _duty_ratio_min,float _duty_ratio_max, float _angle_range,float _start_ang)
{
  id = _id;
  duty_ratio_min = _duty_ratio_min;
  duty_ratio_max = _duty_ratio_max;
  angle_range = _angle_range;   //角度可变的范围
  ang_min = 0;
  ang_max = _angle_range;
  enable = true;
  start_ang = _start_ang; 
  set_ratio = ratio = start_ration = _start_ang/_angle_range * (duty_ratio_max - duty_ratio_min);
  set_speed = 10;   //
  SERVO_PIN = _pin;
  myservo.attach(SERVO_PIN);
}

void SteeringEngine::setAngle(float _ang)
{
  ang = _ang + start_ang;
  if(ang < ang_min) ang = ang_min;
  if(ang > ang_max) ang = ang_max;
  set_ratio =  (ang / angle_range) *(duty_ratio_max - duty_ratio_min) + duty_ratio_min  ;
}

void SteeringEngine::setRad(float _rad)
{
  setAngle(_rad*180/3.14159);
}

void SteeringEngine::setRatio(float _ratio)
{
  set_ratio = _ratio;
}

void SteeringEngine::updateSteering()
{
  if(abs(set_ratio-ratio) > set_speed)
  {
    ratio += abs(set_ratio-ratio)/(set_ratio-ratio) * set_speed;
  }
  else
    ratio = set_ratio;

  // myservo.write((int)_ang);
  myservo.writeMicroseconds((int)ratio);
  
}

void SteeringEngine::setAngRang(float _ang_min,float _ang_max)
{
  ang_min = _ang_min;
  ang_max = _ang_max;
}

float SteeringEngine::getAng()
{
  return myservo.readMicroseconds()/(duty_ratio_max - duty_ratio_min) * angle_range;
}

float SteeringEngine::getRatio()
{
  return myservo.readMicroseconds();
}

