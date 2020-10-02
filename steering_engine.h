#ifndef _STEERING_ENGINE_H
#define _STEERING_ENGINE_H
#include <Servo.h> 
#include "utils.h"

class SteeringEngine
{
private:
	int id;                 // 舵机id  应该与 io口一致
    int SERVO_PIN;          //io口
    Servo myservo; 
    float duty_ratio_min;  //最小pwm
	float duty_ratio_max;  //最大
    float angle_range;      //转动角度范围  度
	bool enable;
	float set_ratio;        //设定值
	float set_speed;        //设定速度    占空比/ms
	float ratio;           //计算用的 占空比
	float ang;              //角度         //中间时角度为零
	float ang_min,ang_max;  //角度限制
	float start_ration;    //起始占空比
	float start_ang;       //起始角度
public:
    void init(int _id ,int _pin, float _duty_ratio_min,float _duty_ratio_max, float _angle_range,float _start_ang);  //³õÊ¼»¯
    //设定角度值  相对于起始角度
	void setAngle(float _ang);              
	//设定pwm值
	void setRatio(float _ratio);   
	//设定弧度值   相对于起始角度
	void setRad(float _rad);   
	//更新            
	void updateSteering();          
	//限定转动的范围        
	void setAngRang(float _ang_min,float _ang_max);  
	float getAng();
	float getRatio();
};

#endif
