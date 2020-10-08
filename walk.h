#ifndef _WALKCLASS
#define _WALKCLASS
#include "leg.h"
#include "walkLeg.h"
#include "pid.hpp"

//行走的具体实现
class WalkClass
{
public:
    void walkInit(LegClass &_legfr,LegClass &_legfl,LegClass &_legbr,LegClass &_legbl,float *_yaw_value);	
    void prepWalk();
    void update(float _dt);	

private:
    WalkLegClass walkFl,walkBl,walkFr,walkBr;
    bool seted_prep;
    int walkNum;                            //步数
    float prep_long;                        //准备阶段的时长

    float l_set,te_set;      //初始步长，着地相时长 
    float v_now,w_now;      //当前速度设定值   //v:m/s  w:rad/s(逆时针为正)
    float v_set,w_set;      //设置速度
    float v_rea,w_rea;      //实际值  算出来的
    float vacc,wacc;        //加速度     角加速度       
    float V_Max_F,V_Max_B,W_Max;  //最大前进后退速度，最大角速度 
    float vl,vr;            //左侧速度  右侧速度   设定值
    float vr_rea,vl_rea;    //实际值
    float te_factor;        //te改变引起速度变化的比例
    float setYaw;           //设定的Yaw值    会随角速度变化
    PID pidYaw;
    bool isSetSpeedFinished; 
    float yawIncrease;
    float *yaw_value;


    float TeMin;  //最短周期
    float TeMax;  //最大周期
    float MaxLToCalSpeed;  //用于计算周期的最大步长  m
    float prept_long;
};


#endif