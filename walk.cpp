#include "walk.h"

void WalkClass::walkInit(LegClass &_legfr,LegClass &_legfl,LegClass &_legbr,LegClass &_legbl,float *_yaw_value)
{
	seted_prep =false;

	vacc=0.1f,wacc=0.1f;    //待定
	v_now=0.1;              //设定速度
	te_set=1000;            //设定周期 
	l_set=v_now*te_set;
	V_Max_F=0.4,V_Max_B=-0.4, W_Max=0.3;    
	yaw_value =  _yaw_value;
	setYaw = *yaw_value;
	prep_long=1;
	
	pidYaw.PidInit(1,0.5f,0.05,0.0f,2.f,1.f);
	         
	walkFr.init(_legfr,11,85,l_set,70,te_set,1,te_set*3/2,0,12,1);
	walkFl.init(_legfl,11,85,l_set,70,te_set,1,0,  0,12,1);
	walkBr.init(_legbr,11,85,l_set,70,te_set,1,0,  0,12,1);
	walkBl.init(_legbl,11,85,l_set,70,te_set,1,te_set*3/2,0,12,1);
	
	w_now=0; vr=vl=0;
	v_set=v_now,w_set=w_now;

	 TeMin=0.2f;       //最短周期
	 TeMax=2;          //最大周期
	 MaxLToCalSpeed=0.200;  //m
	 te_factor=0.5;   //0~1
	 isSetSpeedFinished=false;
	 prept_long=0;
}

//准备阶段
void  WalkClass:: prepWalk()
{
	walkFr.move2InitPos(prep_long);
	walkFl.move2InitPos(prep_long);
	walkBr.move2InitPos(prep_long);
	walkBl.move2InitPos(prep_long);
}

void WalkClass::update(float _dt)
{
	walkFr.walkUpdate(_dt);
	walkFl.walkUpdate(_dt);
	walkBr.walkUpdate(_dt);
	walkBl.walkUpdate(_dt);
}