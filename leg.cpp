
#include "leg.h"

void LegClass::legInit(int _id_c1,int _id_c4){
	c1=c4=c1t=c4t=0;
	//机构参数
  ////  L1~L6杆长  ALP L6与L2夹角  ALP向外为正(逆时针)
   L1=50; L2=80;  L3=80;  L4=50;  L5=22; L6=80; ALP=0.0;
   c10=PI; c40=0; //角度初值   计算坐标系与运动开始坐标系的差值
   r_min = 50; r_max = 110; 
   engine_c1.init(_id_c1,_id_c1,544,2400,160.0,80);
   engine_c4.init(_id_c4,_id_c4,544,2400,160.0,80);
}

void LegClass::legInit(float _L1,float _L2,float _L3,float _L4,float _L5,float _L6,float _ALP,float _c10,float _c40)
{
   L1=_L1; L2=_L2;  L3=_L3;  L4=_L4;  L5=_L5; L6=_L6; ALP=_ALP;
   c10=_c10; c40=_c40;
	
   c1=c4=c1t=c4t=0;
}

void LegClass::update()
{
	engine_c1.setRad(c1);
	engine_c4.setRad(c4);
	engine_c1.updateSteering();
	engine_c4.updateSteering();
}

bool LegClass::setPos(float _x,float _y)
{
	if( (_x*_x + _y * _y > r_max * r_max) || (_x*_x + _y * _y < r_min * r_min)) 
		return false;

	x= _x; y = _y;
	Njie();
	return true;
}

//闭链五杆的逆解  被Njie()调用
void LegClass::nije_5( float *_c1, float *_c4, float x1,float y1,float l1,float l2,float l3,float l4,float l5){
    
    A=2*l1*y1;B=2*l1*x1; C=l2*l2-l1*l1-x1*x1-y1*y1;
    *_c1=2*atan((A+sqrt(A*A+B*B-C*C))/(B-C)) ;
    if(*_c1<0) *_c1+=2*PI;  //

    a=2*y1*l4;
    b=2*l4*(x1-l5);
    c=l3*l3+2*l5*x1-l4*l4-l5*l5-x1*x1-y1*y1;
    *_c4=2*atan((a-sqrt(a*a+b*b-c*c))/(b-c));
}

//开链五杆逆解
void LegClass::Njie(){
	
	nije_5(&c1,&c4,x,y,L1,L6,L3,L4,L5);

/////////////开链计算c4出错???????????////////////////////////////////////////
	// nije_5(&c1,(float*)0,x,y,L1,L6,L3,L4,L5); //利用L1,L6计算c1;
	// m=L1*cos(c1);n=L1*sin(c1);
    // float _b=-ALP;
	// x1=L2/L6*((x-m)*cos(_b)-(y-n)*sin(_b))+m;
    // y1=L2/L6*((x-m)*sin(_b)+(y-n)*cos(_b))+n;  //得到闭链五杆端点的坐标
	// nije_5((float*)0,&c4,x1,y1,L1,L2,L3,L4,L5); //计算c4

	c1 -=c10;           //计算坐标系与运动起始位置的差值
	c4 -=c40;
}


//开链五杆正解   
void LegClass::Zjie(){
	float _c1,_c4;
	_c1=c1+c10; _c4=c4+c40;
	
	xb=L1*cos(_c1);yb=L1*sin(_c1);
	xd=L5+L4*cos(_c4);yd=L4*sin(_c4);
	lbd=sqrt((xd-xb)*(xd-xb)+(yd-yb)*(yd-yb));
	A0=2*L2*(xd-xb);
	B0_=2*L2*(yd-yb);
	C0=L2*L2+lbd*lbd-L3*L3;
	u2=2*atan((B0_+sqrt(A0*A0+B0_*B0_-C0*C0))/(A0+C0));
	xc=xb+L2*cos(u2);
	yc=yb+L2*sin(u2);
	float b_=-ALP;
	float _m=L1*cos(_c1);
    float _n=L1*sin(_c1);
	x=L6/L2*((xc-m)*cos(-b_)-(yc-n)*sin(-b_))+_m;
	y=L6/L2*((xc-m)*sin(-b_)+(yc-n)*cos(-b_))+_n;
}