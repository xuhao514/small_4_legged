#include "walk.h"

void WalkLegClass::init(LegClass &_leg,float _x0,float _y0,float _L,float _H0,float _t_floor,float _tair_tfloor_scale,float _start_tn,float _delay_t,int _bezierLen,int _forward)
{
	 MaxL=100,MinL=-100;   //最大最小步长
	 v_leg=_L/1000/_t_floor;
	
	 leg=&_leg; L5=leg->get_L5(); over_flag=false; fixPointFlag=false; 
	 prep=false; walk_leg_num=0;
	
	 t_air=_t_floor*_tair_tfloor_scale;t_floor=_t_floor; tair_tfloor_scale=_tair_tfloor_scale; 
     alp=0; is_add_start_tn =false; delay_cal_t = 0;
	 start_tn = _start_tn;  delay_t = _delay_t;
     Lst=_L;H0st=_H0;
     tn_last=tn_now=0;
	 T_now=T_last=t_floor+t_air;
	 add_phase=false;
	 setBezier( _x0, _y0, _L, _H0, _bezierLen, _forward);
     x=y=0;
}

//求解单个腿角度  _dt:调用时间间隔
void WalkLegClass::walkUpdate(float _dt)
{
	if(delay_cal_t< delay_t)   //延时
	{
		delay_cal_t+=_dt;
		return;
	}
	T_now = t_floor + t_air;
	tn_last = tn_now;
	tn_now = tn_last/T_last * T_now + _dt;    //如果周期突变，保证在周期中时刻的比例不变
	if(is_add_start_tn==false)    //初始相位差只加一次
	{
	  tn_now += start_tn;             //当前腿时刻
	  is_add_start_tn=true;
	}
	if(tn_now>T_now) tn_now-= T_now;  //在周期中的时刻  tnNow-= (int)(tnNow/T_now) * T_now; 
	
	if(tn_last>=t_air && tn_now<t_air)  //判断是否完成一个周期
         over_flag=true;
    else over_flag=false;
	
	if(over_flag==true) walk_leg_num++;
	
	if(tn_now>=t_floor/2+t_air && tn_last<t_floor/2+t_air)  
        fixPointFlag=true;
	else fixPointFlag=false;
	  
	if(tn_now<t_air)                  //曲线段  由wx12 wy12控制
	{	
		//phaFlag=inAir; 
		Bezier(&x,&y,wx12,wy12,12,tn_now/t_air);	
	}
	else if(tn_now<=t_floor+t_air)    //直线段   
	{
		//phaFlag=inEffect; 
		//if(t_floor>WalkTeMin)                                       
            Bezier(&x,&y,wx2,wy2,2,(tn_now-t_air)/t_floor);	    //均匀的直线
		//else                                                                         // 因为电机速度达不到要求 所以直线两端点数密集  //MIT的论文里有描述原因  有空看下
        //    Bezier(&x,&y,wx,wy ,6,(tn_now-t_air)/t_floor);	     //两头密中间稀的直线    目前好像还不稳定
	}
	
   if(alp!=0) Rotate(&x,&y,L5/2,0,forward*alp);  //围绕腿的原点旋转
  
    leg->setPos(x,y);
	leg->update(_dt);
}

void WalkLegClass::reStart()
{
	tn_now=tn_last=0;
	add_phase=false;
}


//原地转弯  让同一边的腿方向相反  forward +-1 ； reverse 反向  
void WalkLegClass::walkTurnOri(int _forward,int reverse,float _Len)
{
	//修改方向 
	setBezier(x0,y0,_Len,H0,bezier_len,_forward);
	if(reverse== 1)  Lst=fabs(Lst);
    if(reverse==-1)  Lst=-fabs(Lst);
}

// 返回速度 m/s    speed：速度    _te：Te值
float WalkLegClass::setLegSpeed(float speed,float _t_floor )
{	    
	t_floor=_t_floor; t_air=tair_tfloor_scale*_t_floor;
	
	float dspeed=speed-v_leg;
    L+=dspeed*_t_floor *1000;
	if(L<MinL)  L=MinL;
	if(L>MaxL)  L=MaxL;

	Lst=L;    //更新设定的步长
	setBezier(x0,y0,L,H0,bezier_len,forward);   //更新贝赛尔点

	v_leg=L/1000/_t_floor;
	return v_leg;
}

//设置贝赛尔曲线的控制点
void WalkLegClass::setBezier(float _x0,float _y0,float _L,float _H0,int _bezierLen,int _forward)
{
	 x0=_x0;y0=_y0;L=_L;H0=_H0;
	 bezier_len=_bezierLen;forward=_forward;
	 if(L>MaxL) L=MaxL;
	 if(L<MinL) L=MinL;
	 float dL;  //贝赛尔曲线水平段至曲线段的控制长度
	 int i=0;
	//6阶行走时的贝赛尔曲线 用于直线段  //主要用在快速行走 
	// if(t_floor<WalkTeMin)
	// {
	//  wx[0]=-L/2+x0;        wx[1]=-L/2+x0;    wx[2]=-L/2+x0;
	//  wx[3]=L/2+x0;         wx[4]=L/2+x0;     wx[5]=L/2+x0;
    //  wy[0]=y0;             wy[1]=y0;         wy[2]=y0;
	//  wy[3]=y0;             wy[4]=y0;         wy[5]=y0;
	// }
	
  if(bezier_len==12)  //12阶行走时的贝赛尔曲线
	{
	 dL=L/11;
	 wx12[0]=L/2;         wx12[1]=L/2+dL;              wx12[2]=L/2+2*dL;    wx12[3]=L/2+3*dL;
    wx12[4]=L/2+4*dL;    wx12[5]=-L/2-4*dL;           wx12[6]=-L/2-3*dL;   wx12[7]=-L/2-3*dL;
    wx12[8]=-L/2-3*dL;   wx12[9]=-L/2-2*dL;           wx12[10]=-L/2-dL;    wx12[11]=-L/2;	
	 wy12[0]=0;wy12[1]=0; wy12[2]=-H0;                 wy12[3]=-H0;
	 wy12[4]=-H0;         wy12[5]=0-H0*6/5;            wy12[6]=-H0;         wy12[7]=-H0;
	 wy12[8]=0;           wy12[9]=0;                   wy12[10]=0;          wy12[11]=0;
		
	 wx2[0]=-L/2;  wx2[1]=L/2;  
	 wy2[0]=0;     wy2[1]=0;
	}
	for(i=0;i<12;i++)
	{
	  wx12[i]+=x0;
      wy12[i]+=y0;
	}
 
	wx2[0]=-L/2+x0;  wx2[1]=L/2+x0;  
	wy2[0]=y0;       wy2[1]=y0;
		
	if(forward==-1)                                      //关于中心对称
	{
		for(i=0;i<6;i++)
		{
			wx[i]=2*L5/2-wx[i];
		}
		for(i=0;i<12;i++)
		{
			wx12[i]=2*L5/2-wx12[i];
		}
		wx2[0]=2*L5/2-wx2[0];  wx2[1]=2*L5/2-wx2[1];
	}
}