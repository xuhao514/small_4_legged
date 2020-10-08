#ifndef _WALKLEG
#define _WALKLEG
#include "leg.h"

struct WalkLegState
{
    bool to_init_pos;
    bool in_delay;
    bool walking;
    bool in_air_else_floor;
    float x,y;
    float tn;
};


class WalkLegClass
{
public:
    //(x0,y0) 行走曲线中心点 L:步长 H:步高  _t_floor:着地相时长 _tair_tfloor_scale:Tair与t_floor的比值 _start_tn:周期的起始时刻 _delay_t：启动延时
    void init(LegClass &_leg,float _x0,float _y0,float _L,float _H0,float _t_floor,float _tair_tfloor_scale,float _start_tn,float _delay_t,int _bezierLen,int _forward);	
    void walkUpdate(float _dt);
    float setLegSpeed(float speed,float _te);
    void walkTurnOri(int forward,int reverse,float Len);
    void setBezier(float _x0,float _y0,float _L,float _H0,int _bezierLen,int _forward);
    void reStart();        //ToDo
    bool move2InitPos(float _use_t_long);   //移动动初始位置
    WalkLegState walk_leg_state;
// void SetWalkLegAlp(void);
//	void WalkAccSet(void);
private:
    void calPos(float _tn);  //计算位置  _tn:周期中的时刻
    void calTnNow(float _dt); 
private: 
    float x0;             //(x0,y0) 行走曲线中心点 mm
    float y0;    
    float L;              //步长  当前的步长   mm
    float H0;             //步高  
    float Lst;            //初始设置的步长 
    float H0st;           //初始设置的步高
    float t_floor;        //着地相时长 ms
    float t_air;           //腾空相时长	ms
    float tair_tfloor_scale;  //Tair与t_floor的比值
    float start_tn;       //周期的起始时刻

    bool is_add_start_tn; 
    float alp;            // 曲线旋转角度  弧度 //上坡时可能用到
    int bezier_len;        // 使用的贝赛尔曲线的数组长度
    float wx12[12];       //11阶贝赛尔曲线
    float wy12[12];
    float wx2[2];         //1阶贝赛尔曲线  用于走直线
    float wy2[2];         //
    int forward;          //足端曲线的前方与车身前身的正反号  当前后腿的安装方向相反时，需要注意
    LegClass *leg;         //与腿绑定
    float L5;
    float tn_now ,tn_last;  //记录一个周期中的相位  //记录在周期中运行的时刻  范围：0~Tm+Tm1
    float T_last,T_now;      //一个周期的总长度 周期长期可变 记录上周期与此时周期时间长度 
    bool over_flag;            // 一步完成的标志  从抬腿开始 Tn 上一时刻>t_air  这一时刻<t_air  即置true  否则false
    bool fixPointFlag;        //曲线不动点  此时可进行大幅度参数修改 如Lst等  x0不可在此时改变   时间只能渐变
   // ActionPhaseMode phaFlag; //所在相位标志 
    int walk_leg_num;          //单腿步数
    float v_leg;         //单腿的速度

    float delay_t;      //初始延时 ms
    bool prep;
    float prepc1,prepc4;  //记录需要运动到的位置
    float MaxL,MinL;      //最大最小步长	      
    bool add_phase;        //是否加上的相位差
    float x,y;  //计算的坐标 mm
    float use_t_long;   //ms
};

#endif