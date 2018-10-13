#ifndef __PID_H
#define __PID_H

typedef struct {
	float dt; //循环周期(ms)
	float P;
	float I;
	float D;
	float kP;
	float kI;
	float kD;
	float value;
	float target;//目标值存储指针
	//float * feedback;//当前值存储指针

//	float f_H;//微分先行控制，低通滤波器上限频率

    float error;
    float thisError;
    float lastError;

    //float eLimit;
	float errorUpperLimit;
	float errorLowerLimit;
   // float eILimit;
    float out;
  //  float outI;
//
//	float e[2];//本次误差与上次误差
//	float feedback_LPF[2];//滤波后的本次目标值与上次目标值
//	float ei;//误差积分值
//	float fd;//反馈微分值
//	float e_limit;//误差限辐（用于积分分离，可变kp参数和可变kd参数）
//	float ei_limit;//积分限幅
//	float u;//输出值
	//float uI;//输出积分值
} Pid;

int Direction(void);

float pidCalculator(Pid * pid, float inputValue);

void pidInit(Pid * pid);
//void pid_params_CFG(PID_TypeDef *pid, float *kp, float *ki, float *kd, float dt);//pid同步参数配置

//void pid_limits_CFG(PID_TypeDef *pid, float e_limit, float i_limit, float f_H);//pid非同步参数配置

//void pid_data_CFG(PID_TypeDef *pid, float *target, float *feedback);//pid数据指针配置。pid级联时，可以方便地把&(pid1->u)作为pid2的target

//float pid_Calc(PID_TypeDef *pid);//pid计算

#endif