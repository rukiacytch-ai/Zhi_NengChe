/*
 * PID.c
 *
 *  Created on: 2026年2月8日
 *      Author: 19929
 */

#include <math.h>
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中


void PID_Init(PID_t *p)
{
    p->Actual = 0;
    p->Out = 0;
    p->Target = 0;
    p->Error0 = 0;
    p->Error1 = 0;
    p->Error2 = 0;
    p->GKD = 0;
    p->gyro_z = 0;
    p->KP2 = 0;
    p->ErrorInt = 0;
}


// 增量式PID
void PID_Update_Incremental(PID_t *p)
{
    p->Error2 = p->Error1;
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    p->Out += p->Kp * (p->Error0 - p->Error1)
            + p->Ki * p->Error0
            + p->Kd * (p->Error0 - 2*p->Error1 + p->Error2);

    if(p->Out > p->OutMax) p->Out = p->OutMax;
    if(p->Out < p->OutMin) p->Out = p->OutMin;
}

// 位置式PID
void PID_Update_Positional(PID_t *p)
{
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    // 积分项
    if (p->Ki != 0)
    {
        p->ErrorInt += p->Error0;
        // 积分限幅
        if(p->ErrorInt > 10) p->ErrorInt = 10;
        if(p->ErrorInt < -10) p->ErrorInt = -10;
    }
    else
    {
        p->ErrorInt = 0;
    }

    // 位置式PID计算
    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           + p->Kd * (p->Error0 - p->Error1);

    // 输出限幅
    if(p->Out > p->OutMax) p->Out = p->OutMax;
    if(p->Out < p->OutMin) p->Out = p->OutMin;
}

// 双PD式PID
void PID_Update_Double_P(PID_t *p)
{
    p->Error1 = p->Error0;                      // 上一次的角度偏差
    p->Error0 = p->Target - p->Actual;          // 这一次的角度偏差

    // 双PD式PID计算
    p->Out = p->Kp * p->Error0
           + p->KP2 * p->Error0 * fabs(p->Error0)
           + p->Kd * (p->Error0 - p->Error1)
           + p->GKD * p->gyro_z;

    // 输出限幅
    if(p->Out > p->OutMax) p->Out = p->OutMax;
    if(p->Out < p->OutMin) p->Out = p->OutMin;
}




#pragma section all restore
