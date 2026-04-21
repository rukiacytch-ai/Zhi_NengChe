/*
 * PID.h
 *
 *  Created on: 2026年2月8日
 *      Author: 19929
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

typedef struct
{
       float Target;
       float Actual;
       float Out;

       float Error0;            // 当前误差
       float Error1;            // 上一次的误差
       float Error2;            // 上上一次的误差

       float ErrorInt;          // 误差积分

       float Kp;
       float Ki;
       float Kd;

       float GKD;               // kd乘的参数
       float gyro_z;            // 陀螺仪角速度值
       float KP2;               

       float OutMax;
       float OutMin;

}PID_t;

void PID_Init(PID_t *p);
void PID_Update_Incremental(PID_t *p);      // 增量式PID
void PID_Update_Positional(PID_t *p);       // 位置式PID
void PID_Update_Double_P(PID_t *p);          // 双参数PID

#endif /* CODE_PID_H_ */
