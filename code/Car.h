/*
 * Car.h
 *
 *  Created on: 2026年2月8日
 *      Author: 19929
 */

#ifndef CODE_CAR_H_
#define CODE_CAR_H_

#include "stdint.h"

#define Left_Dir                    (P02_4)                         // 左轮方向端
#define Right_Dir                   (P02_6)                         // 右轮方向端
#define Left_PWM                   (ATOM0_CH5_P02_5)                // 左轮PWM
#define Right_PWM                  (ATOM0_CH7_P02_7)                // 右轮PWM
#define fuya_PWM                  (ATOM2_CH3_P00_12)                // 无刷PWM


void Car_Init(void);
void Car_go_forward(uint16_t Speed);
void Car_go_Back(uint16_t Speed);

void Left_Go_Forward(uint16 Speed);
void Left_Go_Back(uint16 Speed);
void Right_Go_Forward(uint16 Speed);
void Right_Go_Back(uint16 Speed);
void Fuya_Speed(uint8 Speed);


#endif /* CODE_CAR_H_ */
