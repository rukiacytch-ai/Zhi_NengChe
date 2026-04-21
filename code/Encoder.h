/*
 * Encoder.h
 *
 *  Created on: 2026年2月8日
 *      Author: 19929
 */

#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "stdint.h"
#include "math.h"

#define ENCODER_DIR_Right                     (TIM5_ENCODER)                         // 右轮 带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_Right               (TIM5_ENCODER_CH1_P10_3)               // 右轮 PULSE 对应的引脚
#define ENCODER_DIR_DIR_Right                 (TIM5_ENCODER_CH2_P10_1)               // 右轮 DIR 对应的引脚

#define ENCODER_DIR_Left                      (TIM6_ENCODER)                          // 左轮 带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_Left                (TIM6_ENCODER_CH1_P20_3)               // 左轮 PULSE 对应的引脚
#define ENCODER_DIR_DIR_Left                  (TIM6_ENCODER_CH2_P20_0)               // 左轮 DIR 对应的引脚

extern int16 encoder_data_dir_right;            // 原始数据
extern int16 encoder_data_dir_left;             // 原始数据
extern float encoder_right_speed;               // 速度（n转/s）
extern float encoder_left_speed;                // 速度（n转/s）
extern float encoder_right_loc_delta;           // 位移增量（cm）
extern float encoder_left_loc_delta;            // 位移增量
extern float encoder_right_loc;                 // 位移总量
extern float encoder_left_loc;                  // 位移总量

void Encoder_Init(void);
void Encoder_Get(void);
void Encoder_Get_Speed(void);
void Encoder_Get_Location(void);


#endif /* CODE_ENCODER_H_ */
