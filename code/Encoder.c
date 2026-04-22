/*
 * Encoder.c
 *
 *  Created on: 2026年2月8日
 *      Author: 19929
 */

#include "zf_common_headfile.h"
#include "isr_config.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

int16 encoder_data_dir_right = 0;
int16 encoder_data_dir_left = 0;
float encoder_right_speed;
float encoder_left_speed;
float encoder_right_loc_delta;
float encoder_left_loc_delta;
float encoder_right_loc;
float encoder_left_loc;

/*编码器初始化*/
void Encoder_Init(void)
{
    encoder_dir_init(ENCODER_DIR_Right, ENCODER_DIR_PULSE_Right, ENCODER_DIR_DIR_Right);          // 初始化右轮方向编码器
    encoder_dir_init(ENCODER_DIR_Left,ENCODER_DIR_PULSE_Left,ENCODER_DIR_DIR_Left);               // 初始化左轮方向编码器

    encoder_right_loc = 0;
    encoder_left_loc  = 0;
}

/*编码器计数获取   放进速度环PID调控内，2ms调用一次*/
void Encoder_Get(void)
{
    encoder_data_dir_right = -encoder_get_count(ENCODER_DIR_Right);                          // 获取右轮编码器计数
    encoder_data_dir_left = encoder_get_count(ENCODER_DIR_Left);                           // 获取左轮编码器计数
    encoder_clear_count(ENCODER_DIR_Right);                                                 // 清空编码器计数
    encoder_clear_count(ENCODER_DIR_Left);                                                  // 清空编码器计数

    /*
     * 速度计算方式:
     *  1024线编码器，转一圈产生1024个脉冲（方向编码器）
     *  车轮齿数和编码器齿数一致，轮毂齿数是车轮齿数的4倍
     *  所以编码器转4圈，车轮才转一圈，即4096个脉冲车轮转一圈
     *  每2ms获取一下编码器的计数，得到的就是 n个脉冲数/2ms
     *  encoder_data_dir_right / 2ms -->  得到 n个脉冲数/s
     *
     *  4096个脉冲数代表一转
     *  那么 encoder_data_dir_right / 2ms / 4096 即为 n转/s
     *
     * 位移获取方式：
     *     首先获取轮子的周长 ： C = 2*PI*R
     *     4096个脉冲数代表一转
     *     还是每2ms获取一下脉冲数，获取的同时清除脉冲计数，那么获取到的脉冲数就是这2ms产生的新脉冲数
     *     再用这个脉冲数差值/4096，得到的就是这2ms轮子转了多少圈（N圈）
     *     再把圈数乘以轮子周长，即 N*C 即可得到2ms车子前进的距离
     *     用一个变量累加这个距离，变量的最终值就是位移值
     *
     *
     *
     *     7.5  1024
     *     1ms 40
     *     25.6ms -> 7.5cm
     *     2.56 * 10-2 -> 7.5cm
     *     2.56 * 10-2 -> 7.5 * 10-2 m
     * */
}

/*2msPID调控周期下的转速，单位：n转/s*/
void Encoder_Get_Speed(void)
{
    encoder_right_speed = 1.0 * encoder_data_dir_right * 500 / 4096;
    encoder_left_speed = 1.0 * encoder_data_dir_left * 500 / 4096;
}

/*2msPID调控周期下的位移增量和总位移，单位：cm*/
void Encoder_Get_Location(void)
{
    // 获取轮子的周长
    float C = 2.0 * PI * 2.43 / 2;                                  // 轮子周长（单位：cm） 2πr
//    float C = 2.0 * PI * 2.43 / 2 / 100;                          // 轮子周长（单位：m）

    float N_left = encoder_data_dir_left * 1.0 / 4096;              // 2ms内轮子转了N圈
    float N_right = encoder_data_dir_right * 1.0 / 4096;

    encoder_right_loc_delta = 1.0 * N_right * C;                    // 得到2ms内位移增量
    encoder_left_loc_delta = 1.0 * N_left * C;
    encoder_right_loc += encoder_right_loc_delta;                   // 位移总量
    encoder_left_loc += encoder_left_loc_delta;
}

#pragma section all restore

