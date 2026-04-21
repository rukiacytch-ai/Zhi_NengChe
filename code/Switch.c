/*
 * Switch.c
 *
 *  Created on: 2026年3月13日
 *      Author: 19929
 */

#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

void Switch_Init(void)
{
    gpio_init(SWITCH1,GPI,1,GPI_PULL_UP);    // SWITCH1初始化为GPIO功能，输入模式，默认高电平，上拉输入
    gpio_init(SWITCH2,GPI,1,GPI_PULL_UP);    // SWITCH2初始化为GPIO功能，输入模式，默认高电平，上拉输入
}

/* 一号开关，ON是低电平，OFF是高电平
 * 返回0为关，返回1为开
 * */
uint8 Switch1_Get(void)
{
    uint8 Switch_Num = 0;                                   // 默认关闭
    if(gpio_get_level(SWITCH1) == 0)    Switch_Num = 1;     // 如果检测到开关为低电平，则设置为打开
    return Switch_Num;
}

/* 二号开关，ON是低电平，OFF是高电平
 * 返回0为关，返回1为开
 * */
uint8 Switch2_Get(void)
{
    uint8 Switch_Num = 0;                                   // 默认关闭
    if(gpio_get_level(SWITCH2) == 0)    Switch_Num = 1;     // 如果检测到开关为低电平，则设置为打开
    return Switch_Num;
}


#pragma section all restore






