/*
 * LED.c
 *
 *  Created on: 2026年3月13日
 *      Author: 19929
 */

#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

void LED_Init(void)
{
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);              // 初始化 LED1 输出 默认高电平 推挽输出模式
    gpio_init(LED2, GPO, GPIO_HIGH, GPO_PUSH_PULL);              // 初始化 LED1 输出 默认高电平 推挽输出模式
    gpio_init(LED3, GPO, GPIO_HIGH, GPO_PUSH_PULL);              // 初始化 LED1 输出 默认高电平 推挽输出模式
    gpio_init(LED4, GPO, GPIO_HIGH, GPO_PUSH_PULL);              // 初始化 LED1 输出 默认高电平 推挽输出模式
}

void LED1_ON(void){gpio_set_level(LED1,0);}
void LED1_OFF(void){gpio_set_level(LED1,1);}
void LED2_ON(void){gpio_set_level(LED2,0);}
void LED2_OFF(void){gpio_set_level(LED2,1);}
void LED3_ON(void){gpio_set_level(LED3,0);}
void LED3_OFF(void){gpio_set_level(LED3,1);}
void LED4_ON(void){gpio_set_level(LED4,0);}
void LED4_OFF(void){gpio_set_level(LED4,1);}



#pragma section all restore

