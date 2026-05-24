/*
 * Key.c
 *
 *  Created on: 2026年4月30日
 *      Author: 19929
 */


#include "Key.h"

#pragma section all "cpu0_dsram"


/* 按键从左往右依次是4321 */

void Key_Init(void)
{
    gpio_init(KEY1, GPI, 1, GPI_PULL_UP);       // 输入模式 初始高电平 上拉输入
    gpio_init(KEY2, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY3, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY4, GPI, 1, GPI_PULL_UP);
}

uint8_t Key_Get(void)
{
    uint8_t Keynum = 0;

    if(gpio_get_level(KEY1) == 0)               // 检测按键一
    {
        system_delay_ms(20);                    // 消抖
        while(gpio_get_level(KEY1) == 0);       // 等待松手
        Keynum = 1;
    }
    if(gpio_get_level(KEY2) == 0)               // 检测按键二
    {
        system_delay_ms(20);                    // 消抖
        while(gpio_get_level(KEY2) == 0);       // 等待松手
        Keynum = 2;
    }
    if(gpio_get_level(KEY3) == 0)               // 检测按键三
    {
        system_delay_ms(20);                    // 消抖
        while(gpio_get_level(KEY3) == 0);       // 等待松手
        Keynum = 3;
    }
    if(gpio_get_level(KEY4) == 0)               // 检测按键四
    {
        system_delay_ms(20);                    // 消抖
        while(gpio_get_level(KEY4) == 0);       // 等待松手
        Keynum = 4;
    }
    return Keynum;
}

/*
 *       // 按键检测测试
      switch(Key_Get())
      {
          case 1:{
          LED1_ON();LED2_OFF();LED3_OFF();LED4_OFF();}break;
          case 2:{
          LED1_OFF();LED2_ON();LED3_OFF();LED4_OFF();}break;
          case 3:{
          LED1_OFF();LED2_OFF();LED3_ON();LED4_OFF();}break;
          case 4:{
          LED1_OFF();LED2_OFF();LED3_OFF();LED4_ON();}break;
      }
 *
 * */










#pragma section all restore
