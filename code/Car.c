/*
 * Car.c
 *
 *  Created on: 2026年2月8日
 *      Author: 19929
 */

#include "zf_common_headfile.h"

void Car_Init(void)
{
    pwm_init(Right_PWM,17000,0);
    pwm_init(Left_PWM,17000,0);
    gpio_init(Left_Dir,GPO,0,GPO_PUSH_PULL);
    gpio_init(Right_Dir,GPO,0,GPO_PUSH_PULL);
    pwm_init(fuya_PWM,50,0);
}

void Car_go_forward(uint16_t Speed)
{
    gpio_set_level(Left_Dir,0);
    gpio_set_level(Right_Dir,1);
    pwm_set_duty(Right_PWM,Speed);
    pwm_set_duty(Left_PWM,Speed);
}

void Car_go_Back(uint16_t Speed)
{
    gpio_set_level(Left_Dir,1);
    gpio_set_level(Right_Dir,0);
    pwm_set_duty(Right_PWM,Speed);
    pwm_set_duty(Left_PWM,Speed);
}

// 左轮正转
void Left_Go_Forward(uint16 Speed)
{
    gpio_set_level(Left_Dir,0);
    pwm_set_duty(Left_PWM,Speed);
}

// 左轮反转
void Left_Go_Back(uint16 Speed)
{
    gpio_set_level(Left_Dir,1);
    pwm_set_duty(Left_PWM,Speed);
}

// 右轮正转
void Right_Go_Forward(uint16 Speed)
{
    gpio_set_level(Right_Dir,0);
    pwm_set_duty(Right_PWM,Speed);
}

// 右轮反转
void Right_Go_Back(uint16 Speed)
{
    gpio_set_level(Right_Dir,1);
    pwm_set_duty(Right_PWM,Speed);
}

/*
 * Speed的值在0到100之间
 * */
void Fuya_Speed(uint8 Speed)
{
    if(Speed >= 100)    Speed = 100;
    uint32 temp = 5*Speed+500;
    pwm_set_duty(fuya_PWM,temp);
}


/*------------------------加速减速测试------------------------*/
//int16 Duty = 0;
//uint8 Dir = 1;
//        if(Duty > 0)
//        {
//            Car_go_forward(Duty);
//        }
//        else if(Duty < 0)
//        {
//            Car_go_Back(-Duty);
//        }
//        oscilloscope_data.data[0] = encoder_data_dir_right;        // 显示右轮速度
//        oscilloscope_data.data[1] = encoder_data_dir_left;         // 显示左轮速度
//        oscilloscope_data.data[2] = Duty;                          // 显示占空比
//          if(Dir == 1)
//          {
//              Duty+=1;
//          }
//          else if(Dir == 0)
//          {
//              Duty-=1;
//          }
//
//          if(Duty >= 3000)
//          {
//              Dir = 0;
//          }
//          else if(Duty <= -3000)
//          {
//              Dir = 1;
//          }
/*------------------------加速减速测试------------------------*/



