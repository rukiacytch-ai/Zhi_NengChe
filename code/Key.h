/*
 * Key.h
 *
 *  Created on: 2026年4月30日
 *      Author: 19929
 */

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

#include "zf_common_headfile.h"

#define KEY1    (P22_0)     // 按键一
#define KEY2    (P22_1)     // 按键二
#define KEY3    (P22_2)     // 按键三
#define KEY4    (P22_3)     // 按键四

// 按键按下后，电平从高变低

// 目前板载的按键，从左往右分别是按键一到按键三

void Key_Init(void);
uint8_t Key_Get(void);


#endif /* CODE_KEY_H_ */
