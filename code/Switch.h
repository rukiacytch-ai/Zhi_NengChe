/*
 * Switch.h
 *
 *  Created on: 2026年3月13日
 *      Author: 19929
 */

#ifndef CODE_SWITCH_H_
#define CODE_SWITCH_H_

#define SWITCH1                 (P33_13)        // 开关1
#define SWITCH2                 (P33_12)        // 开关2

void Switch_Init(void);
uint8 Switch1_Get(void);
uint8 Switch2_Get(void);

#endif /* CODE_SWITCH_H_ */
