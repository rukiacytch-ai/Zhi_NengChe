/*
 * scan_line.h
 *
 *  Created on: 2026年3月12日
 *      Author: 赵先生
 */

#ifndef CODE_SCAN_LINE_H_
#define CODE_SCAN_LINE_H_
#include "zf_common_headfile.h"

extern  int Left_line[DST_H];//左 右侧边界
extern  int Right_line[DST_H];
extern  int Center_line[DST_H];
extern  uint8 Left_line_found[DST_H];//左边线是否真实扫到，1=扫到，0=丢线/补线
extern  uint8 Right_line_found[DST_H];//右边线是否真实扫到，1=扫到，0=丢线/补线

void scan_border(void);

#endif /* CODE_SCAN_LINE_H_ */
