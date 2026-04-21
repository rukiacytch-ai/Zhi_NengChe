/*
 * scan_line.h
 *
 *  Created on: 2026年3月12日
 *      Author: 赵先生
 */

#ifndef CODE_SCAN_LINE_H_
#define CODE_SCAN_LINE_H_

extern  int Left_line[DST_H];//左 右侧边界
extern  int Right_line[DST_H];
extern  int Center_line[DST_H];

void scan_border(void);
void scan_border_2(void);

#endif /* CODE_SCAN_LINE_H_ */
