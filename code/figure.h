/*
 * figure.h
 *
 *  Created on: 2026年1月25日
 *      Author: 赵先生
 */

#ifndef CODE_FIGURE_H_
#define CODE_FIGURE_H_
#define DST_H         60  // 目标图像高度（行）
#define DST_W         140 // 目标图像宽度（列）
#define IMG_BLACK     0X00      //0x00是黑
#define IMG_WHITE     0Xff      //0xff为白
extern uint8 image_press[DST_H][DST_W];// 原始图像数组 + 裁剪后目标数组
void Image_Compress(void);  //图像压缩函数
extern uint8 target[DST_H][DST_W];
void image_crop_center(void);
int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height);
void Image_Binarization(void);
void lq_sobel(uint8 imageIn[DST_H][DST_W], uint8 imageOut[DST_H][DST_W],int threshold);
void lv_po(uint8 imageout[DST_H][DST_W]);
uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height);
void Filtration_Sim(void);


#endif /* CODE_FIGURE_H_ */
