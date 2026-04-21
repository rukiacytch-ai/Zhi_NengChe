/*
 * scan_line.c
 *
 *  Created on: 2026年3月12日
 *      Author: 赵先生
 */
#include "zf_common_headfile.h"
#include "scan_line.h"

 int Left_line[DST_H];//左 右侧边界
 int Right_line[DST_H];
 int Center_line[DST_H];

void scan_border(void)
{
    for (int i = 0; i < DST_H; i++)//数据清零
    {
        Left_line[i] = 2;
        Right_line[i] = DST_W - 3;
    }

    int  y, x;
    int  last_left, last_right, last_mid;
    int  find_left, find_right;
        // 最底行初始化
        y = DST_H - 1;
        find_left = 0;
        for(x = 0; x <= DST_W -1 - 2; x++)
        {
            if(image_press[y][x] == IMG_BLACK && image_press[y][x + 1] == IMG_WHITE && image_press[y][x + 2] == IMG_WHITE)
            {
                Left_line[y] = x ;
                find_left = 1;
                break;
            }
        }
        find_right = 0;
        for(x = DST_W - 1; x >= 2; x--)
        {
            if(image_press[y][x] ==IMG_BLACK && image_press[y][x - 1] == IMG_WHITE && image_press[y][x - 2] == IMG_WHITE)
            {
                Right_line[y] = x - 1;
                find_right = 1;
                break;
            }
        }

        Center_line[y] = (Left_line[y] + Right_line[y]) >> 1;
        last_left  = Left_line[y];
        last_right = Right_line[y];
        last_mid   = Center_line[y];

        // 从下往上逐行
        for(int j = DST_H - 2; j >= 0; j--)
        {
            //找左边界：从 left_start 向右找
            find_left = 0;
            for(int i = last_mid; i >= 2; i--)
            {

                if(image_press[j][i] ==IMG_WHITE && image_press[j][i - 1] == IMG_BLACK && image_press[j][i - 2] == IMG_BLACK)
                {
                    Left_line[j] = i;
                    find_left = 1;
                    break;
                }
            }
            // 左边没找到 → 直接置最左
            if(find_left == 0)
                Left_line[j] = 0;

            // 找右边界：从 right_start 向左找
            find_right = 0;
            for(int i = last_mid; i <= DST_W -1 -2; i++)
            {

                if(image_press[j][i] ==IMG_WHITE && image_press[j][i + 1] == IMG_BLACK && image_press[j][i + 2] == IMG_BLACK)
                {
                    Right_line[j] = i;
                    find_right = 1;
                    break;
                }
            }
            // 右边没找到 → 直接置最右
            if(find_right == 0)
                Right_line[j] = DST_W - 1;

            // 计算中线
            Center_line[j] = (Left_line[j] + Right_line[j]) >> 1;

            last_left  = Left_line[j];
            last_right = Right_line[j];
            last_mid   = Center_line[j];
        }


}

