/*
 * scan_line.c
 *
 *  Created on: 2026年3月12日
 *      Author: 赵先生
 */
#include "zf_common_headfile.h"

int Left_line[DST_H];//左 右侧边界
int Right_line[DST_H];
int Center_line[DST_H];
uint8 Left_line_found[DST_H];//左边线是否真实扫到，1=扫到，0=丢线/补线
uint8 Right_line_found[DST_H];//右边线是否真实扫到，1=扫到，0=丢线/补线
uint8  up_inflection_found = 0;
#define BORDER_SEARCH_OFFSET   70      // 后续行以上一行边线为基准，左右各偏移这个范围作为扫线窗口
#define BORDER_MIN_X           2       // 扫边线时至少保留2个像素，方便判断连续跳变
#define BORDER_MAX_X           (DST_W - 3)
#define BORDER_MIN_WHITE_WIDTH 10
#define BORDER_DEFAULT_WIDTH   40
#define BORDER_LOST_TO_SIDE_TIME 3     // 连续丢线达到这个行数后，将丢线侧边界压到最左或最右
#define center  70



// 将横坐标限制在图像有效范围内，避免扫线访问越界
static int border_limit_x(int x)
{
    if(x < BORDER_MIN_X)
        return BORDER_MIN_X;
    if(x > BORDER_MAX_X)
        return BORDER_MAX_X;
    return x;
}

// 从指定行整行寻找最长连续白色块，用它作为当前行最可信的赛道区域
// 返回1表示白色块宽度有效，返回0表示本行白色块过窄，需要后续用默认宽度兜底
static int find_bottom_white_block(int row, int *left, int *right)
{
    int best_left = 2;
    int best_right = DST_W - 3;
    int best_width = 0;
    int in_white = 0;
    int start = 0;

    for(int x = BORDER_MIN_X; x <= BORDER_MAX_X; x++)
    {
        // 遇到白点且当前不在白色块内，记录这一段白色块的起点
        if(image_press[row][x] == IMG_WHITE && in_white == 0)
        {
            start = x;
            in_white = 1;
        }

        // 遇到黑点或行尾时，说明当前白色块结束，比较并保留最长的一段
        if((image_press[row][x] == IMG_BLACK || x == BORDER_MAX_X) && in_white)
        {
            int end = (image_press[row][x] == IMG_BLACK) ? (x - 1) : x;
            int width = end - start + 1;

            if(width > best_width)
            {
                best_width = width;
                best_left = start;
                best_right = end;
            }

            in_white = 0;
        }
    }

    *left = best_left;
    *right = best_right;
    return best_width >= BORDER_MIN_WHITE_WIDTH;
}

// 在上一行左边线附近窗口内寻找“黑 黑 白”跳变点，避免车身斜时全图误扫
static int find_left_near_last(int row, int last_left)
{
    int start = border_limit_x(last_left - BORDER_SEARCH_OFFSET);
    int end = border_limit_x(last_left + BORDER_SEARCH_OFFSET);

    for(int i = start; i <= end; i++)
    {
        if(image_press[row][i] == IMG_WHITE && image_press[row][i - 1] == IMG_BLACK && image_press[row][i - 2] == IMG_BLACK)
            return i;
    }

    return -1;
}

// 在上一行右边线附近窗口内寻找“白 黑 黑”跳变点，保持右边线独立连续跟踪
static int find_right_near_last(int row, int last_right)
{
    int start = border_limit_x(last_right + BORDER_SEARCH_OFFSET);
    int end = border_limit_x(last_right - BORDER_SEARCH_OFFSET);

    for(int i = start; i >= end; i--)
    {
        if(image_press[row][i] == IMG_WHITE && image_press[row][i + 1] == IMG_BLACK && image_press[row][i + 2] == IMG_BLACK)
            return i;
    }

    return -1;
}

/**
 * @brief  根据二值化图像扫出左右边线和中线
 * @note   扫线思路：
 *         1. 最底行先全行寻找赛道白色区域，得到第一组左右边界；
 *         2. 后续每一行分别使用上一行扫出的左右边线作为参考；
 *         3. 左边界在上一行左边线附近窗口内扫描；
 *         4. 右边界在上一行右边线附近窗口内扫描；
 *         5. 如果某一侧没有扫到边界，则先按照上一行赛道宽度补线，连续丢线后再压到图像边界。
 */
void scan_border(void)
{
    for (int i = 0; i < DST_H; i++)//数据清零
    {
        Left_line[i] = 2;
        Right_line[i] = DST_W - 3;
        Left_line_found[i] = 0;
        Right_line_found[i] = 0;
    }

    int  y;
    int  last_left, last_right, last_width;
    int  left_lost_cnt, right_lost_cnt;
    uint8  find_left, find_right;

        // 最底行初始化：从图像整行寻找赛道白色区域
        y = DST_H - 1;
        if(find_bottom_white_block(y, &Left_line[y], &Right_line[y]))
        {
            find_left = 1;
            find_right = 1;
        }
        else
        {
            find_left = 0;
            find_right = 0;
        }

        // found数组只记录“这一行是否真的扫到边线”，后面即使补线也不改这个标志
        Left_line_found[y] = find_left;
        Right_line_found[y] = find_right;

        Center_line[y] = (Left_line[y] + Right_line[y]) >> 1;
        last_left  = Left_line[y];
        last_right = Right_line[y];

        if(find_left && find_right)
            last_width = last_right - last_left;
        else
            last_width = BORDER_DEFAULT_WIDTH;
        if(last_width < BORDER_MIN_WHITE_WIDTH)
            last_width = BORDER_DEFAULT_WIDTH;
        left_lost_cnt = (find_left == 0);
        right_lost_cnt = (find_right == 0);

        // 从下往上逐行
        for(int j = DST_H - 2; j >= 0; j--)
        {
            // 以后续每一行的上一行左右边线为参考，在边线附近窗口内分别继续扫线
            int left_result = find_left_near_last(j, last_left);
            int right_result = find_right_near_last(j, last_right);
            int next_left = last_left;
            int next_right = last_right;

            //找左边界：在上一行左边线附近搜索，车身斜着进入时边线也能连续跟踪
            find_left = (left_result >= 0);
            if(find_left)
            {
                Left_line[j] = left_result;
                next_left = left_result;
            }

            // 找右边界：在上一行右边线附近搜索，避免只依赖中线造成两侧窗口一起偏移
            find_right = (right_result >= 0);
            if(find_right)
            {
                Right_line[j] = right_result;
                next_right = right_result;
            }

            if(find_left)
                left_lost_cnt = 0;
            else if(left_lost_cnt < BORDER_LOST_TO_SIDE_TIME)
                left_lost_cnt++;

            if(find_right)
                right_lost_cnt = 0;
            else if(right_lost_cnt < BORDER_LOST_TO_SIDE_TIME)
                right_lost_cnt++;

            // 左边没找到 → 短暂丢线先用右边线减去上一行宽度补线，连续丢线后压到最左边
            // 注意：这里虽然用上一行边线补起来用于循迹，但 Left_line_found[j] 仍然是0
            // 这样图1那种左边开口/左边丢线不会被补线“抹平”，节点判断仍能识别到丢线突变
            if(find_left == 0 && find_right)
                next_left = border_limit_x(Right_line[j] - last_width);

            if(find_left == 0 && left_lost_cnt >= BORDER_LOST_TO_SIDE_TIME)
                Left_line[j] = BORDER_MIN_X;
            else if(find_left == 0 && find_right)
                Left_line[j] = next_left;
            else if(find_left == 0)
                Left_line[j] = last_left;

            // 右边没找到 → 短暂丢线先用左边线加上上一行宽度补线，连续丢线后压到最右边
            // 同理，右边线坐标可以补，但 Right_line_found[j] 用来保留真实丢线信息

            if(find_right == 0 && find_left)
                next_right = border_limit_x(Left_line[j] + last_width);

            if(find_right == 0 && right_lost_cnt >= BORDER_LOST_TO_SIDE_TIME)
                Right_line[j] = BORDER_MAX_X;
            else if(find_right == 0 && find_left)
                Right_line[j] = next_right;
            else if(find_right == 0)
                Right_line[j] = last_right;

            Left_line_found[j] = find_left;
            Right_line_found[j] = find_right;

            // 计算中线
            Center_line[j] = (Left_line[j] + Right_line[j]) >> 1;

            // 下一行搜索参考仍使用真实/宽度预测边线，避免压边后搜索窗口锁死在图像边缘
            last_left  = next_left;
            last_right = next_right;
            if(find_left && find_right)
            {
                last_width = last_right - last_left;
                if(last_width < BORDER_MIN_WHITE_WIDTH)
                    last_width = BORDER_DEFAULT_WIDTH;
            }
        }
}

float Err_Sum(int start,int end)
{
    int i;
    float err = 0;
    int cnt = 0;
    for(i = start; i <= end; i++)
    {

        if(Left_line[i] > 0 && Right_line[i] > 0) // 边界有效判断
        {
            int mid = Center_line[i];
            // 下方行权重更高（例：i越大越靠下，权重从1到2线性增加）
            float weight = 1.0f;
            if(end != start)
                weight = 1.0f + (float)(i - start) / (end - start);
            err += weight * (center - mid);
            cnt++;
        }
    }
    if(cnt > 0) err /= cnt; // 按有效行数求平均
    return err;
}
