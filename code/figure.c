/*
 * figure.c
 *
 *  Created on: 2026年1月25日
 *      Author: 赵先生
 */

#include "zf_common_headfile.h"
#include "figure.h"


#define CROP_START_ROW  30
#define CROP_START_COL  24

uint8 image_press[DST_H][DST_W] = {{0}} ;// 原始图像数组 + 裁剪后目标数组
uint8 target[DST_H][DST_W];
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     压缩原图
// 参数说明     void
// 返回参数     void
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Image_Compress(void)                                                       //图像压缩函数
{
  for(uint8 i = 0;i < 60;i++)
    {
        for(uint8 j = 0;j < 94;j++)
        {
            image_press[i][j] = mt9v03x_image[2 * i][2 * j];
        }
    }
}

// 图像中心裁剪函数
void image_crop_center(void)
{
    int pixel_size = sizeof(uint8);     // 单个像素字节数，通用适配
    int copy_len = DST_W * pixel_size;      // 每行拷贝的总字节数

    for (int i = 0; i < DST_H; i++)         // 逐行拷贝：原始图像[30~89行][24~163列] → image_press[0~59行][0~139列]
    {

        const uint8 *src_addr = &mt9v03x_image[CROP_START_ROW + i][CROP_START_COL];
        uint8 *dst_addr = image_press[i];
        memcpy(dst_addr, src_addr, copy_len);
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     普通大津求阈值
  @param     image       图像数组
             width       列 ，宽度
             height      行，长度
  @return    threshold   返回int类型的的阈值
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//普通大津
  @note      据说没有山威大津快，我感觉两个区别不大
-------------------------------------------------------------------------------------------------------------------*/
int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height)   //大津算法，注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int  threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    for (i = 0; i < height; i+=2)//统计灰度级中每个像素在整幅图像中的个数
    {
        for (j = 0; j <width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    for (i = 0; i < GrayScale; i++) //计算每个像素值的点在整幅图像中的比例
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)//遍历灰度级[0,255]
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);//平方
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;//最大类间方差法
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    if(threshold>255)
        threshold=255;
    if(threshold<0)
        threshold=0;
  return threshold;
}

void Image_Binarization(void)//图像二值化
{

    image_crop_center();
    int threshold = otsuThreshold( image_press[0], DST_W, DST_H);
    uint16 i,j;
    for(i=0;i<60;i++)
    {
        for(j=0;j<140;j++)//灰度图的数据只做判断，不进行更改，二值化图像放在了新数组中
        {
            if( image_press[i][j] >= threshold)
                 image_press[i][j] = IMG_WHITE;//白
            else
                 image_press[i][j] = IMG_BLACK;//黑
        }
    }
    Filtration_Sim();
}

void lq_sobel(uint8 imageIn[DST_H][DST_W], uint8 imageOut[DST_H][DST_W],int threshold)
{

  int KERNEL_SIZE = 3;
  int xStart = KERNEL_SIZE/2;
  int xEnd   = DST_W - KERNEL_SIZE/2;
  int yStart = KERNEL_SIZE/2;
  int yEnd   = DST_H - KERNEL_SIZE/2;
  int i, j;
  int temp[2];
  for(i = yStart; i < yEnd; i++)
  {
    for(j = xStart; j < xEnd; j++)
    {

      temp[0] = -(int)imageIn[i-1][j-1] + (int)imageIn[i-1][j+1]     //{{-1, 0, 1},
        -(int)imageIn[i][j-1]   +(int)imageIn[i][j+1]        // {-1, 0, 1},
          -(int)imageIn[i+1][j-1] + (int)imageIn[i+1][j+1];    // {-1, 0, 1}};

      temp[1] = -(int)imageIn[i-1][j-1] + (int)imageIn[i+1][j-1]     //{{-1, -1, -1},
        -(int)imageIn[i-1][j]   + (int)imageIn[i+1][j]       // { 0,  0,  0},
          -(int)imageIn[i-1][j+1] + (int)imageIn[i+1][j+1];    // { 1,  1,  1}};


//      temp[2] = -(int)imageIn[i-1][j]   + (int)imageIn[i][j-1]       //  0, -1, -1
//        -(int)imageIn[i][j+1]   + (int)imageIn[i+1][j]       //  1,  0, -1
//          -(int)imageIn[i-1][j+1] + (int)imageIn[i+1][j-1];    //  1,  1,  0
//
//      temp[3] = -(int)imageIn[i-1][j]   + (int)imageIn[i][j+1]       // -1, -1,  0
//        -(int)imageIn[i][j-1]   + (int)imageIn[i+1][j]       // -1,  0,  1
//          -(int)imageIn[i-1][j-1] + (int)imageIn[i+1][j+1];    //  0,  1,  1

      temp[0] = abs(temp[0]);
      temp[1] = abs(temp[1]);
//      temp[2] = abs(temp[2]);
//      temp[3] = abs(temp[3]);


        if(temp[0] < temp[1])
        {
          temp[0] = temp[1];
        }


      /* 使用像素点邻域内像素点之和的一定比例作为阈值  */
//          temp[3] =
//                  (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
//                  + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
//                  + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];


      if(temp[0] > threshold)
      {
        imageOut[i][j] = IMG_WHITE;
      }
      else
      {
        imageOut[i][j] = IMG_BLACK;
      }
    }
  }
}

void lv_po(uint8 imageout[DST_H][DST_W])
{
    for(int i = 1 ; i < 59 ; i++)
    {
        for(int j = 1 ; j < 139 ; j++)
        {
            if(imageout[i][j] == IMG_WHITE && imageout[i + 1][j] == IMG_BLACK && imageout[i - 1][j] == IMG_BLACK &&imageout[i][j + 1] == IMG_BLACK &&imageout[i][j + 1] == IMG_BLACK)
            {
                imageout[i][j] = IMG_BLACK;
            }
        }
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     大津法求阈值
// 参数说明     uint8 *image 需要处理的灰度图 uint16 width 图像宽度 uint16 height 图像高度
// 返回参数     void
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height)
{
    #define GrayScale 256
    int pixelCount[GrayScale] = {0};                                            //每个灰度值所占像素个数
    float pixelPro[GrayScale] = {0};                                            //每个灰度值所占总像素比例
    int i,j;
    int Sumpix = width * height;                                                //总像素点
    uint8 threshold = 0;
    uint8* data = image;                                                        //指向像素数据的指针


    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;                             //将像素值作为计数数组的下标
        }
    }
    float u = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / Sumpix;                            //计算每个像素在整幅图像中的比例
        u += i * pixelPro[i];                                                   //总平均灰度
    }


    float maxVariance=0.0;                                                      //最大类间方差
    float w0 = 0, avgValue  = 0;                                                //w0 前景比例 ，avgValue 前景平均灰度
    for(int i = 0; i < 256; i++)                                                //每一次循环都是一次完整类间方差计算
    {
        w0 += pixelPro[i];                                                      //假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
        avgValue  += i * pixelPro[i];

        float variance = pow((avgValue/w0 - u), 2) * w0 /(1 - w0);              //类间方差
        if(variance > maxVariance)
        {
            maxVariance = variance;
            threshold = (uint8)i;
        }
    }


    return threshold;

}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     过滤椒盐噪点
// 参数说明     void
// 返回参数     void
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8_t Temp_Image[DST_H][DST_W];
void Filtration_Sim(void)
{

  for(uint8_t y = 0; y < DST_H; y++)
  {
    for(uint8_t x = 0; x < DST_W; x++)
    {
      Temp_Image[y][x] = image_press[y][x];
    }
  }


  for(uint8_t y = 1; y < DST_H - 1; y++)
  {
    for(uint8_t x = 1; x < DST_W - 1; x++)
    {
      // 计算3x3邻域总和（所有下标均在0~59/0~139范围内，无越界）
      uint16_t Temp_Num = image_press[y-1][x-1] + image_press[y-1][x] + image_press[y-1][x+1] +
                          image_press[y][x-1]   + image_press[y][x]   + image_press[y][x+1] +
                          image_press[y+1][x-1] + image_press[y+1][x] + image_press[y+1][x+1];


      if(Temp_Num >= 255 * 5)
      {
        Temp_Image[y][x] = 255;
      }
      else
      {
        Temp_Image[y][x] = 0;
      }
    }
  }


  for(uint8_t y = 1; y < DST_H - 1; y++)
  {
    for(uint8_t x = 1; x < DST_W - 1; x++)
    {
      image_press[y][x] = Temp_Image[y][x];
    }
  }
}
