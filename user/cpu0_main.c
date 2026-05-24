/*
 * 5/12 后续优化思路：
 * 1.先新板子+负压完赛
 * 2.模糊pid
 * 3.卡尔曼滤波预测
 * 4.光电管结合，消除漂移
 * 5.无刷
 * */

#include "isr_config.h"
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 在使用 #pragma section all restore 之前的全局变量都会放入 CPU0 的 RAM 中

/* 定时器中断宏定义 */
#define PIT_NUM (CCU61_CH0) // 使用的定时器中断编号

/*------------------------陀螺仪参数------------------------*/
/* 一阶低通滤波系数，也就是当前数据占滤波结果的 0.2 权重 */
#define Alpha (0.2f)
// 原始 z 轴角速度
float Gyro_z = 0.0f;
/* 一阶低通滤波后的 z 轴角速度 */
float Filtered_Gyro_z;
/* 卡尔曼滤波后的 Yaw */
float KalMan_Yaw = 0;

/*------------------------陀螺仪参数------------------------*/

/*------------------------ PID 参数变量------------------------*/
/* PID 速度环输出限幅 */
#define MAX_SPEED (4000)

/* 左轮速度环 PID */
PID_t SpeedPID_L =
{
//    .Kp = 31.8,
//    .Ki = 5.8,
//    .Kd = 0.0,

     .Kp = 150.0,
     .Ki = 4.0,
     .Kd = 8.0,

     .OutMax = MAX_SPEED,
     .OutMin = -MAX_SPEED,
};

/* 右轮速度环 PID */
PID_t SpeedPID_R =
{
     .Kp = 150.0,
     .Ki = 4.0,
     .Kd = 8.0,

     .OutMax = MAX_SPEED,
     .OutMin = -MAX_SPEED,
};

/* 角度环 PID */
PID_t AnglePID =
{
    .Kp = 0.14,
    .Kd = 0.10,
    .GKD = 0.0008,
    .KP2 = 0.0005,
    .Ki = 0.00,

    //    .Kp = 0.1,
    //    .Kd = 2,

    .OutMax = 30,
    .OutMin = -30,
};

///* 左轮速度环 PID */
//PID_t SpeedPID_L = {
////    .Kp = 31.8,
////    .Ki = 5.8,
////    .Kd = 0.0,
//
//    .Kp = 80.0,
//    .Ki = 8.0,
//    .Kd = 0.0,
//
//    .OutMax = MAX_SPEED,
//    .OutMin = -MAX_SPEED,
//};
//
///* 右轮速度环 PID */
//PID_t SpeedPID_R = {
//    .Kp = 80.0,
//    .Ki = 8.0,
//    .Kd = 0.0,
//
//    .OutMax = MAX_SPEED,
//    .OutMin = -MAX_SPEED,
//};
//
///* 角度环 PID */
//PID_t AnglePID = {
//    .Kp = 0.11,
//    .Kd = 0.021,
//    .GKD = 0.05,
//    .KP2 = 0.001,
//
//    //    .Kp = 0.1,
//    //    .Kd = 2,
//
//    .OutMax = 50,
//    .OutMin = -50,
//};

// 初始化 0.5s 后，读取当前角度值作为目标角度值
float Target_init_angle = 0.0f;
bool is_waiting_done = false;    // 保证 count6 只走一次
bool is_init_angle_done = false; // 读取初始角度值标志位
bool is_set_once = false;        // 保证初始角度只被设置一次

/* 速度目标值 */
float Target_AveSpeed = 0;
/* 角度目标值 */
float Target_Angle = 0;
/* 输出 PWM 的平衡 PWM 以及转向 PWM */
float LeftSpeed, RightSpeed;                      // 左右轮实际速度值
float AveSpeed, DifSpeed_Actual, DifSpeed_Target; // 平均速度和差速速度值
/* PID 开启标志位 */
bool PID_Flag = false;
/* 角度偏差值值 */
double error_angle = 0;

bool is_use_fuya = false; // 是否使用负压标志位

bool high_duty_time = false; // 上电占空比变长时间

/*------------------------PID 参数变量------------------------*/

/*------------------------惯导算法变量------------------------*/
uint8 is_record_for_tft = 0;        // tft屏幕显示是否处于记录状态
uint8 is_replay_for_tft = 0;        // tft屏幕显示是否处于复现状态

bool is_clear_loc = false;      // 是否需要清空位置
bool has_reached_point = false; // 记录是否已经到达了当前点
uint16 count5 = 0;              // 复现模式开启后，等待 n s 再出发

// 比赛调试参数集中放在这里，避免临场改逻辑代码
#define INIT_YAW_LOCK_MS (1500U)                 // 上电后等待 IMU 稳定再锁定初始 yaw
#define REPLAY_WAIT_MS (3000U)                   // 复现按键按下后的静止等待时间
#define REPLAY_FUYA_SPEED (0)                  // 复现时负压占空比，0 表示关闭
#define REPLAY_START_YAW_CHECK_ENABLE (1)        // IMU 偶发 -178/178 跳变，默认不拦截起跑
#define REPLAY_START_YAW_TOLERANCE (4.0f)        // 起跑 yaw 允许偏差，单位：度
#define TFT_DEBUG_ENABLE (0)                     // 1 在 TFT 上显示比赛调试量

#define Get_Dot_Loc (2)                 // 打点距离 2cm
#define ROUTE_FLASH_PAGE_COUNT (12U)    // 路线存储使用 DFlash 的 12 页
#define ROUTE_FLASH_POINTS_PER_PAGE (1000U) // 每页第 0 位存数量，后面最多存 1000 个 yaw 点
#define Buffer_Max_Num          (8000) // 12页，每一页 1024 个 uint32 类型数据，存储 8000 个航向角
uint32 Record_Index = 0;               // 记录模式的索引，也需要存储到 Flash 中
float My_Flash_Buffer[Buffer_Max_Num]; // 存放打点数据
bool is_recording = false;             // 记录模式，默认关闭
uint8 page = 0;                        // 当前使用的 flash 页
uint32 Save_To_Buffer_Index = 0;       // 存入 flash 的索引
float Car_Go_Location = 0;             // 小车前进的位置

static float Normalize_Yaw_0_360(float yaw)
{
    while (yaw < 0.0f)
    {
        yaw += 360.0f;
    }
    while (yaw >= 360.0f)
    {
        yaw -= 360.0f;
    }

    return yaw;
}

static float Get_Normalized_Angle_Error(float actual, float target)
{
    float diff = actual - target;

    while (diff > 180.0f)
    {
        diff -= 360.0f;
    }
    while (diff < -180.0f)
    {
        diff += 360.0f;
    }

    return diff;
}


void Save_Data_To_Flash(void) // 存储数据到 flash 函数
{
    Save_To_Buffer_Index = 0; // 清空当前存储的角度打点
    uint32 remain_points = Record_Index;
    uint16 temp_all_points = (uint16)Record_Index;

    // 每次保存新路线前先擦完整个路线区，避免短路线接上旧路线尾巴
    for (uint8 i = 0; i < ROUTE_FLASH_PAGE_COUNT; i++)
    {
        if (flash_check(0, i) == 1)
        {
            flash_erase_page(0, i);
        }
    }

    // 0. 先检查一下缓存数组里有没有点需要存入 flash
    if (Record_Index == 0)
    {
        printf("没有记录到点，已清除旧数据\r\n");
        return;
    }

    // 1. 首先，计算需要存储几页。整 1000 点不能额外写一页空数据。
    uint8 To_Save_Page_Count = (uint8)((remain_points + ROUTE_FLASH_POINTS_PER_PAGE - 1U) / ROUTE_FLASH_POINTS_PER_PAGE);
    if (To_Save_Page_Count > ROUTE_FLASH_PAGE_COUNT)
    {
        To_Save_Page_Count = ROUTE_FLASH_PAGE_COUNT;
    }
    flash_buffer_clear();

    // 2. 开始存储数据
    for (uint8 i = 0; i < To_Save_Page_Count; i++)
    {
        uint16 Save_Temp_Count = (uint16)(remain_points > ROUTE_FLASH_POINTS_PER_PAGE ? ROUTE_FLASH_POINTS_PER_PAGE : remain_points);
        flash_union_buffer[0].uint16_type = Save_Temp_Count; // 每一页要存多少个点放在每一页的第 0 位
        // 存入角度数据，每一页最大存放 1000 个角度
        for (uint16 j = 1; j <= Save_Temp_Count; j++)
        {
            flash_union_buffer[j].float_type = Normalize_Yaw_0_360(My_Flash_Buffer[Save_To_Buffer_Index]); // 从第 1 个位置开始存 yaw 角
            Save_To_Buffer_Index++;
        }
        flash_write_page_from_buffer(0, i); // 将数据写入 flash 的第 i 页
        flash_buffer_clear();               // 然后清空一下缓存区准备下一次
        remain_points -= Save_Temp_Count;   // 更新剩余要存储的点数
    }
    Record_Index = temp_all_points;
    printf("共存储了 %d 个点，已清除旧路线尾巴\r\n", temp_all_points);
}

bool is_replaying = false;       // 复现模式，默认关闭
bool is_replay_only_once = true; // 保证打开开关，只复现一次
uint32 curr_point = 0;           // 当前打点序号
#define Replay_Speed (10)        // 复现时基础前进速度
#define K_Turn_Decel  (0.8f)     // 弯道减速系数，越大入弯越保守
#define Min_Turn_Speed (9.0f)    // 转弯最低速度下限

uint32 pre_look_point = 0;       // 前瞻目标点
#define PRE_LOOK_COUNT (0)       // 转向前瞻点数，0 表示跟当前点
#define DECEL_LOOK_AHEAD (0)     // 减速前瞻点数，0 表示关闭前方弯道扫描
#define FUTURE_DECEL_K (0.8f)    // 前瞻减速衰减系数

float Future_Turn_Angle = 0.0f; // 前瞻弯道幅度
bool wait_for_a_while = false;

static void Stop_Replay_Safely(void)
{
    // 复现退出统一收口，避免漏关 PID、负压或电机输出
    is_replay_for_tft = 0;
    Fuya_Speed(0);
    is_use_fuya = false;
    LED1_ON();
    LED2_ON();
    LED3_ON();
    LED4_ON();
    is_replaying = false;
    PID_Flag = false;
    DifSpeed_Target = 0;
    Target_AveSpeed = 0;
    SpeedPID_L.Out = 0;
    SpeedPID_R.Out = 0;
    Stop_Car();
}

void Read_the_flash(void) // 读取 flash 数据
{
    Save_To_Buffer_Index = 0;

    // 从第 0 页顺序读取，遇到第一张空页就停止，避免把旧路线后半段接进来
    for (uint8 i = 0; i < ROUTE_FLASH_PAGE_COUNT; i++)
    {
        if (flash_check(0, i) == 0)
        {
            break;
        }

        flash_read_page_to_buffer(0, i); // 把第 i 页的 flash 数据读入全局变量缓存
        uint16 To_Save_Count_This_Paper = flash_union_buffer[0].uint16_type; // 读取这一页中有多少个数据
        if (To_Save_Count_This_Paper == 0 || To_Save_Count_This_Paper > ROUTE_FLASH_POINTS_PER_PAGE)
        {
            printf("Flash 第 %d 页点数异常: %d，停止读取\r\n", i, To_Save_Count_This_Paper);
            break;
        }

        for (uint16 j = 1; j <= To_Save_Count_This_Paper && Save_To_Buffer_Index < Buffer_Max_Num; j++)
        {
            My_Flash_Buffer[Save_To_Buffer_Index] =
                    Normalize_Yaw_0_360(flash_union_buffer[j].float_type); // 兼容旧数据里的负角度
            Save_To_Buffer_Index++;
        }
    }
    printf("复现模式读取了 %d 个打点数据\r\n", (int)Save_To_Buffer_Index);
}

void Replay_the_path(void) // 路径复现函数
{
    // 检查是否追完了所有的点
    if (curr_point >= Save_To_Buffer_Index)
    {
        Stop_Replay_Safely();
        printf("复现结束!\r\n");
        return;
    }
    else
    {
        LED4_ON();
        LED2_ON();
        Target_AveSpeed = Replay_Speed;               // 设置复现速度
        pre_look_point = curr_point + PRE_LOOK_COUNT; // 寻找前瞻目标点
        if (pre_look_point >= Save_To_Buffer_Index)   // 前瞻目标边界保护
        {
            pre_look_point = Save_To_Buffer_Index - 1;
        }
        // 角度环 角度目标值更新 追踪前瞻目标点的 yaw 角
        AnglePID.Target = My_Flash_Buffer[pre_look_point];

        // 减速前瞻：扫描前方弯道幅度
        {
            float ref_yaw = My_Flash_Buffer[pre_look_point];
            uint32 scan_end = curr_point + DECEL_LOOK_AHEAD;
            if (scan_end >= Save_To_Buffer_Index)
            {
                scan_end = Save_To_Buffer_Index - 1;
            }
            float max_diff = 0.0f;
            for (uint32 i = pre_look_point; i < scan_end; i++)
            {
                float diff = fabsf(Get_Normalized_Angle_Error(My_Flash_Buffer[i], ref_yaw));
                if (diff > max_diff)
                {
                    max_diff = diff;
                }
            }
            Future_Turn_Angle = max_diff;
        }

        // 只有当没有到达目标点的情况下，才加位置
        if (Car_Go_Location >= Get_Dot_Loc && has_reached_point == false)
        {
            curr_point++;
            is_clear_loc = true;
            has_reached_point = true; // 标志位，说明我已经到达了，等定时器帮我清空
            //  printf(">>> Reach Point %d\r\n", (int)curr_point);
        }
    }
}

/*------------------------惯导算法变量------------------------*/

// 实例化辅助示波器用的结构体
seekfree_assistant_oscilloscope_struct oscilloscope_data;

#if TFT_DEBUG_ENABLE
static void Show_Race_Debug_Info(void)
{
    // 比赛调试页：优先看 yaw、目标、误差、点号和状态
    tft180_show_string(0, 16 * 0, "YAW:");          // yaw角
    tft180_show_float(32, 16 * 0, KalMan_Yaw, 5, 1);
    tft180_show_string(0, 16 * 1, "TGT:");          // 目标角度
    tft180_show_float(32, 16 * 1, AnglePID.Target, 5, 1);
    tft180_show_string(0, 16 * 2, "ERR:");          // 角度误差
    tft180_show_float(32, 16 * 2, error_angle, 4, 1);
    tft180_show_string(0, 16 * 3, "VEL:");          // 行进速度
    tft180_show_float(32, 16 * 3, Target_AveSpeed, 3, 1);
    tft180_show_string(0, 16 * 4, "PNT:");          // 当前复现点数
    tft180_show_int(32, 16 * 4, curr_point, 4);
    tft180_show_string(72, 16 * 4, "/");            // 存入flash的点数
    tft180_show_int(80, 16 * 4, Save_To_Buffer_Index, 4);
    tft180_show_string(0, 16 * 5, "REC:");          // 是否处于记录模式
    tft180_show_int(32, 16 * 5, is_record_for_tft, 1);
    tft180_show_string(64, 16 * 5, "REP:");         // 是否处于复现模式
    tft180_show_int(96, 16 * 5, is_replay_for_tft, 1);
    tft180_show_string(0, 16 * 6, "LSP:");          // 左轮速度
    tft180_show_float(32, 16 * 6, LeftSpeed, 3, 1);
    tft180_show_string(0, 16 * 7, "RSP:");          // 右轮速度
    tft180_show_float(32, 16 * 7, RightSpeed, 3, 1);
    tft180_show_string(0, 16 * 8, "FUT:");          // 前瞻弯道幅度
    tft180_show_float(32, 16 * 8, Future_Turn_Angle, 3, 1);
}
#endif

// **************************** 主函数 ****************************
int core0_main(void)
{
    clock_init(); // 获取时钟频率 <务必保留>
    debug_init(); // 初始化默认调试串口

    /* 底盘初始化 */
    Car_Init();

    /* 编码器初始化 */
    Encoder_Init();

    /* led 初始化 */
    LED_Init();

    /* PID 初始化 */
    PID_Init(&SpeedPID_L);
    PID_Init(&SpeedPID_R);
    PID_Init(&AnglePID);

    /* 拨码开关初始化 */
    Switch_Init();

    /* 按键初始化 */
    Key_Init();

    /* tft屏幕初始化 */
    tft180_set_dir(TFT180_PORTAIT);
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    tft180_init();
    tft180_clear();

    /*
            调大 Q 或减小 R --------> 响应加快 / 调大 R 或减小 Q --------> 响应变平滑
    */
    Yaw_Kalman_Filter_Init(0.01, 0);

    // 逐飞助手初始化，使用 DEBUG 串口进行收发
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    oscilloscope_data.channel_num = 5; // 设置显示通道数量，这里最大支持 8 个通道

    // imu 初始化
    while (1) {
        if (imu660rc_init(IMU660RC_QUARTERNION_240HZ)) {
            printf("\r\n IMU660RC init error.");
        } else {
            printf("\r\n IMU660RC init right.");
            break;
        }
    }

    /* 定时器初始化 */
    pit_ms_init(PIT_NUM, 1); // 初始化 CCU6_1_CH0 为周期中断 1ms 频率

//        cpu_wait_event_ready(); // 等待所有核心初始化完毕

    Fuya_Speed(0);          // 上电先关闭负压，复现时再按宏打开

    PID_Flag = false; // PID 开启标志位

    bool is_send_once = true; // 记录模式进入提示，只打印一次，不重复打印

//    SpeedPID_L.Target = 13;
//    SpeedPID_R.Target = 13;         // 初始速度环目标值

//    Right_Go_Forward(1500);Left_Go_Forward(1500);

    while (TRUE)
    {
        //        printf("test\r\n");
//        if(mt9v03x_finish_flag)
//        {
//            Image_Binarization();
//            scan_border();
//            mt9v03x_finish_flag = 0;
//        }

#if TFT_DEBUG_ENABLE
        Show_Race_Debug_Info();
#endif

        uint8_t key = Key_Get(); // 每轮循环只读一次按键
        if (is_recording == false && is_init_angle_done) // 如果不在记录模式中且已经读取了初始角度值
        {
            if (key == 4) // 检测按键4是否按下
            {
                is_recording = true; // 进入记录模式
                Fuya_Speed(0);      // 关掉负压
                is_record_for_tft = 1;      // tft屏幕显示
                if (is_send_once)
                {
                    printf("\r\n开始记录!\r\n");
                    is_use_fuya = false;
                    is_send_once = false;
                }
                PID_Flag = false;
                LED1_ON();
                LED2_ON();
                LED3_ON();
                LED4_ON(); // 点亮 LED1234

                Record_Index = 0;      // 记录点数清零
                encoder_right_loc = 0; // 编码器计程清零
                encoder_left_loc = 0;
                Car_Go_Location = 0; // 位置变量清零
            }
        }
        else if (is_recording == true) // 如果正在进行记录模式，那就检查是否要关闭记录模式
        {
            if (key == 2)       // 检测按键2是否按下
            {
                is_recording = false; // 关闭记录模式
                is_record_for_tft = 0;  // tft屏幕显示
                LED1_OFF();
                LED2_OFF();
                LED3_OFF();
                LED4_OFF(); // 熄灭 LED1 2 3 4

                printf("停止记录并准备存储...\r\n");
                printf("当前 Record_Index = %d\r\n", (int)Record_Index);
                printf("前 3 个打点的角度: [0]=%.2f, [1]=%.2f, [2]=%.2f\r\n",
                              My_Flash_Buffer[0], My_Flash_Buffer[1], My_Flash_Buffer[2]);

                Save_Data_To_Flash();
                printf("存储成功!\r\n");
            }
        }
        if (is_replaying == false && is_replay_only_once == true && is_init_angle_done) // 检测复现模式且已经拿到了初始角度值
        {
            if (key == 3)       // 检测按键3是否按下
            {
                is_replaying = true; // 进入复现模式
                is_replay_for_tft = 1;  // tft屏幕显示
                PID_Flag = true;
                is_use_fuya = true;
                Fuya_Speed(REPLAY_FUYA_SPEED); // 等待阶段先打开负压，避免起步瞬间打滑
                printf("复现开始，等待 %dms...\r\n", (int)REPLAY_WAIT_MS);
                is_replay_only_once = false;
                Read_the_flash();
                curr_point = 0;        // 当前点序号清零
                encoder_right_loc = 0; // 编码器计程清零
                encoder_left_loc = 0;
                Car_Go_Location = 0;       // 位置变量清零
                is_clear_loc = false;      // 标志位复位
                has_reached_point = false; // 保护一下，强制复位
                wait_for_a_while = false;
                count5 = 0;
                Target_AveSpeed = 0;
                Target_Angle = Target_init_angle;
                AnglePID.Target = Target_init_angle;
                AnglePID.Out = 0; // 强制角度环输出
                DifSpeed_Target = 0;
            }
        }

        if (is_replaying)
        {
            if (wait_for_a_while)
                Replay_the_path();
        }

        /* 示波器显示通道 */
        /*-----------------------角度环波形-----------------------*/
//           oscilloscope_data.data[0] = AnglePID.Actual;     // 显示AnglePID.Actual
//           oscilloscope_data.data[1] = AnglePID.Target;     //显示 AnglePID.Target
//           oscilloscope_data.data[2] = SpeedPID_R.Out;      //显示 AnglePID.Out
//           oscilloscope_data.data[3] = SpeedPID_L.Out;      //显示 SpeedPID_L.Out
//           oscilloscope_data.data[4] = AnglePID.Out;        //显示 SpeedPID_R.Out
        /*-----------------------角度环波形-----------------------*/

        /*-----------------------速度环波形-----------------------*/
//            oscilloscope_data.data[0] = SpeedPID_L.Actual; // 显示AnglePID.Actual
//            oscilloscope_data.data[1] = SpeedPID_L.Target; //显示 AnglePID.Target
//            oscilloscope_data.data[2] = SpeedPID_L.Out; // 显示 AnglePID.Out
        /*-----------------------速度环波形-----------------------*/

        /*------------------------逐飞上位机无线调参------------------------*/
        // 每次通过接收中断接收数据
//      seekfree_assistant_data_analysis();
//      // 调参
//      for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
//      {
//          // 更新标志位
//          if(seekfree_assistant_parameter_update_flag[i])
//          {
//              seekfree_assistant_parameter_update_flag[i] = 0;
//
//              // 通过 DEBBUG 串口反馈信息
//              printf("receive data channel : %d ", i);
//              printf("data : %f ", seekfree_assistant_parameter[i]);
//              printf("\r\n");
//          }
//       }
        //        根据通道选择
        /*-----------------------角度环参数-----------------------*/
//                AnglePID.Kp = seekfree_assistant_parameter[0];
//                AnglePID.KP2 = seekfree_assistant_parameter[1];
//                AnglePID.GKD = seekfree_assistant_parameter[2];
//                AnglePID.Kd = seekfree_assistant_parameter[3];
//                AnglePID.Target = seekfree_assistant_parameter[4];
        /*-----------------------角度环参数-----------------------*/

        /*-----------------------速度环参数-----------------------*/
//           SpeedPID_L.Kp = seekfree_assistant_parameter[0];
//           SpeedPID_L.Ki = seekfree_assistant_parameter[1];
//           SpeedPID_L.Target = seekfree_assistant_parameter[2];
        /*-----------------------速度环参数-----------------------*/

        /*------------------------逐飞上位机无线调参------------------------*/

        /*最后发送给上位机需要显示波形的值*/
//    seekfree_assistant_oscilloscope_send(&oscilloscope_data);

        // 此处可以写需要循环执行的代码
    }
}

// PID 中断函数 --> 1ms 定时中断
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    static uint16 Count1 = 0;   // 速度环 PID 周期计数
    static uint16 Count2 = 0;   // 角度环 PID 周期计数
    static uint16 Count6 = 0; // 上电后等待 IMU 稳定，再锁定初始 yaw

    Count1++;
    Count2++;

    // 等待 IMU 稳定后确定初始角度
    if (is_waiting_done == false)
    {
        Count6++;
    }
    if (Count6 >= INIT_YAW_LOCK_MS)
    {
        is_waiting_done = true;
        Count6 = 0;
    }

    // 复现等待，让负压和姿态先稳定
    if (is_replaying && wait_for_a_while == false)
    {
        count5++;
    }
    if (count5 >= REPLAY_WAIT_MS)
    {
        wait_for_a_while = true;
        count5 = 0;

        bool start_yaw_ok = true;
#if REPLAY_START_YAW_CHECK_ENABLE
        float replay_ref_yaw = 0.0f;
        float start_yaw_error = 0.0f;
        if (Save_To_Buffer_Index > 0)
        {
            replay_ref_yaw = My_Flash_Buffer[0];
            start_yaw_error = fabsf(Get_Normalized_Angle_Error(KalMan_Yaw, replay_ref_yaw));
            if (start_yaw_error > REPLAY_START_YAW_TOLERANCE)
            {
                start_yaw_ok = false;
            }
        }
        else
        {
            start_yaw_ok = false;
        }
#endif

        if (start_yaw_ok == false)
        {
#if REPLAY_START_YAW_CHECK_ENABLE
            printf("!!! replay yaw abnormal (yaw=%.2f, ref=%.2f, err=%.2f), skip replay !!!\r\n",
                    KalMan_Yaw, replay_ref_yaw, start_yaw_error);
#else
            printf("!!! replay start blocked, skip replay !!!\r\n");
#endif
            Stop_Replay_Safely();
        }
        else
        {
            Fuya_Speed(REPLAY_FUYA_SPEED);
            printf(">>> 等待结束，开始奔跑吧\r\n");
        }
    }

    /* 内环 速度环 PID 计算频率 2ms */
    if (Count1 >= 2) // 2ms
    {
        Count1 = 0;
        // 1. 获取编码器数值
        Encoder_Get();          // 获取编码器原始值
        Encoder_Get_Speed();    // 获取转速
        Encoder_Get_Location(); // 获取位置

        Car_Go_Location = 1.0 * (encoder_right_loc + encoder_left_loc) / 2; // 计算小车前进距离 cm
        if (is_clear_loc == true)
        {
            is_clear_loc = false;
            encoder_right_loc = 0;
            encoder_left_loc = 0;
            Car_Go_Location = 0;
            has_reached_point = false; // 标志位复位：位置完成，可以去下一个点了
        }

        LeftSpeed = encoder_left_speed;   // 获取左电机速度
        RightSpeed = encoder_right_speed; // 获取右电机速度

        AveSpeed = (LeftSpeed + RightSpeed) / 2.0; // 速度转换
        DifSpeed_Actual = LeftSpeed - RightSpeed;  // 速度转换

        SpeedPID_L.Actual = LeftSpeed;  // 设置实际速度
        SpeedPID_R.Actual = RightSpeed; // 设置实际速度

        // 速度环 速度目标值更新
        SpeedPID_L.Target = Target_AveSpeed + DifSpeed_Target;
        SpeedPID_R.Target = Target_AveSpeed - DifSpeed_Target;

        // 调参专用速度目标值
//    SpeedPID_L.Target = 15;
//    SpeedPID_R.Target = 15;

        /*------------------------循迹打点逻辑------------------------*/
        if (is_recording) // 如果正在记录模式
        {
            if (Car_Go_Location >= Get_Dot_Loc) // 如果小车位置大于等于打点距离
            {
                encoder_right_loc = 0;
                encoder_left_loc = 0;              // 清空小车当前位置
                Car_Go_Location = 0;               // 同时同步清零
                if (Record_Index < Buffer_Max_Num) // 如果缓存数组还有空余位置
                {
                    // 记录当前的 yaw 角，并存储到 flash 数组中
                    My_Flash_Buffer[Record_Index] = KalMan_Yaw; // 存储 yaw 角
                    Record_Index++;
                }
                else
                {
                    printf("没有空余位置了！\r\n");
                }
            }
        }
        /*------------------------循迹打点逻辑------------------------*/

        // 2. 速度环 PID 更新
        if (PID_Flag)
        {
            PID_Update_Incremental(&SpeedPID_L); // 更新速度环控制
            PID_Update_Incremental(&SpeedPID_R); // 更新速度环控制

            if (SpeedPID_L.Out > 0){Left_Go_Forward(SpeedPID_L.Out);}
            else if (SpeedPID_L.Out < 0){Left_Go_Back(-SpeedPID_L.Out);}
            else{Left_Go_Forward(0);}
            if (SpeedPID_R.Out > 0){Right_Go_Forward(SpeedPID_R.Out);}
            else if (SpeedPID_R.Out < 0){Right_Go_Back(-SpeedPID_R.Out);}
            else{Right_Go_Forward(0);}
        }
    }

    /* 外环 角度环 PID 计算频率 5ms */
    if (Count2 >= 5)
    {
        Count2 = 0;

        KalMan_Yaw = Normalize_Yaw_0_360(Kalman_Filter_Yaw_Update(imu660rc_yaw)); // 统一到 0~360，避免 -178/178 跳变
        Gyro_z = imu660rc_gyro_transition(imu660rc_gyro_z);  // 获取当前角速度值
        Filtered_Gyro_z = Alpha * Gyro_z + (1 - Alpha) * Filtered_Gyro_z; // 一阶低通滤波
        AnglePID.gyro_z = Filtered_Gyro_z;                  // 更新角度环的角度值

        if (is_waiting_done == true && is_set_once == false) // 设置初始角度
        {
            Target_init_angle = Normalize_Yaw_0_360(KalMan_Yaw);
            AnglePID.Target = Target_init_angle;
            is_init_angle_done = true;
            PID_Flag = true; // 补充：获取到初始角度，才开启 PID
            printf("\r\n>>> 初始角度已锁定: %.2f, PID 已开启\r\n", Target_init_angle);
            is_set_once = true;
        }

        // 只有中断开启
        if (PID_Flag)
        {
            // 角度环实际值更新
            AnglePID.Actual = KalMan_Yaw;

            // 计算偏差
            error_angle = Get_Normalized_Angle_Error(AnglePID.Actual, AnglePID.Target);

            // 偏差超过 0 度时启用 PID 控制
            if (fabs(error_angle) > 0)
            {
                PID_Update_Double_P(&AnglePID);
                // PID_Update_Positional(&AnglePID);
            }
            else
            {
                AnglePID.Out = 0.0;
            }
            DifSpeed_Target = AnglePID.Out;
            if (is_replaying && wait_for_a_while)
            {
                float effective_angle = fmax(fabs(error_angle), Future_Turn_Angle * FUTURE_DECEL_K);
                float turn_decel = effective_angle * K_Turn_Decel;
                Target_AveSpeed = Replay_Speed - turn_decel;
                if (Target_AveSpeed < Min_Turn_Speed)
                {
                    Target_AveSpeed = Min_Turn_Speed;
                }
            }
        }
    }
    pit_clear_flag(CCU61_CH0);
}

#pragma section all restore
// **************************** 主函数 ****************************
