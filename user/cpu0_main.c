#include "zf_common_headfile.h"
#include "isr_config.h"
#pragma section all "cpu0_dsram"

/* 定时中断宏定义 */
#define PIT_NUM                 (CCU61_CH0 )

/*------------------------ 陀螺仪处理参数 ------------------------*/
float yaw;
float KalMan_Yaw = 0;
float Target_init_angle = 0.0f;
bool is_waiting_done = false;
bool is_init_angle_done = false;
bool is_set_once = false;

/*------------------------ PID 处理参数 ------------------------*/
#define MAX_SPEED               (5000)
PID_t SpeedPID_L = {.Kp = 31.8, .Ki = 5.8, .Kd = 0.0, .OutMax = MAX_SPEED, .OutMin = -MAX_SPEED};
PID_t SpeedPID_R = {.Kp = 31.8, .Ki = 5.8, .Kd = 0.0, .OutMax = MAX_SPEED, .OutMin = -MAX_SPEED};
PID_t AnglePID = {.Kp = 0.05, .Kd = 0.0,.GKD = 0.04,.KP2 = 0.002,.OutMax = 30,.OutMin = -30,};

float Target_AveSpeed = 0;
float Target_Angle = 0;
float LeftSpeed, RightSpeed;
float AveSpeed, DifSpeed_Actual, DifSpeed_Target;
bool PID_Flag = false;
double error_angle = 0;
bool is_use_fuya = false;

/*------------------------ 惯导+存储参数 ------------------------*/
// xy 坐标系
float x_delta, y_delta;
float x_sum, y_sum;           // 当前实时坐标
float rad_angle;

// 路径点结构体
typedef struct
{
    float x;
    float y;
} nav_spot;

// Flash 存储配置
#define MAX_PATH_POINTS_PER_PAGE     500
#define MAX_PATH_POINTS              6000
#define MAX_PAGE_COUNTS              12

nav_spot gps_buffer[MAX_PATH_POINTS];       // 路径点缓冲区
uint16 path_point_count = 0;                // 已记录的点数

// 记录/复现标志
bool is_recording = false;
bool is_replaying = false;
bool is_replay_only_once = true;

// 复现用变量
uint32 replay_current_count = 0;
uint32 replay_sum_count = 0;
#define Replay_Speed    11
#define PRE_LOOK_COUNT  7
#define REACH_THRESHOLD 12.0f

// 原有变量兼容
float Car_Go_Location = 0;
bool is_clear_loc = false;
bool has_reached_point = false;
uint16 count5 = 0;
bool wait_for_a_while = false;
#define Get_Dot_Loc     (2)           // 仍然保持每 2cm 存一个点的习惯

/*------------------------ 函数声明 ------------------------*/
void Save_Path_To_Flash(void);
bool is_reading_flash(void);
void replay_the_path(void);

// 初始化逐飞助手示波器
seekfree_assistant_oscilloscope_struct oscilloscope_data;

// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();
    debug_init();
    Car_Init();
    Encoder_Init();
    LED_Init();
    PID_Init(&SpeedPID_L);
    PID_Init(&SpeedPID_R);
    PID_Init(&AnglePID);
    Switch_Init();

    Yaw_Kalman_Filter_Init(0.01, 1.5);
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    oscilloscope_data.channel_num = 1;                                      // 选择波形显示数量

    while(1) {
        if(imu660rc_init(IMU660RC_QUARTERNION_120HZ)) {
            printf("\r\n IMU660RC init error.");
        } else {
            printf("\r\n IMU660RC init right.");
            break;
        }
    }

    pit_ms_init(PIT_NUM, 1);
    Fuya_Speed(0);
    PID_Flag = false;
    bool is_send_once = true;

    while (TRUE)
    {
        /*------------------------ 按键逻辑 ------------------------*/
        if(is_recording == false && is_init_angle_done)
        {
            if(Switch1_Get() == 1)              // 开启记录模式
            {
                is_recording = true;
                if(is_send_once)
                {
                    printf("\r\n>>> Start Recording XY Path...\r\n");
                    is_use_fuya = false;
                    is_send_once = false;
                }
                PID_Flag = false;
                LED1_ON();LED2_ON();LED3_ON();LED4_ON();

                // 清零坐标
                path_point_count = 0;
                x_sum = 0; y_sum = 0;
                encoder_right_loc = 0; encoder_left_loc = 0;
                Car_Go_Location = 0;
            }
        }
        else if(is_recording == true)               // 关闭记录模式
        {
            if(Switch1_Get() == 0)
            {
                is_recording = false;
                LED1_OFF();LED2_OFF();LED3_OFF();LED4_OFF();
                printf("停止记录！准备存储... 当前点数: %d\r\n", path_point_count);
                Save_Path_To_Flash();
                printf("存储成功!\r\n");
                is_send_once = true;            // 恢复标志位以便下次记录
            }
        }

        if(is_replaying == false && is_replay_only_once == true && is_init_angle_done)
        {
            if(Switch2_Get() == 1)                  // 开启复现模式
            {
                is_replaying = true;
                PID_Flag = true;
                is_use_fuya = true;
                printf("复现开始，等待...\r\n");
                is_replay_only_once = false;

                if(is_reading_flash())
                {
                    // 读取成功，初始化
                    x_sum = 0; y_sum = 0;
                    replay_current_count = 0;
                    encoder_right_loc = 0; encoder_left_loc = 0;
                    Car_Go_Location = 0;
                    is_clear_loc = false;
                    has_reached_point = false;
                    wait_for_a_while = false;
                    count5 = 0;
                    Target_AveSpeed = 0;
                    // 【漏了这两句极其关键的代码！加上它们！】
                    // 把放下的这一瞬间强制设为车头 0 度
                    Target_init_angle = KalMan_Yaw;
                    yaw = 0;
                    // 【必须修改】：把下面这两行改为 0
                    Target_Angle = 0;
                    AnglePID.Target = 0;
                    AnglePID.Out = 0;
                    DifSpeed_Target = 0;
                }
                else
                {
                    // 读取失败
                    is_replaying = false;
                    is_replay_only_once = true;
                }
            }
        }

        if(is_replaying)
        {
            if(wait_for_a_while) replay_the_path();
        }

        /*----------------------- 角度环波形 -----------------------*/
//       oscilloscope_data.data[0] = AnglePID.Actual;                             // 显示 AnglePID.Actual
//       oscilloscope_data.data[1] = AnglePID.Target;                             // 显示 AnglePID.Target
//       oscilloscope_data.data[2] = SpeedPID_R.Out;                              // 显示 AnglePID.Out
//       oscilloscope_data.data[3] = SpeedPID_L.Out;                              // 显示 SpeedPID_L.Out
//       oscilloscope_data.data[4] = AnglePID.Out;                                // 显示 SpeedPID_R.Out
        /*----------------------- 角度环波形 -----------------------*/

        /*----------------------- 速度环波形 -----------------------*/
//        oscilloscope_data.data[0] = SpeedPID_R.Actual;                             // 显示 SpeedPID_R.Actual
//        oscilloscope_data.data[1] = SpeedPID_R.Target;                             // 显示 SpeedPID_R.Target
//        oscilloscope_data.data[2] = SpeedPID_R.Out;                                // 显示 SpeedPID_R.Out
       /*----------------------- 速度环波形 -----------------------*/

        /*----------------------- 临时波形显示 -----------------------*/

       /*----------------------- 临时波形显示 -----------------------*/

/*------------------------ 逐飞上位机无线调参 ------------------------*/
//       seekfree_assistant_data_analysis();
//        for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
//        {
//            if(seekfree_assistant_parameter_update_flag[i])
//            {
//                seekfree_assistant_parameter_update_flag[i] = 0;
//                printf("receive data channel : %d ", i);
//                printf("data : %f ", seekfree_assistant_parameter[i]);
//                printf("\r\n");
//            }
//        }
       /*----------------------- 速度环参数 -----------------------*/
//       SpeedPID_R.Kp = seekfree_assistant_parameter[0];
//       SpeedPID_R.Ki = seekfree_assistant_parameter[1];
//       SpeedPID_R.Target = seekfree_assistant_parameter[2];
       /*----------------------- 速度环参数 -----------------------*/

        /*----------------------- 角度环参数 -----------------------*/
//        AnglePID.Kp = seekfree_assistant_parameter[0];
//        AnglePID.GKD = seekfree_assistant_parameter[1];
//        AnglePID.KP2 = seekfree_assistant_parameter[2];
        /*-----------------------角度环参数 -----------------------*/

/*------------------------ 发送波形 ------------------------*/
//       seekfree_assistant_oscilloscope_send(&oscilloscope_data);
    }
}

// PID 中断函数 --> 1ms 定时中断
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    static uint16 Count1 = 0;
    static uint16 Count2 = 0;
    static uint16 Count6 = 0;

    Count1++; Count2++;

    if(is_waiting_done == false) Count6 ++;
    if(Count6 >= 2000) {is_waiting_done = true; Count6 = 0;}

    if(is_replaying && wait_for_a_while == false) count5++;
    if(count5 >= 3000) {
        wait_for_a_while = true; count5 = 0;
        printf(">>> 等待结束，开始跑车！\r\n");
    }

    /* 内环：速度环 PID 调控周期 2ms */
    if(Count1 >= 2)
    {
        Count1 = 0;
        Encoder_Get();
        Encoder_Get_Speed();
        Encoder_Get_Location();

        Car_Go_Location = 1.0*(encoder_right_loc + encoder_left_loc) / 2;

        if(is_clear_loc == true)
        {
            is_clear_loc = false;
            encoder_right_loc = 0; encoder_left_loc = 0;
            Car_Go_Location = 0;
            has_reached_point = false;
        }

        LeftSpeed = encoder_left_speed;
        RightSpeed = encoder_right_speed;
        AveSpeed = (LeftSpeed+RightSpeed) / 2.0;
        DifSpeed_Actual = LeftSpeed - RightSpeed;

        SpeedPID_L.Actual = LeftSpeed;
        SpeedPID_R.Actual = RightSpeed;

        // 接入全局目标速度 和 角度环输出的差速
        SpeedPID_L.Target = Target_AveSpeed + DifSpeed_Target;
        SpeedPID_R.Target = Target_AveSpeed - DifSpeed_Target;

        /*------------------------ 惯导计算（记录+复现通用） ------------------------*/
        if(is_recording || is_replaying)
        {
            float distance_delta = (encoder_right_loc_delta + encoder_left_loc_delta) / 2.0;
            rad_angle = yaw * PI / 180.0f;

            x_delta = distance_delta * sinf(rad_angle);
            y_delta = distance_delta * cosf(rad_angle);
            x_sum += x_delta;
            y_sum += y_delta;

            if(is_recording)
            {
                if(Car_Go_Location >= Get_Dot_Loc)
                {
                    encoder_right_loc = 0; encoder_left_loc = 0;
                    Car_Go_Location = 0;

                    if(path_point_count < MAX_PATH_POINTS)
                    {
                        gps_buffer[path_point_count].x = x_sum;
                        gps_buffer[path_point_count].y = y_sum;
                        path_point_count++;
                    }
                }
            }
        }

        if(PID_Flag)
        {
            PID_Update_Incremental(&SpeedPID_L);
            PID_Update_Incremental(&SpeedPID_R);
        }

        if(SpeedPID_L.Out > 0) Left_Go_Forward(SpeedPID_L.Out);
        else if(SpeedPID_L.Out < 0) Left_Go_Back(-SpeedPID_L.Out);
        else Left_Go_Forward(0);

        if(SpeedPID_R.Out > 0) Right_Go_Forward(SpeedPID_R.Out);
        else if(SpeedPID_R.Out < 0) Right_Go_Back(-SpeedPID_R.Out);
        else Right_Go_Forward(0);
    }

    /* 外环：角度环 PID 调控周期 5ms */
    if(Count2 >= 5)
    {
        Count2 = 0;
        KalMan_Yaw = Kalman_Filter_Yaw_Update(imu660rc_yaw);
        yaw = KalMan_Yaw - Target_init_angle; // 相对角度

        if(is_waiting_done == true && is_set_once == false)
        {
            Target_init_angle = KalMan_Yaw;         // 得出初始角度
            yaw = KalMan_Yaw - Target_init_angle;   // 相对角度
            AnglePID.Target = yaw;                  // 设置目标角度
            is_init_angle_done = true;
            is_set_once = true;
            PID_Flag = true;
            printf("\r\n>>> 初始角度已对齐: %.2f\r\n", Target_init_angle);
        }

        if(PID_Flag)
        {
            AnglePID.Actual = yaw;

            // 1. 误差防缠绕计算区 (使用 Target - Actual 保持和PID库内部一致的符号)
            error_angle = AnglePID.Target - AnglePID.Actual;

            // 误差解卷绕 (Wrap-around)，强制拉回最短路径
            while (error_angle > 180.0f) {
                error_angle -= 360.0f;
            }
            while (error_angle < -180.0f) {
                error_angle += 360.0f;
            }

            // 2. 将修剪后的真实误差送入 PID 控制器
            if(fabs(error_angle) > 1.0)
            {
                // 【核心逻辑：变量欺骗法】
                // 暂存原数值，以免破坏示波器的波形观察
                float original_target = AnglePID.Target;

                // 伪装变量：让 PID 底层算出来的 p->Target - p->Actual 恰好等于修剪好的 error_angle
                AnglePID.Target = error_angle;
                AnglePID.Actual = 0;

                // 执行控制计算
                PID_Update_Double_P(&AnglePID);

                // 计算完毕，火速恢复现场
                AnglePID.Target = original_target;
                AnglePID.Actual = yaw;
            }
            else
            {
                AnglePID.Out = 0.0;
            }
            DifSpeed_Target = AnglePID.Out;
        }
    }
    pit_clear_flag(CCU61_CH0);
}

/*------------------------ 核心功能函数 ------------------------*/

/**
 * @brief 存储路径到 Flash (XY 结构体版本)
 */
void Save_Path_To_Flash(void)
{
    uint32 remain_path_points = path_point_count;
    if(remain_path_points == 0 || remain_path_points > MAX_PATH_POINTS) {
        printf("数据异常!\r\n"); return;
    }

    uint32 global_index = 0;
    int page = 0;
    printf("开始存储...\r\n");

    for(page = 0; page < MAX_PAGE_COUNTS; page++)
    {
        if(remain_path_points == 0) break;

        flash_erase_page(0, page);
        flash_buffer_clear();

        uint32 current_page_points = remain_path_points > MAX_PATH_POINTS_PER_PAGE ? MAX_PATH_POINTS_PER_PAGE : remain_path_points;
        flash_union_buffer[0].uint32_type = current_page_points;

        for(int i = 0; i < current_page_points; i++)
        {
            uint32 save_index = 1 + 2 * i;
            flash_union_buffer[save_index].float_type = gps_buffer[global_index].x;
            flash_union_buffer[save_index + 1].float_type = gps_buffer[global_index].y;
            global_index++;
        }

        flash_write_page_from_buffer(0, page);
        printf("第 %d 页存了 %d 个点\r\n", (int)page, (int)current_page_points);
        remain_path_points -= current_page_points;
    }
    printf("存储完毕，共 %d 页\r\n", page);
}

/**
 * @brief 从 Flash 读取路径
 */
bool is_reading_flash(void)
{
    uint32 global_index = 0;
    replay_sum_count = 0;
    printf("开始读取...\r\n");

    for(int page = 0; page < MAX_PAGE_COUNTS; page++)
    {
        if(flash_check(0, page) == 0) break;

        flash_buffer_clear();
        flash_read_page_to_buffer(0, page);

        uint32 replay_page_count = flash_union_buffer[0].uint32_type;
        replay_sum_count += replay_page_count;

        if(replay_page_count > MAX_PATH_POINTS_PER_PAGE || replay_sum_count > MAX_PATH_POINTS) {
            printf("数据溢出!\r\n"); return false;
        }

        for(int i = 0; i < replay_page_count; i++)
        {
            uint32 points_index = 1 + 2 * i;
            gps_buffer[global_index].x = flash_union_buffer[points_index].float_type;
            gps_buffer[global_index].y = flash_union_buffer[points_index + 1].float_type;
            if(global_index < 3) {
                printf("点 %d: (%.2f, %.2f)\r\n", (int)global_index, gps_buffer[global_index].x, gps_buffer[global_index].y);
            }
            global_index++;
        }
        printf("第 %d 页读取完成\r\n", page);
    }

    if(replay_sum_count > 0) {
        printf("读取成功，共 %d 个点\r\n", (int)replay_sum_count);
        return true;
    } else {
        printf("读取失败\r\n");
        return false;
    }
}

//void replay_the_path(void)
//{
//    // =========================================================================
//    // 1. 【核心修复】：动态寻找当前车身最近的路径点（滑动窗口法，彻底解决切弯漏点死锁）
//    // =========================================================================
//    // 往前方找 25 个点（大约 50cm 的窗口），看看哪个点离车最近
//    uint32 search_end = replay_current_count + 10;
//    if (search_end > replay_sum_count) search_end = replay_sum_count;
//
//    float min_dist = 99999.0f;
//    uint32 best_index = replay_current_count;
//
//    for (uint32 i = replay_current_count; i < search_end; i++)
//    {
//        float dx = gps_buffer[i].x - x_sum;
//        float dy = gps_buffer[i].y - y_sum;
//        float d = sqrtf(dx*dx + dy*dy);
//        if (d < min_dist)
//        {
//            min_dist = d;
//            best_index = i;
//        }
//    }
//
//    // 自动将当前索引更新为物理最近点！哪怕切弯漏掉了直角顶点，它也会自动跳过并跟上！
//    replay_current_count = best_index;
//
//    // =========================================================================
//    // 2. 终点安全判定逻辑：看最近点是否到达末尾，且真的进入了停车圈
//    // =========================================================================
//    if(replay_current_count >= replay_sum_count - 2)
//    {
//        // 算一下离绝对终点的距离
//        float end_dx = gps_buffer[replay_sum_count-1].x - x_sum;
//        float end_dy = gps_buffer[replay_sum_count-1].y - y_sum;
//        float dist_to_end = sqrtf(end_dx*end_dx + end_dy*end_dy);
//
//        if (dist_to_end < REACH_THRESHOLD) // 真正到达了终点 10cm 内
//        {
//            is_replaying = false;
//            Target_AveSpeed = 0;
//            DifSpeed_Target = 0;
//            PID_Flag = false;
//
//            // 清空冻结输出，安全停车
//            SpeedPID_L.Out = 0;
//            SpeedPID_R.Out = 0;
//            AnglePID.Out = 0;
//            Left_Go_Forward(0);
//            Right_Go_Forward(0);
//
//            printf("复现完美到达终点!\r\n");
//            return;
//        }
//    }
//
//    // =========================================================================
//    // 3. 正常跟踪计算
//    // =========================================================================
//    // 计算前视点
//    uint32 Look_Ahead_Index = replay_current_count + PRE_LOOK_COUNT;
//    if(Look_Ahead_Index >= replay_sum_count) Look_Ahead_Index = replay_sum_count - 1;
//
//    // 计算目标绝对角度
//    float look_dx = gps_buffer[Look_Ahead_Index].x - x_sum;
//    float look_dy = gps_buffer[Look_Ahead_Index].y - y_sum;
//    float target_rad = atan2f(look_dx, look_dy);
//    float target_deg = target_rad * 180.0f / PI;
//
//    // 赋值控制
//    Target_AveSpeed = Replay_Speed;
//    AnglePID.Target = target_deg;
//
//    // 调试打印
//    static int debug_cnt = 0;
//    if(debug_cnt++ >= 100) {
//        debug_cnt = 0;
//        printf("Idx:%d | Pos(%.1f,%.1f) | TgtDeg:%.1f | MinDist:%.1f\r\n",
//               (int)replay_current_count, x_sum, y_sum, target_deg, min_dist);
//    }
//}

void replay_the_path(void)
{
    // =========================================================================
    // 1. 终点安全停车判定（先判断是否该下班了）
    // =========================================================================
    if(replay_current_count >= replay_sum_count - 2)
    {
        float end_dx = gps_buffer[replay_sum_count-1].x - x_sum;
        float end_dy = gps_buffer[replay_sum_count-1].y - y_sum;
        if (sqrtf(end_dx*end_dx + end_dy*end_dy) < REACH_THRESHOLD)
        {
            is_replaying = false;
            Target_AveSpeed = 0; DifSpeed_Target = 0; PID_Flag = false;
            SpeedPID_L.Out = 0; SpeedPID_R.Out = 0; AnglePID.Out = 0;
            Left_Go_Forward(0); Right_Go_Forward(0);
            printf("复现完美到达终点!\r\n");
            return;
        }
    }

    // =========================================================================
    // 2. 动态寻找最近点（防丢失）
    // =========================================================================
    uint32 search_end = replay_current_count + 15; // 找 30cm 内的最近点
    if (search_end > replay_sum_count) search_end = replay_sum_count;

    float min_dist = 99999.0f;
    uint32 best_index = replay_current_count;

    for (uint32 i = replay_current_count; i < search_end; i++)
    {
        float dx = gps_buffer[i].x - x_sum;
        float dy = gps_buffer[i].y - y_sum;
        float d = sqrtf(dx*dx + dy*dy);
        if (d < min_dist) { min_dist = d; best_index = i; }
    }
    replay_current_count = best_index;

    // =========================================================================
        // 3. 【极简追踪核心】：砍掉长前视，只盯住眼前 3 个点（6cm）的位置
        // =========================================================================
        uint32 target_index = replay_current_count + 3;
        if (target_index >= replay_sum_count) target_index = replay_sum_count - 1;

        float look_dx = gps_buffer[target_index].x - x_sum;
        float look_dy = gps_buffer[target_index].y - y_sum;
        float target_rad = atan2f(look_dx, look_dy);

        // 设置目标角度
        AnglePID.Target = target_rad * 180.0f / PI;

        // 【新增绝招：弯道自适应降速 (Corner Braking)】
        // 计算当前角度环到底输出了多大的转向力（取绝对值）
        float turn_effort = fabs(AnglePID.Out);

        // 用基础速度减去转向力的一部分。转弯越急，前进速度越慢！
        // 0.4 是个阻尼系数，你可以微调它。
        float current_forward_speed = Replay_Speed - turn_effort * 0.4f;

        // 保证不要倒车
        if(current_forward_speed < 0) current_forward_speed = 0;

        // 赋值最终速度
        Target_AveSpeed = current_forward_speed;

        static int debug_cnt = 0;
        if(debug_cnt++ >= 100) {
            debug_cnt = 0;
            printf("Idx:%d | Pos(%.1f,%.1f) | TgtDeg:%.1f | FwdSpd:%.1f\r\n",
                   (int)replay_current_count, x_sum, y_sum, AnglePID.Target, current_forward_speed);
        }
    }

#pragma section all restore





