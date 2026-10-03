#include "maze.h"
#include "motor.h"
#include "pid.h"
#include "wit.h"       // 你角度数据的头文件
#include "VL6180X.h"   // 如果需要地址宏等

#define IS_LEFT_TRIGGERED()   (g_left_mm_filt > 180)

// 检测到路口后，车还需要往前拱多少距离才到路口中心？
// 调大：车会走远点再转；调小：车会提前转
#define TEST_OFFSET_TICKS     200

static HardcodedState_t s_test_state = TEST_IDLE;
static float            s_target_yaw = 0.0f;    // 记录期望的绝对角度
static int32_t          s_start_tick = 0;       // 记录动作开始时的里程
volatile int32_t  g_maze_ticks = 0;
MazeState_t       g_maze_state = MAZE_IDLE;

// ToF 滤波后的距离
uint8_t g_front_mm_filt = 0;
uint8_t g_left_mm_filt  = 0;
uint8_t g_right_mm_filt = 0;

// 墙标志
uint8_t g_wall_front = 0;
uint8_t g_wall_left  = 0;
uint8_t g_wall_right = 0;

// ---- 内部：ToF 滑动平均缓冲 ----
static uint8_t s_front_buf[TOF_FILTER_LEN] = {0};
static uint8_t s_left_buf [TOF_FILTER_LEN] = {0};
static uint8_t s_right_buf[TOF_FILTER_LEN] = {0};
static uint8_t s_tof_idx = 0;

static void Start_Straight(void) {
		Motor_Stop();
    Maze_Odom_Reset();
    PID_Reset(&g_motorA_speed_pid);
    PID_Reset(&g_motorB_speed_pid);
    
    Motion_SetControlMode(MOTION_MODE_DIFFERENTIAL); // 使用你 pid.c 新增的辅助直行模式
    Motion_Set_Speed(TEST_FWD_SPEED, 0.0f);
		Motor_On();
    
    g_motor_state = MOTOR_STATE_RUNNING;
}

static void Start_Turn(float angle_delta) {
		Motor_Stop();
		g_turn_complete = 0;
    PID_Reset(&g_motorA_speed_pid);
    PID_Reset(&g_motorB_speed_pid);
    PID_Reset(&g_yaw_angle_pid);
    
    Motion_SetControlMode(MOTION_MODE_YAW_CORRECTED);
    
    // 更新全局目标角度
    s_target_yaw += angle_delta; 
	
		//if (s_target_yaw > 180.0f) s_target_yaw -= 360.0f;
    //if (s_target_yaw < -180.0f) s_target_yaw += 360.0f;
    Motion_Set_Target_Angle(s_target_yaw);
    
    Motion_Set_Speed(0, 0); // 原地转向
	
		Motor_On();
    g_motor_state = MOTOR_STATE_RUNNING;
}

void Maze_Hardcoded_Start(void)
{
    // 初始化
    s_target_yaw = g_wit_data.yaw; // 以当前车头为0参考
    s_test_state = TEST_SEQ1_BLIND_RUN;
    
    // 开始第一步：直行
    Start_Straight();
}

void Maze_Hardcoded_Task(void)
{
    // 如果电机被 pid.c 自动关停了，且我们在等待转弯结束，这里不需要特殊处理
    // 因为我们会检查 g_motor_state
    
    switch (s_test_state)
    {
    case TEST_IDLE:
        break;

		case TEST_DEBUG_ONE_CELL:
        // 专门跑一格：使用 MAZE_CELL_TICKS 这个宏
        // 当里程达到设定的值时，停车
        if (g_maze_ticks >= MAZE_CELL_TICKS) {
            Motor_Stop();
            g_motor_state = MOTOR_STATE_IDLE; // 标记为空闲
            s_test_state = TEST_DONE;         // 标记为完成
        }
        break;
				
    // --------------------------------------------------------
    // 阶段 1：特殊的前进 5 格 (盲走 -> 搜索 -> 补偿)
    // --------------------------------------------------------
    case TEST_SEQ1_BLIND_RUN:
        // 目标：盲走 4 格
        if (g_maze_ticks >= 4 * MAZE_CELL_TICKS) {
            // 不停车，状态无缝切换，继续走
            s_test_state = TEST_SEQ1_SEARCH;
        }
        break;

    case TEST_SEQ1_SEARCH:
        // 目标：检测左侧路口
        if (IS_LEFT_TRIGGERED()) {
            // 记录此刻里程，开始最后一段“进中补偿”
            s_start_tick = g_maze_ticks; 
            s_test_state = TEST_SEQ1_OFFSET;
        }
        break;

    case TEST_SEQ1_OFFSET:
        // 目标：把车身拱到路口中心
        if (g_maze_ticks >= (s_start_tick + TEST_OFFSET_TICKS)) {
            Motor_Stop(); // 先停
            // 准备左转 90
            Start_Turn(90.0f); // 左转是 +90 (根据你 app_key.c 的逻辑)
            s_test_state = TEST_SEQ2_TURN_L_90;
        }
        break;

    // --------------------------------------------------------
    // 阶段 2：左转 90 (等待 PID 自动完成)
    // --------------------------------------------------------
    case TEST_SEQ2_TURN_L_90:
        // 你的 pid.c 里 ControlLoop_YawCorrected 会在误差<2度时自动设为 IDLE
        if (g_turn_complete == 1) {
            // 转弯完成，开始下一段直行 (1格)
            Start_Straight();
            s_test_state = TEST_SEQ3_FWD_1;
        }
        break;

    // --------------------------------------------------------
    // 阶段 3：前进 1 格
    // --------------------------------------------------------
    case TEST_SEQ3_FWD_1:
        if (g_maze_ticks >= 1 * MAZE_CELL_TICKS) {
            Motor_Stop();
            Start_Turn(90.0f); // 左转 90
            s_test_state = TEST_SEQ4_TURN_L_90;
        }
        break;

    // --------------------------------------------------------
    // 阶段 4：左转 90
    // --------------------------------------------------------
    case TEST_SEQ4_TURN_L_90:
        if (g_turn_complete == 1) {
            Start_Straight();
            s_test_state = TEST_SEQ5_FWD_1;
        }
        break;

    // --------------------------------------------------------
    // 阶段 5：前进 1 格
    // --------------------------------------------------------
    case TEST_SEQ5_FWD_1:
        if (g_maze_ticks >= 1 * MAZE_CELL_TICKS) {
            Motor_Stop();
            Start_Turn(-90.0f); // 右转 90 (通常是 -90)
            s_test_state = TEST_SEQ6_TURN_R_90;
        }
        break;

    // --------------------------------------------------------
    // 阶段 6：右转 90
    // --------------------------------------------------------
    case TEST_SEQ6_TURN_R_90:
        if (g_turn_complete == 1) {
            Start_Straight();
            s_test_state = TEST_SEQ7_FWD_3;
        }
        break;

    // --------------------------------------------------------
    // 阶段 7：前进 3 格
    // --------------------------------------------------------
    case TEST_SEQ7_FWD_3:
        if (g_maze_ticks >= 3 * MAZE_CELL_TICKS) {
            Motor_Stop();
            g_motor_state = MOTOR_STATE_IDLE;
            s_test_state = TEST_DONE;
        }
        break;

    case TEST_DONE:
        // 任务完成，可以在这里让蜂鸣器响一声
		Motor_Stop();
        break;
    }
}

// ================= 1. 编码器里程计 =================
void Maze_Odom_Reset(void)
{
    g_maze_ticks = 0;
}

void Maze_Test_OneCell_Start(void)
{
    // 1. 复位里程和所有PID
		Motor_Stop();
    Maze_Odom_Reset();
    PID_Reset(&g_motorA_speed_pid);
    PID_Reset(&g_motorB_speed_pid);
    
    
    // 3. 设置为直行模式
    Motion_SetControlMode(MOTION_MODE_DIFFERENTIAL);
    Motion_Set_Speed(TEST_FWD_SPEED, 0.0f); // 使用宏定义的测试速度
    
    // 4. 启动电机并切换到调试状态
		Motor_On();
    g_motor_state = MOTOR_STATE_RUNNING;
    s_test_state = TEST_DEBUG_ONE_CELL; // <--- 设置状态
}

void Maze_Odom_Update(int16_t deltaA, int16_t deltaB)
{
    int32_t delta_avg = ((int32_t)deltaA + (int32_t)deltaB) / 2;
    g_maze_ticks += delta_avg;
}

// ================= 2. ToF 滤波 & 判墙 =================
void Maze_OnTofUpdate(uint8_t d_front, uint8_t d_left, uint8_t d_right)
{
    // 把这一次的原始读数放进环形缓冲
    s_front_buf[s_tof_idx] = d_front;
    s_left_buf [s_tof_idx] = d_left;
    s_right_buf[s_tof_idx] = d_right;

    s_tof_idx++;
    if (s_tof_idx >= TOF_FILTER_LEN) s_tof_idx = 0;

    // 求平均
    uint16_t sum_f = 0, sum_l = 0, sum_r = 0;
    for (uint8_t i = 0; i < TOF_FILTER_LEN; i++) {
        sum_f += s_front_buf[i];
        sum_l += s_left_buf[i];
        sum_r += s_right_buf[i];
    }
    g_front_mm_filt = (uint8_t)(sum_f / TOF_FILTER_LEN);
    g_left_mm_filt  = (uint8_t)(sum_l / TOF_FILTER_LEN);
    g_right_mm_filt = (uint8_t)(sum_r / TOF_FILTER_LEN);

    // 阈值判定“是不是墙”
    g_wall_front = (g_front_mm_filt <= WALL_FRONT_THRESH_MM);
    g_wall_left  = (g_left_mm_filt  <= WALL_LEFT_THRESH_MM);
    g_wall_right = (g_right_mm_filt <= WALL_RIGHT_THRESH_MM);
}

// ================= 3. 启动迷宫模式 =================
void Maze_Start(void)
{
    Maze_Odom_Reset();

    // 设置控制模式：直线 + 航向保持
    Motion_SetControlMode(MOTION_MODE_YAW_STRAIGHT);
    // 把当前角度作为“直线参考”
    Motion_Set_Target_Angle(g_wit_data.yaw);
    // 设置一个你合适的前进速度
    Motion_Set_Speed(10.0f, 0.0f);   // 这个 100 自己按你工程的单位改

    g_motor_state = MOTOR_STATE_RUNNING;
    g_maze_state  = MAZE_GO_1CELL;
}

// ================= 4. 迷宫任务（状态机框架） =================
void Maze_Task(void)
{
    switch (g_maze_state)
    {
    case MAZE_IDLE:
        // 等待按键调用 Maze_Start()
        break;

    case MAZE_GO_1CELL:
        // 简单版：每次只走 1 格，不减去一格，走到就停
        if (g_maze_ticks >= MAZE_CELL_TICKS) {
            Motion_Set_Speed(0.0f, 0.0f);
            g_motor_state = MOTOR_STATE_IDLE;
            g_maze_state  = MAZE_IDLE;			//调试测试完毕后修改
        }
        break;

    case MAZE_DECIDE:
        // 右手法则示例（你可以以后再细化）:
        // 优先右无墙 -> 前无墙 -> 左无墙 -> 否则掉头

        if (!g_wall_right) {
            // TODO: 设置目标角 = 当前角 - 90°，g_maze_state = MAZE_TURNING;
        } else if (!g_wall_front) {
            // 继续直走一格
            Maze_Odom_Reset();
            Motion_Set_Target_Angle(g_wit_data.yaw); // 重新锁定当前方向
            Motion_Set_Speed(30.0f, 0.0f);
            g_motor_state = MOTOR_STATE_RUNNING;
            g_maze_state  = MAZE_GO_1CELL;
        } else if (!g_wall_left) {
            // TODO: 设置目标角 = 当前角 + 90°，g_maze_state = MAZE_TURNING;
        } else {
            // 四面都是墙 -> 掉头 180°
            // TODO: 设置目标角 = 当前角 +/- 180°，g_maze_state = MAZE_TURNING;
        }
        break;

    case MAZE_TURNING:
        // 这里用你的角度 PID 判断 “转到了目标角没有”
        // if (fabs(angle_error) < 某个阈值) {
        //     Maze_Odom_Reset();
        //     Motion_Set_Speed(100.0f, 0.0f);
        //     g_motor_state = MOTOR_STATE_RUNNING;
        //     g_maze_state  = MAZE_GO_1CELL;
        // }
        break;
    }
}
