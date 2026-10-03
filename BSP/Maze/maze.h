#ifndef __MAZE_H__
#define __MAZE_H__

#include <stdint.h>

// ================== 1. 编码器“走一格”的 tick 数（实验调） ==================
#define MAZE_CELL_TICKS           800  // 先随便给个数，跑一次再慢慢调

#define TEST_FWD_SPEED        10.0f   // 直行速度

// ================== 2. ToF 判定“是不是墙”的阈值（mm） ==================
#define WALL_FRONT_THRESH_MM      150    // 前方小于这个距离认为有墙
#define WALL_LEFT_THRESH_MM       150
#define WALL_RIGHT_THRESH_MM      150

// ================== 3. ToF 滤波参数 ==================
#define TOF_FILTER_LEN            3     // 简单 N 点滑动平均
// 建议读取周期：50~100ms 一次就够了（20Hz~10Hz）
// 每次调用 VL6180X_ReadRange_Safe 会自己等测量完成，太频繁也没意义

// ================== 4. 迷宫状态机 ==================
typedef enum {
    MAZE_IDLE = 0,
    MAZE_GO_1CELL,
    MAZE_DECIDE,
    MAZE_TURNING
} MazeState_t;

typedef enum {
    TEST_IDLE = 0,
	
		TEST_DEBUG_ONE_CELL,
    
    // --- 第一段：特殊的前进5格 ---
    TEST_SEQ1_BLIND_RUN,    // 盲走前4格
    TEST_SEQ1_SEARCH,       // 第5格：搜路口
    TEST_SEQ1_OFFSET,       // 搜到后：补尝进中
    
    // --- 路径动作 ---
    TEST_SEQ2_TURN_L_90,    // 左转
    TEST_SEQ3_FWD_1,        // 直行1格
    TEST_SEQ4_TURN_L_90,    // 左转
    TEST_SEQ5_FWD_1,        // 直行1格
    TEST_SEQ6_TURN_R_90,    // 右转
    TEST_SEQ7_FWD_3,        // 直行3格
    
    TEST_DONE               // 结束
} HardcodedState_t;

extern volatile int32_t  g_maze_ticks;
extern MazeState_t       g_maze_state;

// 经过滤波后的距离（mm）
extern uint8_t g_front_mm_filt;
extern uint8_t g_left_mm_filt;
extern uint8_t g_right_mm_filt;

// 是否有墙（根据阈值判断后）
extern uint8_t g_wall_front;   // 1=有墙, 0=无墙
extern uint8_t g_wall_left;
extern uint8_t g_wall_right;

// ========== 接口函数 ==========

// 编码器 -> 里程计
void Maze_Odom_Reset(void);
void Maze_Odom_Update(int16_t deltaA, int16_t deltaB);

// ToF 更新（在主循环里读完原始距离后调用）
void Maze_OnTofUpdate(uint8_t d_front, uint8_t d_left, uint8_t d_right);

// 启动迷宫模式（比如按键触发时调用）
void Maze_Start(void);

// 迷宫主任务（在 while(1) 里周期性调用）
void Maze_Task(void);

void Maze_Hardcoded_Start(void);
void Maze_Hardcoded_Task(void);
void Maze_Test_OneCell_Start(void);

#endif
