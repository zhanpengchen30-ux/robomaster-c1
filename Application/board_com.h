#ifndef __BOARD_COM_H
#define __BOARD_COM_H

#include <stdint.h> 
#include "can.h"   
#include "main.h"  

#pragma pack(1) // 强制 1 字节对齐

// ?? 将模式枚举直接放入双板通信头文件中，这样 A、C 两板都可以直接使用模式，不需要再依赖外部遥控器头文件！
typedef enum {
    MODE_RELAX = 0,       // 全车失能
    MODE_GIMBAL_ONLY,     // 只动云台
    MODE_SPIN,            // 小陀螺模式 (底盘自旋 + 平移) ?? 统一命名为 SPIN
    MODE_FOLLOW,          // 跟随模式 (底盘跟随云台)
    MODE_CHASSIS_ONLY     // 只动底盘 ?? 新增这一项！
} Robot_Mode_e;

// A板 -> C板：打包遥控器通道、开关以及计算好的机器人模式（共 8 字节）
typedef struct {
    int16_t ch_yaw;      // Yaw 控制通道 (对应 rc_ctrl.rc.ch[2])
    int16_t ch_pitch;    // Pitch 控制通道 (对应 rc_ctrl.rc.ch[3])
    uint8_t s0;          // 右拨杆状态 (对应 rc_ctrl.rc.s[0])
    uint8_t s1;          // 左拨杆状态 (对应 rc_ctrl.rc.s[1])
    uint16_t robot_mode; // A板计算出来的全局机器人模式 (Robot_Mode_e)
} BoardA_to_BoardC_t;

// C板 -> A板：同步云台电机反馈与底盘平移自旋指令（共 8 字节）
typedef struct {
    int16_t yaw_relative_ecd; // Yaw 轴电机的相对编码器值
    int16_t vx;               // 前后速度
    int16_t vy;               // 左右速度
    int16_t wz;               // 自旋速度 (小陀螺)
} BoardC_to_BoardA_t;

#pragma pack()

// 全局变量声明
extern BoardA_to_BoardC_t board_a_tx_data;
extern BoardC_to_BoardA_t board_c_tx_data;

// 函数声明
void Board_Com_Init(void);
HAL_StatusTypeDef CAN_Send_A_to_C(void);
HAL_StatusTypeDef CAN_Send_C_to_A(void);
BoardA_to_BoardC_t BoardC_Get_RxData(void);
void BoardC_Set_TxData(int16_t yaw_ecd);
void Board_Com_Rx_Update(uint32_t stdId, uint8_t *rx_buf);

#endif /* __BOARD_COM_H */
