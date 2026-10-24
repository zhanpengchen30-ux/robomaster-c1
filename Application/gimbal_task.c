#include "gimbal_task.h"
#include "cmsis_os.h"
#include "controller.h" 
#include "motor.h"
#include "bsp_can.h"
#include "board_com.h"
#include "ins_task.h"  

// 声明外部底层发送函数和全局绝对姿态结构体
extern void CAN_Send_Gimbal_Voltage(int16_t yaw_v, int16_t pitch_v);
extern INS_t INS;       

// 绝对角度闭环核心算法：使用 360 度的圆弧误差计算
float circle_error(float target, float measure, float total)
{
    float error = target - measure;
    if (error > total / 2.0f)  error -= total;
    else if (error < -total / 2.0f) error += total;
    return error;
}

 static uint8_t last_robot_mode = MODE_RELAX;

// 定义双环 PID 实例 
static PID_t yaw_pos_pid; 
static PID_t yaw_spd_pid; 
static PID_t pitch_pos_pid; 
static PID_t pitch_spd_pid; 

// 绝对控制下推荐的 PID 参数
volatile float YAW_P_KP = 15.0f;     
volatile float YAW_S_KP = 100.0f;   
volatile float YAW_S_KI = 1.0f;     

volatile float PIT_P_KP = 7.0f;     
volatile float PIT_S_KP = 25.0f;    
volatile float PIT_S_KI = 0.8f;     

// 观察变量
float debug_pos_out;   // Yaw 位置环输出目标速度
float debug_final_v;   // Yaw 速度环最终控制电流
float debug_pitch_pos_out; // Pitch 位置环输出
float debug_pitch_final_v; // Pitch 最终控制电流

// 绝对控制目标值，单位：度 (°)
float yaw_target_angle = 0;   
float pitch_target_angle = 0; 

void StartGimbalTask(void *argument) {
    // 初始化所有 PID 
    PID_Init(&yaw_pos_pid, 800.0f, 0.0f, 0.0f, YAW_P_KP, 0.0f, 0.0f, 0, 0, 0, 0, 0, NONE);
    PID_Init(&yaw_spd_pid, 16384.0f, 5000.0f, 0.0f, YAW_S_KP, YAW_S_KI, 0.0f, 0, 0, 0, 0, 0, Integral_Limit);
    
    PID_Init(&pitch_pos_pid, 800.0f, 0.0f, 0.0f, PIT_P_KP, 0.0f, 0.0f, 0, 0, 0, 0, 0, NONE);
    PID_Init(&pitch_spd_pid, 16384.0f, 5000.0f, 0.0f, PIT_S_KP, PIT_S_KI, 0.0f, 0, 0, 0, 0, 0, Integral_Limit);

    osDelay(200); 
    // 初始目标角度对齐当前陀螺仪开机绝对角度（度）
    yaw_target_angle = INS.Yaw;
     //    pitch_target_angle = INS.Roll ;
     pitch_target_angle = 0.0f; // 0 度
    for(;;) {
			
        // 从双板通信中获取 A 板打包好的指令
        BoardA_to_BoardC_t cmd = BoardC_Get_RxData();

        yaw_pos_pid.Kp = YAW_P_KP;
        yaw_spd_pid.Kp = YAW_S_KP;
        yaw_spd_pid.Ki = YAW_S_KI;
        
        pitch_pos_pid.Kp = PIT_P_KP;
        pitch_spd_pid.Kp = PIT_S_KP;
        pitch_spd_pid.Ki = PIT_S_KI;
 			
        // 状态判断
        if (cmd.robot_mode == MODE_GIMBAL_ONLY || cmd.robot_mode == MODE_FOLLOW || 
            cmd.robot_mode == MODE_SPIN || cmd.robot_mode == MODE_CHASSIS_ONLY) {
            
							 //如果刚才处于失能(RELAX)，现在刚切到工作模式，瞬间将 Pitch 归零（水平）
            if (last_robot_mode == MODE_RELAX) {
                yaw_target_angle = INS.Yaw;   // Yaw 保持当前朝向
                pitch_target_angle = 0.0f;    // Pitch 归位到水平 0 度
            }
							
            if (cmd.robot_mode != MODE_CHASSIS_ONLY) {
                
                yaw_target_angle -= (float)cmd.ch_yaw * 0.0005f; 
                pitch_target_angle += (float)cmd.ch_pitch * 0.0003f;
            }
            
            // Yaw 轴绝对角度范围约束 (-180.0° 到 180.0°)
            if (yaw_target_angle > 180.0f)  yaw_target_angle -= 360.0f;
            if (yaw_target_angle < -180.0f) yaw_target_angle += 360.0f;

            pitch_target_angle = float_constrain(pitch_target_angle, -40.0f, 40.0f);

            debug_pos_out = yaw_pos_pid.Kp * circle_error(yaw_target_angle, INS.Yaw, 360.0f);
            
            // 限制位置环输出的最大目标转速
            debug_pos_out = float_constrain(debug_pos_out, -yaw_pos_pid.MaxOut, yaw_pos_pid.MaxOut);
            debug_final_v = PID_Calculate(&yaw_spd_pid, (float)motor_yaw.speed_rpm, debug_pos_out);
            
            debug_pitch_pos_out = pitch_pos_pid.Kp * circle_error(pitch_target_angle, INS.Roll, 360.0f);
            
            //  对 Pitch 位置环进行同样的限幅限制
            debug_pitch_pos_out = float_constrain(debug_pitch_pos_out, -pitch_pos_pid.MaxOut, pitch_pos_pid.MaxOut);
            debug_pitch_final_v = PID_Calculate(&pitch_spd_pid, (float)motor_pitch.speed_rpm, debug_pitch_pos_out);
            
            // 双轴电流合并发送（0x1FE 控制帧）
            CAN_Send_Gimbal_Voltage((int16_t)debug_final_v, (int16_t)debug_pitch_final_v);
        } 
        else {
            // 如果失能，向两个电机均发送 0 电流
            CAN_Send_Gimbal_Voltage(0, 0);
            yaw_target_angle = INS.Yaw; 
            pitch_target_angle = INS.Roll; 
        }

        // 打包小陀螺和平移速度数据回传给 A 板
        if (cmd.robot_mode == MODE_SPIN) {
            board_c_tx_data.wz = 3000;              
            board_c_tx_data.vx = cmd.ch_pitch * 10; 
            board_c_tx_data.vy = cmd.ch_yaw * 10;   
        } else {
            board_c_tx_data.wz = 0;
            board_c_tx_data.vx = 0;
            board_c_tx_data.vy = 0;
        }

        // 反馈云台当前机械角度给底盘做跟随基准
        extern motor_6020_t motor_yaw; 
        board_c_tx_data.yaw_relative_ecd = motor_yaw.ecd;
        
        // 执行双板发送包 (0x302) 传回 A 板
        CAN_Send_C_to_A();
				
				last_robot_mode = cmd.robot_mode;

        osDelay(1); 
    }
}
