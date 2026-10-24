#include "board_com.h"
#include "string.h"

// 声明外部 CAN 句柄
extern CAN_HandleTypeDef hcan1;

// 定义全局双板通信数据结构体变量
BoardA_to_BoardC_t board_a_tx_data; 
BoardC_to_BoardA_t board_c_tx_data; 

/**
  * @brief          双板数据初始化，设定安全默认值
  */
void Board_Com_Init(void)
{
    // 将所有数据清零
    memset(&board_a_tx_data, 0, sizeof(BoardA_to_BoardC_t));
    memset(&board_c_tx_data, 0, sizeof(BoardC_to_BoardA_t));
    
    // 遥控器通道中位值为 0
    board_a_tx_data.ch_yaw = 0;
    board_a_tx_data.ch_pitch = 0;
    
    // 大疆 DR16 拨码开关默认安全值设置为 2 (下档)
    board_a_tx_data.s0 = 2; 
    board_a_tx_data.s1 = 2;
    board_a_tx_data.robot_mode = 0; 
    
    // C板反馈数据初始化
    board_c_tx_data.yaw_relative_ecd = 0;
    board_c_tx_data.vx = 0; 
    board_c_tx_data.vy = 0;
    board_c_tx_data.wz = 0;
}

// 供 C 板调用的获取接收数据接口
BoardA_to_BoardC_t BoardC_Get_RxData(void)
{
    return board_a_tx_data;
}

// 供 A 板调用的获取反馈数据接口
BoardC_to_BoardA_t BoardA_Get_RxData(void)
{
    return board_c_tx_data;
}

//  调用的数据更新接口
void Board_Com_Rx_Update(uint32_t stdId, uint8_t *rx_buf)
{
    if (stdId == 0x301)
    {
        memcpy(&board_a_tx_data, rx_buf, sizeof(BoardA_to_BoardC_t));
    }
    else if (stdId == 0x302)
    {
        memcpy(&board_c_tx_data, rx_buf, sizeof(BoardC_to_BoardA_t));
    }
}

//  调用的回传数据设置接口
void BoardC_Set_TxData(int16_t yaw_ecd)
{
    board_c_tx_data.yaw_relative_ecd = yaw_ecd;
}

// 供 A 板发送数据
HAL_StatusTypeDef CAN_Send_A_to_C(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    if (hcan1.State != HAL_CAN_STATE_LISTENING) return HAL_ERROR;
    memcpy(tx_data, &board_a_tx_data, sizeof(BoardA_to_BoardC_t));
    tx_header.StdId = 0x301;             
    tx_header.IDE = CAN_ID_STD;          
    tx_header.RTR = CAN_RTR_DATA;        
    tx_header.DLC = sizeof(BoardA_to_BoardC_t); 
    tx_header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

// 供 C 板发送数据
HAL_StatusTypeDef CAN_Send_C_to_A(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    if (hcan1.State != HAL_CAN_STATE_LISTENING) return HAL_ERROR;
    memcpy(tx_data, &board_c_tx_data, sizeof(BoardC_to_BoardA_t));
    tx_header.StdId = 0x302;             
    tx_header.IDE = CAN_ID_STD;          
    tx_header.RTR = CAN_RTR_DATA;        
    tx_header.DLC = sizeof(BoardC_to_BoardA_t); 
    tx_header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}
