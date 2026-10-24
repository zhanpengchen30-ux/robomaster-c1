#include "bsp_can.h"
#include "motor.h"    
#include "board_com.h"

extern CAN_HandleTypeDef hcan1;

// C 板的 CAN1 过滤器：同时接收电机反馈和 A 板指令
void CAN_Filter_Init(void) {
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.FilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// C 板的 CAN 中断：只解析云台和 A 板指令
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (rx_header.StdId == 0x205) { decode_6020_measure(&motor_yaw, rx_data); }
    else if (rx_header.StdId == 0x206) { decode_6020_measure(&motor_pitch, rx_data); }
    else if (rx_header.StdId == 0x301) { Board_Com_Rx_Update(0x301, rx_data); } // 接收 A 板遥控
}

// C 板发送：云台电压指令 (CAN1, ID 0x1FE)
// C 板发送：云台电流控制指令 (CAN1, ID: 0x1FE)
void CAN_Send_Gimbal_Voltage(int16_t yaw_v, int16_t pitch_v) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t send_mail_box;
    
    tx_header.StdId = 0x1FE; //  0x1FE 控制标识符，对 GM6020 进行转矩电流控制，避免总线冲突
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_data[0] = yaw_v >> 8; tx_data[1] = yaw_v;
    tx_data[2] = pitch_v >> 8; tx_data[3] = pitch_v;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}
