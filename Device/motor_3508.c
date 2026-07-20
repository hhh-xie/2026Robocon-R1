#include "motor_3508.h"

speed_ID motor_speed;
angle_ID motor_angle;

M3508 motor_3508;

void CAN1_Motor_3508(uint32_t ID,int16_t motor1_i, int16_t motor2_i, int16_t motor3_i, int16_t motor4_i)
{
  FDCAN_TxHeaderTypeDef motor_tx_message = {0};

  uint8_t send_data[8];
  motor_tx_message.Identifier = ID;
  motor_tx_message.IdType = FDCAN_STANDARD_ID;
  motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  motor_tx_message.MessageMarker = 0;

  send_data[0] = (uint8_t)(motor1_i >> 8);
  send_data[1] = (uint8_t)(motor1_i & 0x00FF);
  send_data[2] = (uint8_t)(motor2_i >> 8);
  send_data[3] = (uint8_t)(motor2_i & 0x00FF);
  send_data[4] = (uint8_t)(motor3_i >> 8);
  send_data[5] = (uint8_t)(motor3_i & 0x00FF);
  send_data[6] = (uint8_t)(motor4_i >> 8);
  send_data[7] = (uint8_t)(motor4_i & 0x00FF);

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &motor_tx_message, send_data) != HAL_OK)
  {
    return;
  }
}

void CAN2_Motor_3508(uint32_t ID,int16_t motor1_i, int16_t motor2_i, int16_t motor3_i, int16_t motor4_i)
{
  FDCAN_TxHeaderTypeDef motor_tx_message = {0};
  
  uint8_t send_data[8];
  motor_tx_message.Identifier = ID;
  motor_tx_message.IdType = FDCAN_STANDARD_ID;
  motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  motor_tx_message.MessageMarker = 0;

  send_data[0] = (uint8_t)(motor1_i >> 8);
  send_data[1] = (uint8_t)(motor1_i & 0x00FF);
  send_data[2] = (uint8_t)(motor2_i >> 8);
  send_data[3] = (uint8_t)(motor2_i & 0x00FF);
  send_data[4] = (uint8_t)(motor3_i >> 8);
  send_data[5] = (uint8_t)(motor3_i & 0x00FF);
  send_data[6] = (uint8_t)(motor4_i >> 8);
  send_data[7] = (uint8_t)(motor4_i & 0x00FF);

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &motor_tx_message, send_data) != HAL_OK)
  {
    return;
  }
}

void CAN3_Motor_3508(uint32_t ID,int16_t motor1_i, int16_t motor2_i, int16_t motor3_i, int16_t motor4_i)
{
  FDCAN_TxHeaderTypeDef motor_tx_message = {0};
  
  uint8_t send_data[8];
  motor_tx_message.Identifier = ID;
  motor_tx_message.IdType = FDCAN_STANDARD_ID;
  motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  motor_tx_message.MessageMarker = 0;

  send_data[0] = (uint8_t)(motor1_i >> 8);
  send_data[1] = (uint8_t)(motor1_i & 0x00FF);
  send_data[2] = (uint8_t)(motor2_i >> 8);
  send_data[3] = (uint8_t)(motor2_i & 0x00FF);
  send_data[4] = (uint8_t)(motor3_i >> 8);
  send_data[5] = (uint8_t)(motor3_i & 0x00FF);
  send_data[6] = (uint8_t)(motor4_i >> 8);
  send_data[7] = (uint8_t)(motor4_i & 0x00FF);

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &motor_tx_message, send_data) != HAL_OK)
  {
    return;
  }
}

void CAN1_Receive_Motor(M3508_msg *motor_msg,uint8_t *receive_data)
{
  motor_msg->ecd = ((int16_t)receive_data[0] << 8) | receive_data[1];
  motor_msg->speed = ((int16_t)receive_data[2] << 8) | receive_data[3];
  motor_msg->current = ((int16_t)receive_data[4] << 8) | receive_data[5];
  motor_msg->temperature = receive_data[6];
  get_total_3508_angle(motor_msg);
}

void CAN2_Receive_Motor(M3508_msg *motor_msg,uint8_t *receive_data)
{
  motor_msg->ecd = ((int16_t)receive_data[0] << 8) | receive_data[1];
  motor_msg->speed = ((int16_t)receive_data[2] << 8) | receive_data[3];
  motor_msg->current = ((int16_t)receive_data[4] << 8) | receive_data[5];
  motor_msg->temperature = receive_data[6];
  get_total_3508_angle(motor_msg);
}

void CAN3_Receive_Motor(M3508_msg *motor_msg,uint8_t *receive_data)
{
  motor_msg->ecd = ((int16_t)receive_data[0] << 8) | receive_data[1];
  motor_msg->speed = ((int16_t)receive_data[2] << 8) | receive_data[3];
  motor_msg->current = ((int16_t)receive_data[4] << 8) | receive_data[5];
  motor_msg->temperature = receive_data[6];
  get_total_3508_angle(motor_msg);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef motor_rx_message;
  uint8_t receive_data[8];
    (void)RxFifo0ITs;

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &motor_rx_message, receive_data);
    if(hfdcan->Instance == FDCAN1)//麦轮底盘的四个3508电机
    {
        switch (motor_rx_message.Identifier)
        {
            case 0x201:
            CAN1_Receive_Motor(&motor_3508.ID1, receive_data);
            break;
            case 0x202:
            CAN1_Receive_Motor(&motor_3508.ID2, receive_data);
            break;
            case 0x203:
            CAN1_Receive_Motor(&motor_3508.ID3, receive_data);
            break;
            case 0x204:
            CAN1_Receive_Motor(&motor_3508.ID4, receive_data);
            break;
            default:
            break;
        }
    }
    else if(hfdcan->Instance == FDCAN2)//机械臂的达妙电机
    {
        if(motor_rx_message.Identifier == DMReturn_ID)
        {
            CAN2_Motor_DM_Feedback_Update(&dm_motor, receive_data);
        }
    }
    else if(hfdcan->Instance == FDCAN3)//抬升和夹爪旋转的3508电机
    {
        switch (motor_rx_message.Identifier)
        {
            case DMReturn_ID:
            CAN2_Motor_DM_Feedback_Update(&dm_motor, receive_data);
            break;
            case 0x205:
            CAN3_Receive_Motor(&motor_3508.ID5, receive_data);
            break;
            case 0x206:
            CAN3_Receive_Motor(&motor_3508.ID6, receive_data);
            break;
            case 0x207:
            CAN3_Receive_Motor(&motor_3508.ID7, receive_data);
            break;
            default:
            break;
        }
    }
}

void M3508_Speed_Contorl(void)
{
  motor_3508.ID1.current_out=PID_Speed_Calc_3508(&motor_3508.ID1,&motor_speed.M3508_ID1);
  motor_3508.ID2.current_out=PID_Speed_Calc_3508(&motor_3508.ID2,&motor_speed.M3508_ID2);
  motor_3508.ID3.current_out=PID_Speed_Calc_3508(&motor_3508.ID3,&motor_speed.M3508_ID3);
  motor_3508.ID4.current_out=PID_Speed_Calc_3508(&motor_3508.ID4,&motor_speed.M3508_ID4);
  CAN1_Motor_3508(0x200,(int16_t)motor_3508.ID1.current_out,(int16_t)motor_3508.ID2.current_out,(int16_t)motor_3508.ID3.current_out,(int16_t)motor_3508.ID4.current_out);
}

void M3508_Angle_Contorl(void)
{
  motor_3508.ID5.current_out=PID_Angle_Calc_3508(&motor_3508.ID5,&motor_angle.M3508_ID5);
  motor_3508.ID6.current_out=PID_Angle_Calc_3508(&motor_3508.ID6,&motor_angle.M3508_ID6);
  motor_3508.ID7.current_out=PID_Angle_Calc_3508(&motor_3508.ID7,&motor_angle.M3508_ID7);
  CAN3_Motor_3508(0x1FF,(int16_t)motor_3508.ID5.current_out,(int16_t)motor_3508.ID6.current_out,(int16_t)motor_3508.ID7.current_out,0);
}

void M3508_Chassis_Angle_Contorl(void)
{
  motor_3508.ID1.current_out = PID_Angle_Calc_3508(&motor_3508.ID1, &motor_angle.M3508_ID1);
  motor_3508.ID2.current_out = PID_Angle_Calc_3508(&motor_3508.ID2, &motor_angle.M3508_ID2);
  motor_3508.ID3.current_out = PID_Angle_Calc_3508(&motor_3508.ID3, &motor_angle.M3508_ID3);
  motor_3508.ID4.current_out = PID_Angle_Calc_3508(&motor_3508.ID4, &motor_angle.M3508_ID4);
  CAN1_Motor_3508(0x200,motor_3508.ID1.current_out,motor_3508.ID2.current_out,motor_3508.ID3.current_out,motor_3508.ID4.current_out);
}
