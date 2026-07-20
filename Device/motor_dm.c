#include "motor_dm.h"

DM_Motor dm_motor;

uint8_t DM_Start_Cmd[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
uint8_t DM_Stop_Cmd[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
uint8_t DM_Save_Zone_Cmd[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
uint8_t DM_Clear_Error_Cmd[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};

uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;

  if (x > x_max)
  {
    x = x_max;
  }
  else if (x < x_min)
  {
    x = x_min;
  }

  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void CAN2_Motor_DM_Pos_Speed(uint32_t ID,float p_des, float v_des)
{
  FDCAN_TxHeaderTypeDef motor_tx_message = {0};

  uint8_t* p_des_bytes = (uint8_t*)&p_des;
  uint8_t* v_des_bytes = (uint8_t*)&v_des;
  uint8_t send_data_DM[8];

  motor_tx_message.Identifier = 0x100 + ID;
  motor_tx_message.IdType = FDCAN_STANDARD_ID;
  motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  motor_tx_message.MessageMarker = 0;

  send_data_DM[0] = p_des_bytes[0];
  send_data_DM[1] = p_des_bytes[1];
  send_data_DM[2] = p_des_bytes[2];
  send_data_DM[3] = p_des_bytes[3];
  send_data_DM[4] = v_des_bytes[0];
  send_data_DM[5] = v_des_bytes[1];
  send_data_DM[6] = v_des_bytes[2];
  send_data_DM[7] = v_des_bytes[3];

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &motor_tx_message, send_data_DM) != HAL_OK)
  {
    return;
  }
}

void CAN3_Motor_DM_Pos_Speed_4310(uint32_t ID,float p_des, float v_des)
{
  FDCAN_TxHeaderTypeDef motor_tx_message = {0};

  uint8_t* p_des_bytes = (uint8_t*)&p_des;
  uint8_t* v_des_bytes = (uint8_t*)&v_des;
  uint8_t send_data_DM[8];

  motor_tx_message.Identifier = 0x100 + ID;
  motor_tx_message.IdType = FDCAN_STANDARD_ID;
  motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  motor_tx_message.MessageMarker = 0;

  send_data_DM[0] = p_des_bytes[0];
  send_data_DM[1] = p_des_bytes[1];
  send_data_DM[2] = p_des_bytes[2];
  send_data_DM[3] = p_des_bytes[3];
  send_data_DM[4] = v_des_bytes[0];
  send_data_DM[5] = v_des_bytes[1];
  send_data_DM[6] = v_des_bytes[2];
  send_data_DM[7] = v_des_bytes[3];

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &motor_tx_message, send_data_DM) != HAL_OK)
  {
    return;
  }
}


void CAN2_Motor_DM_Pos_Speed_Current(uint32_t ID,float p_des,float v_des,float i_des)
{
  FDCAN_TxHeaderTypeDef motor_tx_message = {0};

  uint8_t* p_des_bytes = (uint8_t*)&p_des;
  uint8_t send_data_DM[8];
  uint16_t v_uint;
  uint16_t i_uint;

  //放大100倍再发送，范围是0~10000
  if (v_des < 0.0f) v_des = 0.0f;
  else if (v_des > 100.0f) v_des = 100.0f;
  //放大10000倍再发送，范围是0~10000
  if (i_des < 0.0f) i_des = 0.0f;
  else if (i_des > 1.0f) i_des = 1.0f;

  v_uint = (uint16_t)(v_des * 100.0f);
  i_uint = (uint16_t)(i_des * 10000.0f);

  motor_tx_message.Identifier = 0x300 + ID;
  motor_tx_message.IdType = FDCAN_STANDARD_ID;
  motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  motor_tx_message.MessageMarker = 0;

  send_data_DM[0] = p_des_bytes[0];
  send_data_DM[1] = p_des_bytes[1];
  send_data_DM[2] = p_des_bytes[2];
  send_data_DM[3] = p_des_bytes[3];
  send_data_DM[4] = v_uint;
  send_data_DM[5] = v_uint >> 8;
  send_data_DM[6] = i_uint;
  send_data_DM[7] = i_uint >> 8;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &motor_tx_message, send_data_DM) != HAL_OK)
  {
    return;
  }
}

/*
    pos12位，speed12位，kp12位，kd12位，t_ff12位
    Kp:[0,500], Kd:[0,5]，p和v的范围在调试助手设定，根据这个来调整
    输出力矩 = kp * 位置误差 + kd * 速度误差 + t_ff
    pos:rad
    v:rad/s
    t_ff:N*m
*/
void CAN2_Motor_DM_MIT(uint32_t ID,float p_des,float v_des,float kp,float kd,float t_ff,float P_MIN,float P_MAX,float V_MIN,float V_MAX,float T_MIN,float T_MAX)
{
  FDCAN_TxHeaderTypeDef motor_tx_message = {0};

  uint8_t send_data_DM[8];

  uint16_t p_uint;
  uint16_t v_uint;
  uint16_t kp_uint;
  uint16_t kd_uint;
  uint16_t t_uint;

  p_uint = float_to_uint(p_des, P_MIN, P_MAX, 16);
  v_uint = float_to_uint(v_des, V_MIN, V_MAX, 12);
  kp_uint = float_to_uint(kp, 0.0f, 500.0f, 12);
  kd_uint = float_to_uint(kd, 0.0f, 5.0f, 12);
  t_uint = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  motor_tx_message.Identifier = ID;
  motor_tx_message.IdType = FDCAN_STANDARD_ID;
  motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  motor_tx_message.MessageMarker = 0;

  send_data_DM[0] = (uint8_t)(p_uint >> 8);
  send_data_DM[1] = (uint8_t)(p_uint);
  send_data_DM[2] = (uint8_t)(v_uint >> 4);
  send_data_DM[3] = (uint8_t)(((v_uint & 0x0F) << 4) | (kp_uint >> 8));
  send_data_DM[4] = (uint8_t)(kp_uint);
  send_data_DM[5] = (uint8_t)(kd_uint >> 4);
  send_data_DM[6] = (uint8_t)(((kd_uint & 0x0F) << 4) | (t_uint >> 8));
  send_data_DM[7] = (uint8_t)(t_uint);

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &motor_tx_message, send_data_DM) != HAL_OK)
  {
    return;
  }
}

void CAN2_Motor_DM_Write_Reg(uint32_t motor_id, uint8_t rid, uint32_t value)
{
  FDCAN_TxHeaderTypeDef motor_tx_message = {0};
  uint8_t send_data[8];

  motor_tx_message.Identifier = 0x7FF;//写入参数和读取参数的报文ID
  motor_tx_message.IdType = FDCAN_STANDARD_ID;
  motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  motor_tx_message.MessageMarker = 0;

  send_data[0] = motor_id & 0xFF;
  send_data[1] = (motor_id >> 8) & 0xFF;
  send_data[2] = 0x55;
  send_data[3] = rid;

  send_data[4] = value & 0xFF;
  send_data[5] = (value >> 8) & 0xFF;
  send_data[6] = (value >> 16) & 0xFF;
  send_data[7] = (value >> 24) & 0xFF;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &motor_tx_message, send_data) != HAL_OK)
  {
    return;
  }
}

void CAN2_Motor_DM_Save_Reg(uint32_t motor_id)
{
  FDCAN_TxHeaderTypeDef motor_tx_message = {0};
  uint8_t send_data[8] = {0};

  motor_tx_message.Identifier = 0x7FF;//保存参数的报文ID
  motor_tx_message.IdType = FDCAN_STANDARD_ID;
  motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  motor_tx_message.MessageMarker = 0;

  send_data[0] = motor_id & 0xFF;
  send_data[1] = (motor_id >> 8) & 0xFF;
  send_data[2] = 0xAA;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &motor_tx_message, send_data) != HAL_OK)
  {
    return;
  }
}

void CAN2_Motor_DM_Feedback_Update(DM_Motor* dm_motor, uint8_t *rx_data)
{
    DM_Motor_msg* msg;
    uint8_t id;

    if(dm_motor == NULL || rx_data == NULL)
    {
        return;
    }

    id = rx_data[0] & 0x0F;

    if (id == DM4310_ID) msg = &dm_motor->DM4310_ID4;
    else if (id == DM4340p1_ID) msg = &dm_motor->DM4340p1_ID1;
    else if (id == DM4340p2_ID) msg = &dm_motor->DM4340p2_ID3;
    else if (id == DM10010L_ID) msg = &dm_motor->DM10010L_ID2;
    else return;

    msg->ID = id;
    msg->ERR = (rx_data[0] >> 4) & 0x0F;
    msg->position = ((uint16_t)rx_data[1] << 8) | rx_data[2];
    msg->speed = ((uint16_t)rx_data[3] << 4) | (rx_data[4] >> 4);
    msg->torque = ((uint16_t)(rx_data[4] & 0x0F) << 8) | rx_data[5];
    msg->T_Mos = rx_data[6];
    msg->T_Rotor = rx_data[7];
}


void CAN2_Motor_DM_Write_Reg_Float(uint32_t motor_id, uint8_t rid, float value)
{
  uint32_t value_u32;
  memcpy(&value_u32, &value, sizeof(float));

  CAN2_Motor_DM_Write_Reg(motor_id, rid, value_u32);
}

void CAN2_Motor_DM_Write_Reg_Uint32(uint32_t motor_id, uint8_t rid, uint32_t value)
{
  CAN2_Motor_DM_Write_Reg(motor_id, rid, value);
}

void DM_Set_Mode(uint32_t id, uint8_t mode)//设置控制模式
{
  if (mode < 1 || mode > 4)
  {
    return;
  }

  CAN2_Motor_DM_Write_Reg_Uint32(id, DM_REG_CTRL_MODE, mode);
}

void DM_Set_AccDec(uint32_t id, float acc, float dec)//设置加速度和减速度
{
  if (acc <= 0.0f || dec <= 0.0f)
  {
    return;
  }

  CAN2_Motor_DM_Write_Reg_Float(id, DM_REG_ACC, acc);
  HAL_Delay(5);
  CAN2_Motor_DM_Write_Reg_Float(id, DM_REG_DEC, -dec);
}

void DM_Set_MaxSpeed(uint32_t id, float max_speed)//设置最大速度
{
  if (max_speed <= 0.0f)
  {
    return;
  }

  CAN2_Motor_DM_Write_Reg_Float(id, DM_REG_MAX_SPD, max_speed);
}

void DM_Set_Timeout(uint32_t id, uint32_t timeout)//设置超时报警时间
{
  CAN2_Motor_DM_Write_Reg_Uint32(id, DM_REG_TIMEOUT, timeout);
}

void DM_Set_CurrentLimit(uint32_t id, float oc_value)//设置过流保护值
{
  if (oc_value <= 0.0f || oc_value >= 1.0f)
  {
    return;
  }

  CAN2_Motor_DM_Write_Reg_Float(id, DM_REG_OC_VALUE, oc_value);
}

void DM_Set_TempLimit(uint32_t id, float temp_limit)//设置过热保护值
{
  if (temp_limit < 80.0f || temp_limit >= 200.0f)
  {
    return;
  }

  CAN2_Motor_DM_Write_Reg_Float(id, DM_REG_OT_VALUE, temp_limit);
}

void DM_Save_Params(uint32_t id)//保存参数到电机非易失性存储器
{
  CAN2_Motor_DM_Save_Reg(id);
}

void CAN2_Motor_DM_Commom_Cmd(uint32_t ID,uint8_t* cmd)//发送命令的对应的ID与当前的控制模式的控制ID有关
{
    FDCAN_TxHeaderTypeDef motor_tx_message = {0};
    
    motor_tx_message.Identifier = ID;
    motor_tx_message.IdType = FDCAN_STANDARD_ID;
    motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    motor_tx_message.MessageMarker = 0;
    
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
    {
        return;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &motor_tx_message, cmd) != HAL_OK)
    {
        return;
    }
}

void CAN3_Motor_DM_Commom_Cmd(uint32_t ID,uint8_t* cmd)
{
    FDCAN_TxHeaderTypeDef motor_tx_message = {0};
    
    motor_tx_message.Identifier = ID;
    motor_tx_message.IdType = FDCAN_STANDARD_ID;
    motor_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    motor_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    motor_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    motor_tx_message.MessageMarker = 0;
    
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0U)
    {
        return;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &motor_tx_message, cmd) != HAL_OK)
    {
        return;
    }
}

void DM_Start(uint8_t mode,uint32_t ID)
{
    switch (mode)
    {
    case 1:
        CAN2_Motor_DM_Commom_Cmd(ID,DM_Start_Cmd);
        break;
    case 2:
        CAN2_Motor_DM_Commom_Cmd(ID+0x100,DM_Start_Cmd);
        break;
    case 3:
        CAN2_Motor_DM_Commom_Cmd(ID+0x200,DM_Start_Cmd);
        break;
    case 4:
        CAN2_Motor_DM_Commom_Cmd(ID+0x300,DM_Start_Cmd);
        break;
    default:
        break;
    }
    
}

void DM_Stop(uint8_t mode,uint32_t ID)
{
    switch (mode)
    {
    case 1:
        CAN2_Motor_DM_Commom_Cmd(ID,DM_Stop_Cmd);
        break;
    case 2:
        CAN2_Motor_DM_Commom_Cmd(ID+0x100,DM_Stop_Cmd);
        break;
    case 3:
        CAN2_Motor_DM_Commom_Cmd(ID+0x200,DM_Stop_Cmd);
        break;
    case 4:
        CAN2_Motor_DM_Commom_Cmd(ID+0x300,DM_Stop_Cmd);
        break;
    default:
        break;
    }
}

void DM_Save_Zone(uint8_t mode,uint32_t ID)
{
    switch (mode)
    {
    case 1:
        CAN2_Motor_DM_Commom_Cmd(ID,DM_Save_Zone_Cmd);
        break;
    case 2:
        CAN2_Motor_DM_Commom_Cmd(ID+0x100,DM_Save_Zone_Cmd);
        break;
    case 3:
        CAN2_Motor_DM_Commom_Cmd(ID+0x200,DM_Save_Zone_Cmd);
        break;
    case 4:
        CAN2_Motor_DM_Commom_Cmd(ID+0x300,DM_Save_Zone_Cmd);
        break;
    default:
        break;
    }
}

void DM_Clear_Error(uint8_t mode,uint32_t ID)
{
    switch (mode)
    {
    case 1:
        CAN2_Motor_DM_Commom_Cmd(ID,DM_Clear_Error_Cmd);
        break;
    case 2:
        CAN2_Motor_DM_Commom_Cmd(ID+0x100,DM_Clear_Error_Cmd);
        break;
    case 3:
        CAN2_Motor_DM_Commom_Cmd(ID+0x200,DM_Clear_Error_Cmd);
        break;
    case 4:
        CAN2_Motor_DM_Commom_Cmd(ID+0x300,DM_Clear_Error_Cmd);
        break;
    default:
        break;
    }
}

