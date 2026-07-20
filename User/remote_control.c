#include "remote_control.h"

#define RC_RX_BUF_SIZE 64U                                 //DMA串口接收缓冲去大小
#define CRSF_ADDRESS 0xC8U                                 //CRSF协议地址，0xC8是遥控器发给飞控的地址，0xC0是飞控发给遥控器的地址
#define CRSF_LEN_RC_CHANNELS 0x18U                         //CRSF协议中RC通道数据的长度，24字节
#define CRSF_TYPE_RC_CHANNELS_PACKED 0x16U                 //CRSF协议中RC通道数据的类型，0x16表示打包的RC通道数据
#define CRSF_RC_FRAME_SIZE (CRSF_LEN_RC_CHANNELS + 2U)     //CRSF协议中RC通道数据帧的总长度，包含地址、长度、类型和CRC校验
#define CRSF_MAX_FRAME_SIZE 64U                            //CRSF协议中最大帧长度，根据实际情况设置，通常不超过64字节
#define CRSF_CRC_POLY 0xD5U                                //CRSF协议中CRC8校验的多项式，0xD5是常用的CRC8多项式

#define RC_CODE12_CHANNEL_MIN 172
#define RC_CODE12_CHANNEL_MAX 1811
#define RC_CODE12_PACK3_MAX 63U
#define RC_CODE12_PACK3_TRIGGER_MAX 127U
#define RC_CODE12_TRIGGER_BIT 64U
volatile uint8_t rx_end_flag;
volatile uint16_t rx_len;
volatile uint32_t rc_rx_event_count;
volatile uint32_t rc_parse_ok_count;
volatile uint32_t rc_header_fail_count;
volatile uint32_t rc_crc_fail_count;
volatile uint32_t rc_uart_error_count;
volatile uint32_t rc_arm_ok_count;
volatile uint32_t rc_arm_fail_count;
volatile uint32_t rc_last_error_code;
volatile uint16_t rc_last_expected_size;
volatile uint8_t rc_last_arm_status;
volatile uint8_t rc_last_frame_len;
volatile uint8_t rc_last_frame_type;
volatile uint8_t rc_last_crc_calc;
volatile uint8_t rc_last_crc_recv;
volatile uint8_t rc_last_rx_data[8];
volatile uint8_t rc_code12_pack[4];
volatile uint8_t rc_code12_trigger;
static uint8_t rc_code12_last_trigger = 0xFFU;
static uint8_t rx_buff[RC_RX_BUF_SIZE];       
static uint8_t rx_parse_buff[RC_RX_BUF_SIZE];              //临时拷贝缓冲区，防止 DMA 重新接收时覆盖数据
static uint8_t crsf_frame[CRSF_MAX_FRAME_SIZE];            //CRSF协议帧缓冲区，存储当前正在解析的CRSF帧数据
static uint8_t crsf_index = 0U;                            //CRSF协议帧解析索引，指示当前已经接收了多少字节的CRSF帧数据
static uint16_t crsf_expected_size = 0U;                   //CRSF协议帧预期大小，根据CRSF协议中的长度字段计算得出，指示完整的CRSF帧应该有多少字节

volatile ELRS_t Elrs;
volatile uint8_t rc_rx_started = 0U;                       //标志位，指示DMA串口接收是否已经启动，0表示未启动，1表示已启动

static uint8_t RC_Calc_Crc8(const uint8_t *data, uint8_t len)//CRC校验
{
  uint8_t crc = 0U;

  while (len-- > 0U)
  {
    crc ^= *data++;
    for (uint8_t i = 0U; i < 8U; i++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1U) ^ CRSF_CRC_POLY);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }

  return crc;
}

static void RC_Arm_DMA(void)//启动DMA串口接收，准备接收CRSF协议数据
{
  //清除UART错误标志
  __HAL_UART_CLEAR_PEFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  __HAL_UART_CLEAR_IDLEFLAG(&huart2);

  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, sizeof(rx_buff));
  rc_last_arm_status = (uint8_t)status;
  rc_last_error_code = huart2.ErrorCode;

  if (status == HAL_OK)
  {
    if (huart2.hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);//关闭DMA半传输中断，避免在接收过程中触发半传输中断，导致数据处理混乱
    }
    rc_rx_started = 1U;
    rc_arm_ok_count++;
  }
  else
  {
    rc_rx_started = 0U;
    rc_arm_fail_count++;
  }
}

static void RC_Reset_Crsf_Parser(void)//重置CRSF解析器
{
  crsf_index = 0U;
  crsf_expected_size = 0U;
}

static void RC_Process_RxByte(uint8_t byte)//处理接收到的每个字节，解析CRSF协议帧
{
  if (crsf_index == 0U)
  {
    if (byte == CRSF_ADDRESS)//识别到地址则判断开始接收CRSF帧，否则继续等待地址字节
    {
      crsf_frame[crsf_index++] = byte;
    }
    return;
  }

  crsf_frame[crsf_index++] = byte;

  if (crsf_index == 2U)
  {
    crsf_expected_size = (uint16_t)byte + 2U;//计算出完整帧的大小，2是指前面的两个字节
    rc_last_expected_size = crsf_expected_size;
    if ((byte < 2U) || (crsf_expected_size > CRSF_MAX_FRAME_SIZE))
    {
      RC_Reset_Crsf_Parser();
    }
    return;
  }

  if (crsf_index >= crsf_expected_size)
  {
    if (RC_Receive_Data(crsf_frame) != 0U)//接收到完整的CRSF帧，进行数据处理，如果数据处理成功则设置接收完成标志位，否则丢弃数据并重置
    {
      rx_end_flag = 1U;
    }
    RC_Reset_Crsf_Parser();
  }
}

static void RC_Process_RxBuffer(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0U; i < len; i++)
  {
    RC_Process_RxByte(data[i]);
  }
}

void RC_Receive(void)
{
  if (rc_rx_started == 0U)
  {
    RC_Arm_DMA();
  }

  if (rx_end_flag != 0U)
  {
    rx_end_flag = 0U;
  }
}

void Data_Change(int16_t *value)
{
  if (*value < 594)
  {
    *value = 1;
  }
  else if (*value < 1395)
  {
    *value = 2;
  }
  else
  {
    *value = 3;
  }
}

void Channel_optimization_key(int16_t *value)
{
  if (*value > 992)
  {
    *value = 1;
  }
  else
  {
    *value = 0;
  }
}

static uint8_t RC_Code12_Channel_To_Pack(int16_t value, uint8_t max_pack)
{
  int32_t pack;

  if (value < RC_CODE12_CHANNEL_MIN)
  {
    value = RC_CODE12_CHANNEL_MIN;
  }
  else if (value > RC_CODE12_CHANNEL_MAX)
  {
    value = RC_CODE12_CHANNEL_MAX;
  }

  pack = ((int32_t)(value - RC_CODE12_CHANNEL_MIN) * (int32_t)max_pack)
       + ((RC_CODE12_CHANNEL_MAX - RC_CODE12_CHANNEL_MIN) / 2);
  pack /= (RC_CODE12_CHANNEL_MAX - RC_CODE12_CHANNEL_MIN);

  return (uint8_t)pack;
}
static void RC_Code12_Unpack3(uint8_t pack, uint8_t index)
{
  Elrs.code12[index + 0U] = (uint8_t)(pack / 16U);
  Elrs.code12[index + 1U] = (uint8_t)((pack / 4U) % 4U);
  Elrs.code12[index + 2U] = (uint8_t)(pack % 4U);
}
static void RC_Code12_Update(const int16_t *ch)
{
  uint8_t pack0 = RC_Code12_Channel_To_Pack(ch[12], RC_CODE12_PACK3_MAX);
  uint8_t pack1 = RC_Code12_Channel_To_Pack(ch[13], RC_CODE12_PACK3_MAX);
  uint8_t pack2 = RC_Code12_Channel_To_Pack(ch[14], RC_CODE12_PACK3_MAX);
  uint8_t pack3 = RC_Code12_Channel_To_Pack(ch[15], RC_CODE12_PACK3_TRIGGER_MAX);
  uint8_t trigger = ((pack3 & RC_CODE12_TRIGGER_BIT) != 0U) ? 1U : 0U;
  uint8_t pack3_digits = (uint8_t)(pack3 & RC_CODE12_PACK3_MAX);

  rc_code12_pack[0] = pack0;
  rc_code12_pack[1] = pack1;
  rc_code12_pack[2] = pack2;
  rc_code12_pack[3] = pack3;
  rc_code12_trigger = trigger;

  RC_Code12_Unpack3(pack0, 0U);
  RC_Code12_Unpack3(pack1, 3U);
  RC_Code12_Unpack3(pack2, 6U);
  RC_Code12_Unpack3(pack3_digits, 9U);

  if ((rc_code12_last_trigger == 0xFFU) || (trigger != rc_code12_last_trigger))
  {
    rc_code12_last_trigger = trigger;
    Elrs.code12_update_flag = 1U;
    Elrs.code12_update_count++;
  }
}
void Dead_Zone(int16_t *value)
{
  if ((*value <= 5) && (*value >= -5))
  {
    *value = 0;
  }
}

uint8_t RC_Receive_Data(const uint8_t *data_buff)
{
  int16_t ch[16];
  uint8_t crc_calc;

  rc_last_frame_len = data_buff[1];
  rc_last_frame_type = data_buff[2];

  if ((data_buff[0] != CRSF_ADDRESS) || (data_buff[1] != CRSF_LEN_RC_CHANNELS) || (data_buff[2] != CRSF_TYPE_RC_CHANNELS_PACKED))
  {
    rc_header_fail_count++;
    return 0U;
  }

  crc_calc = RC_Calc_Crc8(&data_buff[2], CRSF_LEN_RC_CHANNELS - 1U);
  rc_last_crc_calc = crc_calc;
  rc_last_crc_recv = data_buff[CRSF_RC_FRAME_SIZE - 1U];

  if (crc_calc != data_buff[CRSF_RC_FRAME_SIZE - 1U])
  {
    rc_crc_fail_count++;
    return 0U;
  }
  ch[0] = ((data_buff[3] >> 0) | (data_buff[4] << 8)) & 0x07FF;//右x
  ch[1] = ((data_buff[4] >> 3) | (data_buff[5] << 5)) & 0x07FF;//右y
  ch[2] = ((data_buff[5] >> 6) | (data_buff[6] << 2) | (data_buff[7] << 10)) & 0x07FF;//左y
  ch[3] = ((data_buff[7] >> 1) | (data_buff[8] << 7)) & 0x07FF;//左x
  ch[4] = ((data_buff[8] >> 4) | (data_buff[9] << 4)) & 0x07FF;//A
  ch[5] = ((data_buff[9] >> 7) | (data_buff[10] << 1) | (data_buff[11] << 9)) & 0x07FF;//E
  ch[6] = ((data_buff[11] >> 2) | (data_buff[12] << 6)) & 0x07FF;//F
  ch[7] = ((data_buff[12] >> 5) | (data_buff[13] << 3)) & 0x07FF;//B
  ch[8] = ((data_buff[14] >> 0) | (data_buff[15] << 8)) & 0x07FF;//C
  ch[9] = ((data_buff[15] >> 3) | (data_buff[16] << 5)) & 0x07FF;//S2
  ch[10] = ((data_buff[16] >> 6) | (data_buff[17] << 2) | (data_buff[18] << 10)) & 0x07FF;//D
  ch[11] = ((data_buff[18] >> 1) | (data_buff[19] << 7)) & 0x07FF;//S1
  ch[12] = ((data_buff[19] >> 4) | (data_buff[20] << 4)) & 0x07FF;//Code12-1
  ch[13] = ((data_buff[20] >> 7) | (data_buff[21] << 1) | (data_buff[22] << 9)) & 0x07FF;//Code12-2
  ch[14] = ((data_buff[22] >> 2) | (data_buff[23] << 6)) & 0x07FF;//Code12-3
  ch[15] = ((data_buff[23] >> 5) | (data_buff[24] << 3)) & 0x07FF;//Code12-Send

  

  RC_Code12_Update(ch);

  ch[0] -= 960;
  ch[1] -= 992;
  ch[2] -= 992;
  ch[3] -= 992;
  ch[9] -= 992;
  ch[11] -= 992;

  Data_Change(&ch[5]);
  Data_Change(&ch[6]);
  Data_Change(&ch[7]);
  Data_Change(&ch[8]);
  Dead_Zone(&ch[0]);
  Dead_Zone(&ch[1]);
  Dead_Zone(&ch[2]);
  Dead_Zone(&ch[3]);
  Channel_optimization_key(&ch[4]);
  Channel_optimization_key(&ch[10]);

  for (uint8_t i = 0U; i < 16U; i++)
  {
    Elrs.ch[i] = ch[i];
  }

  rc_parse_ok_count++;
  return 1U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  rc_rx_event_count++;
  rc_last_error_code = huart->ErrorCode;

  if (Size > sizeof(rx_parse_buff))
  {
    Size = sizeof(rx_parse_buff);
  }

  memcpy(rx_parse_buff, rx_buff, Size);
  rx_len = Size;
  for (uint8_t i = 0U; i < sizeof(rc_last_rx_data); i++)
  {
    rc_last_rx_data[i] = (i < Size) ? rx_parse_buff[i] : 0U;
  }
  RC_Arm_DMA();
  RC_Process_RxBuffer(rx_parse_buff, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    rc_uart_error_count++;
    rc_last_error_code = huart->ErrorCode;
    rc_rx_started = 0U;
  }
}
