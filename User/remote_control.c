#include "remote_control.h"

/* DMA 区和软件环形区分离：ISR 只搬字节，RemoteTask 独占协议解析器。 */
#define RC_DMA_BUF_SIZE 128U
#define RC_RING_SIZE 512U
#define RC_PARSE_BLOCK_SIZE 64U
#define CRSF_ADDRESS 0xC8U
#define CRSF_LEN_RC_CHANNELS 0x18U
#define CRSF_TYPE_RC_CHANNELS_PACKED 0x16U
#define CRSF_RC_FRAME_SIZE (CRSF_LEN_RC_CHANNELS + 2U)
#define CRSF_MAX_FRAME_SIZE 64U
#define CRSF_CRC_POLY 0xD5U

#define RC_CODE12_CHANNEL_MIN 172
#define RC_CODE12_CHANNEL_MAX 1811
#define RC_CODE12_PACK3_MAX 63U
#define RC_CODE12_PACK3_TRIGGER_MAX 127U
#define RC_CODE12_TRIGGER_BIT 64U

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
volatile uint8_t rc_rx_started;
volatile uint32_t rc_ring_overflow_count;
volatile uint32_t rc_dropped_byte_count;
volatile uint32_t rc_recovery_count;
volatile uint16_t rc_ring_high_water;

/* Elrs 仅用于调试观察；消费者必须使用 RC_GetSnapshot()。 */
volatile ELRS_t Elrs;
static ELRS_t rc_state;
static QueueHandle_t rc_state_queue;
static TaskHandle_t rc_owner_task;
static uint8_t rx_buff[RC_DMA_BUF_SIZE];
static uint8_t rc_ring[RC_RING_SIZE];
static volatile uint16_t rc_ring_head;
static volatile uint16_t rc_ring_tail;
static volatile uint8_t rc_stream_fault;
static volatile uint8_t rc_recovery_requested;
static uint16_t rc_dma_last_pos;
static uint8_t rc_code12_last_trigger = 0xFFU;
static uint8_t crsf_frame[CRSF_MAX_FRAME_SIZE];
static uint16_t crsf_index;
static uint16_t crsf_expected_size;

static uint8_t RC_Receive_Data(const uint8_t *data_buff);

static uint8_t RC_Calc_Crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;
  while (len-- > 0U)
  {
    crc ^= *data++;
    for (uint8_t i = 0U; i < 8U; i++)
    {
      crc = ((crc & 0x80U) != 0U)
          ? (uint8_t)((crc << 1U) ^ CRSF_CRC_POLY)
          : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

static void RC_Reset_Crsf_Parser(void)
{
  crsf_index = 0U;
  crsf_expected_size = 0U;
}

/* 长度异常或 CRC 错误时逐字节重新找帧头，保留后面可能已经收到的合法帧。 */
static void RC_Process_RxByte(uint8_t byte)
{
  crsf_frame[crsf_index++] = byte;
  while (crsf_index != 0U)
  {
    if (crsf_frame[0] == CRSF_ADDRESS)
    {
      if (crsf_index < 2U)
      {
        return;
      }
      crsf_expected_size = (uint16_t)crsf_frame[1] + 2U;
      rc_last_expected_size = crsf_expected_size;
      if ((crsf_frame[1] >= 2U) && (crsf_expected_size <= CRSF_MAX_FRAME_SIZE))
      {
        if (crsf_index < crsf_expected_size)
        {
          return;
        }
        if (RC_Receive_Data(crsf_frame) != 0U)
        {
          crsf_index -= crsf_expected_size;
          memmove(crsf_frame, &crsf_frame[crsf_expected_size], crsf_index);
          continue;
        }
      }
      else
      {
        rc_header_fail_count++;
      }
    }
    crsf_index--;
    memmove(crsf_frame, &crsf_frame[1], crsf_index);
  }
}

BaseType_t RC_Init(TaskHandle_t owner_task)
{
  if ((owner_task == NULL) || (rc_state_queue != NULL))
  {
    return pdFAIL;
  }
  rc_state_queue = xQueueCreate(1U, sizeof(ELRS_t));
  if (rc_state_queue == NULL)
  {
    return pdFAIL;
  }
  rc_owner_task = owner_task;
  return pdPASS;
}

/* 多个业务任务都能读取，不移除队列元素；首帧到来前返回失败和全零快照。 */
BaseType_t RC_GetSnapshot(ELRS_t *snapshot)
{
  if (snapshot == NULL)
  {
    return pdFAIL;
  }
  if ((rc_state_queue != NULL) &&
      (xQueuePeek(rc_state_queue, snapshot, 0U) == pdPASS))
  {
    return pdPASS;
  }
  memset(snapshot, 0, sizeof(*snapshot));
  return pdFAIL;
}

/* 仅由 RemoteTask 调用；正常接收期间不重启，错误恢复时先终止旧接收。 */
static void RC_Arm_DMA(void)
{
  HAL_StatusTypeDef status = HAL_UART_AbortReceive(&huart2);
  if (status != HAL_OK)
  {
    rc_last_arm_status = (uint8_t)status;
    rc_arm_fail_count++;
    return;
  }

  taskENTER_CRITICAL();
  rc_ring_head = 0U;
  rc_ring_tail = 0U;
  rc_dma_last_pos = 0U;
  rc_stream_fault = 0U;
  rc_recovery_requested = 0U;
  RC_Reset_Crsf_Parser();
  __HAL_UART_CLEAR_PEFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  __HAL_UART_CLEAR_IDLEFLAG(&huart2);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, sizeof(rx_buff));
  rc_last_arm_status = (uint8_t)status;
  rc_last_error_code = huart2.ErrorCode;
  if (status == HAL_OK)
  {
    /* 保留 HT/TC 中断，连续输入无 IDLE 时也能及时搬走半个 DMA 区。 */
    rc_rx_started = 1U;
    rc_arm_ok_count++;
  }
  else
  {
    rc_rx_started = 0U;
    rc_arm_fail_count++;
  }
  taskEXIT_CRITICAL();
}

void RC_Receive(void)
{
  uint8_t block[RC_PARSE_BLOCK_SIZE];
  uint16_t processed = 0U;

  configASSERT(rc_state_queue != NULL);
  if (rc_rx_started == 0U)
  {
    RC_Arm_DMA();
  }
  while ((rc_rx_started != 0U) && (processed < RC_RING_SIZE))
  {
    uint16_t count = 0U;
    uint8_t reset_parser = 0U;
    /* 短临界区取出一批字节；解析、CRC 均在临界区外执行。 */
    taskENTER_CRITICAL();
    if (rc_stream_fault != 0U)
    {
      rc_ring_tail = rc_ring_head;
      rc_stream_fault = 0U;
      reset_parser = 1U;
    }
    while ((rc_ring_tail != rc_ring_head) && (count < sizeof(block)))
    {
      block[count++] = rc_ring[rc_ring_tail];
      rc_ring_tail = (uint16_t)((rc_ring_tail + 1U) % RC_RING_SIZE);
    }
    taskEXIT_CRITICAL();
    if (reset_parser != 0U)
    {
      RC_Reset_Crsf_Parser();
    }
    if (count == 0U)
    {
      break;
    }
    for (uint16_t i = 0U; i < count; i++)
    {
      if ((rc_stream_fault != 0U) || (rc_recovery_requested != 0U))
      {
        RC_Reset_Crsf_Parser();
        break;
      }
      RC_Process_RxByte(block[i]);
    }
    processed += count;
  }
  /* 限制一次处理量；仍有积压则保留一次通知，下一轮继续，不等待 10 ms。 */
  if ((rc_rx_started != 0U) &&
      ((rc_ring_head != rc_ring_tail) || (rc_recovery_requested != 0U)))
  {
    xTaskNotifyGive(rc_owner_task);
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
static void RC_Code12_Unpack3(ELRS_t *state, uint8_t pack, uint8_t index)
{
  state->code12[index + 0U] = (uint8_t)(pack / 16U);
  state->code12[index + 1U] = (uint8_t)((pack / 4U) % 4U);
  state->code12[index + 2U] = (uint8_t)(pack % 4U);
}
static void RC_Code12_Update(ELRS_t *state, const int16_t *ch)
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

  RC_Code12_Unpack3(state, pack0, 0U);
  RC_Code12_Unpack3(state, pack1, 3U);
  RC_Code12_Unpack3(state, pack2, 6U);
  RC_Code12_Unpack3(state, pack3_digits, 9U);

  if ((rc_code12_last_trigger == 0xFFU) || (trigger != rc_code12_last_trigger))
  {
    state->code12_update_flag = 1U;
    state->code12_update_count++;
  }
}
void Dead_Zone(int16_t *value)
{
  if ((*value <= 5) && (*value >= -5))
  {
    *value = 0;
  }
}

static uint8_t RC_Receive_Data(const uint8_t *data_buff)
{
  int16_t ch[16];
  ELRS_t next = rc_state;
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

  

  RC_Code12_Update(&next, ch);

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
    next.ch[i] = ch[i];
  }

  /* 只有完整且 CRC 正确的帧才能发布；多任务通过 Peek 读取同一份快照。 */
  taskENTER_CRITICAL();
  if ((rc_stream_fault != 0U) || (rc_recovery_requested != 0U))
  {
    taskEXIT_CRITICAL();
    return 0U;
  }
  next.frame_seq = rc_parse_ok_count + 1U;
  next.last_valid_tick = xTaskGetTickCount();
  rc_code12_last_trigger = rc_code12_trigger;
  rc_state = next;
  Elrs = next; /* 仅保留调试镜像，业务任务不再直接读取该变量。 */
  xQueueOverwrite(rc_state_queue, &next);
  rc_parse_ok_count++;
  taskEXIT_CRITICAL();
  return 1U;
}

/* USART2 与 DMA1_Channel1 必须使用相同抢占优先级，串行维护写位置。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  BaseType_t task_woken = pdFALSE;
  uint16_t pos;
  uint16_t count;
  uint16_t head;
  uint16_t used;
  (void)Size;

  if ((huart->Instance != USART2) || (rc_rx_started == 0U) ||
      (rc_owner_task == NULL))
  {
    return;
  }
  rc_rx_event_count++;
  /* 读取当前 DMA 写位置，而非使用可能滞后的 HT/TC 回调位置。 */
  pos = (uint16_t)(RC_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx));
  pos %= RC_DMA_BUF_SIZE;
  count = (uint16_t)((pos + RC_DMA_BUF_SIZE - rc_dma_last_pos) % RC_DMA_BUF_SIZE);
  rx_len = count;
  if (count == 0U)
  {
    return; /* TC 后紧跟 IDLE 等重复通知不重复投递。 */
  }
  head = rc_ring_head;
  used = (uint16_t)((head + RC_RING_SIZE - rc_ring_tail) % RC_RING_SIZE);
  if ((rc_stream_fault != 0U) || (count > RC_RING_SIZE - 1U - used))
  {
    if (rc_stream_fault == 0U)
    {
      rc_ring_overflow_count++;
    }
    rc_stream_fault = 1U;
    rc_dropped_byte_count += count;
  }
  else
  {
    for (uint16_t i = 0U; i < count; i++)
    {
      uint8_t byte = rx_buff[(rc_dma_last_pos + i) % RC_DMA_BUF_SIZE];
      rc_ring[head] = byte;
      head = (uint16_t)((head + 1U) % RC_RING_SIZE);
      if (i < sizeof(rc_last_rx_data))
      {
        rc_last_rx_data[i] = byte;
      }
    }
    __DMB(); /* 先写好字节，再发布 head；读端在短临界区内取数据。 */
    rc_ring_head = head;
    if (used + count > rc_ring_high_water)
    {
      rc_ring_high_water = (uint16_t)(used + count);
    }
  }
  rc_dma_last_pos = pos;
  vTaskNotifyGiveFromISR(rc_owner_task, &task_woken);
  portYIELD_FROM_ISR(task_woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart->Instance == USART2) && (rc_owner_task != NULL))
  {
    BaseType_t task_woken = pdFALSE;
    rc_uart_error_count++;
    rc_last_error_code = huart->ErrorCode;
    rc_rx_started = 0U;
    rc_recovery_requested = 1U;
    rc_recovery_count++;
    vTaskNotifyGiveFromISR(rc_owner_task, &task_woken);
    portYIELD_FROM_ISR(task_woken);
  }
}
