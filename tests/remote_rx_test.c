/* Execute the production receiver against a circular DMA and RTOS test double. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../User/remote_control.h"

struct TestQueue
{
    size_t item_size;
    int valid;
    unsigned char item[sizeof(ELRS_t)];
};

static struct TestQueue test_queue;
static DMA_HandleTypeDef test_dma;
UART_HandleTypeDef huart2 = { USART2, 0U, &test_dma };
static int in_isr;
static unsigned critical_depth;
static unsigned start_count;
static unsigned abort_count;
static unsigned notification_count;
static HAL_StatusTypeDef next_start_status;
static uint8_t *dma_target;
static uint16_t dma_size;
static uint16_t dma_pos;
static TickType_t test_tick;

void Test_EnterCritical(void) { assert(!in_isr); critical_depth++; }
void Test_ExitCritical(void) { assert(critical_depth); critical_depth--; }
TickType_t xTaskGetTickCount(void) { assert(!in_isr); return test_tick; }
BaseType_t xTaskNotifyGive(TaskHandle_t task)
{
    assert(!in_isr && task); notification_count++; return pdPASS;
}
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken)
{
    assert(in_isr && task); notification_count++; *woken = pdTRUE;
}
QueueHandle_t xQueueCreate(UBaseType_t count, UBaseType_t size)
{
    assert(!in_isr && count == 1U && size == sizeof(ELRS_t));
    test_queue.item_size = size;
    return &test_queue;
}
BaseType_t xQueuePeek(QueueHandle_t queue, void *value, TickType_t wait)
{
    assert(!in_isr && wait == 0U);
    if (!queue->valid) return pdFAIL;
    memcpy(value, queue->item, queue->item_size);
    return pdPASS;
}
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *value)
{
    assert(!in_isr);
    memcpy(queue->item, value, queue->item_size);
    queue->valid = 1;
    return pdPASS;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    assert(!in_isr && uart == &huart2);
    abort_count++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                            uint8_t *data, uint16_t count)
{
    assert(!in_isr && uart == &huart2);
    start_count++;
    dma_target = data;
    dma_size = count;
    dma_pos = 0U;
    test_dma.remaining = count;
    return next_start_status;
}
void HAL_NVIC_ClearPendingIRQ(int irq) { assert(irq == 1 || irq == 2); }

#include "../User/remote_control.c"

static uint8_t Test_Crc(const uint8_t *bytes, unsigned length)
{
    uint8_t crc = 0;
    for (unsigned i = 0; i < length; i++)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; bit++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void MakeFrame(uint8_t *frame, uint16_t *channels)
{
    memset(frame, 0, 26);
    frame[0] = 0xC8; frame[1] = 24; frame[2] = 0x16;
    for (unsigned c = 0; c < 16; c++)
        for (unsigned bit = 0; bit < 11; bit++)
            if (channels[c] & (1U << bit))
                frame[3 + (c * 11 + bit) / 8] |= (uint8_t)(1U << ((c * 11 + bit) % 8));
    frame[25] = Test_Crc(frame + 2, 23);
}

static void DmaEvent(uint16_t hint)
{
    uint32_t parsed = rc_parse_ok_count;
    in_isr = 1;
    HAL_UARTEx_RxEventCallback(&huart2, hint);
    in_isr = 0;
    assert(rc_parse_ok_count == parsed); /* ISR never parses or publishes. */
}

static void Feed(const uint8_t *bytes, size_t count, int idle)
{
    for (size_t i = 0; i < count; i++)
    {
        dma_target[dma_pos] = bytes[i];
        dma_pos = (uint16_t)((dma_pos + 1U) % dma_size);
        test_dma.remaining = (uint16_t)(dma_size - dma_pos);
        if (dma_pos == 0U || dma_pos == dma_size / 2U)
            DmaEvent(dma_pos == 0U ? dma_size : dma_pos);
    }
    if (idle) DmaEvent(dma_pos);
}

static void BeginCase(void)
{
    memset(&rc_state, 0, sizeof(rc_state));
    test_queue.valid = 0;
    rc_rx_started = 0;
    rc_recovery_requested = 0;
    rc_parse_ok_count = rc_crc_fail_count = rc_header_fail_count = 0;
    rc_ring_overflow_count = rc_dropped_byte_count = 0;
    rc_recovery_count = 0;
    rc_ring_high_water = 0;
    rc_code12_last_trigger = 0xFF;
    start_count = abort_count = notification_count = 0;
    next_start_status = HAL_OK;
    test_tick = 1234;
    RC_Receive();
    assert(start_count == 1U && rc_rx_started);
}

static void TestSplitAndSnapshot(uint8_t *frame)
{
    ELRS_t first, second;
    BeginCase();
    assert(RC_GetSnapshot(&first) == pdFAIL && first.frame_seq == 0);
    Feed(frame, 10, 1);
    RC_Receive();
    assert(rc_parse_ok_count == 0);
    Feed(frame + 10, 16, 1);
    RC_Receive();
    assert(RC_GetSnapshot(&first) == pdPASS);
    assert(first.ch[0] == 0 && first.ch[1] == 0 && first.ch[2] == 0);
    assert(first.ch[5] == 1 && first.ch[6] == 2 && first.ch[7] == 3);
    assert(first.ch[4] == 0 && first.ch[10] == 1);
    assert(first.frame_seq == 1 && first.last_valid_tick == test_tick);
    assert(first.code12[0] == 0 && first.code12[3] == 3 && first.code12[11] == 3);
    assert(RC_GetSnapshot(&second) == pdPASS);
    assert(memcmp(&first, &second, sizeof(first)) == 0);
    DmaEvent(26); DmaEvent(64); /* Duplicate/stale event hints. */
    RC_Receive();
    assert(rc_parse_ok_count == 1 && start_count == 1);
    puts("PASS split frame, channel/Code12 decoding, repeated queue Peek, duplicate events");
}

static void TestWrapAndMultipleFrames(uint8_t *frame)
{
    ELRS_t snapshot;
    BeginCase();
    for (unsigned i = 0; i < 100; i++)
    {
        Feed(frame, 26, 1);
        RC_Receive();
    }
    assert(rc_parse_ok_count == 100 && start_count == 1);
    assert(RC_GetSnapshot(&snapshot) == pdPASS && snapshot.frame_seq == 100);
    assert(snapshot.code12_update_count == 1);
    Feed(frame, 26, 0); Feed(frame, 26, 0); Feed(frame, 26, 1);
    RC_Receive();
    assert(rc_parse_ok_count == 103);
    BeginCase();
    /* 连续 416 字节，没有任何 IDLE，先靠 HT/TC 取走前 384 字节。 */
    for (unsigned i = 0; i < 16; i++) Feed(frame, 26, 0);
    RC_Receive();
    assert(rc_parse_ok_count == 14);
    DmaEvent(dma_pos);
    RC_Receive();
    assert(rc_parse_ok_count == 16 && start_count == 1);
    puts("PASS DMA/software ring wrap, HT/TC without IDLE, multiple frames, latest overwrite");
}

static void TestCorruptionAndResync(uint8_t *frame)
{
    uint8_t bad[26];
    const uint8_t noise[] = {0x55, 0xC8, 0xFF, 0x33, 0xC8, 0x1A};
    BeginCase();
    Feed(noise, sizeof(noise), 0);
    Feed(frame, 26, 1); /* Valid frame nested after a bogus longer header. */
    RC_Receive();
    assert(rc_parse_ok_count == 1);
    memcpy(bad, frame, sizeof(bad));
    bad[25] ^= 1;
    Feed(bad, sizeof(bad), 1);
    RC_Receive();
    assert(rc_parse_ok_count == 1 && rc_crc_fail_count > 0);
    Feed(frame, 26, 1); RC_Receive();
    assert(rc_parse_ok_count == 2);
    puts("PASS noise, invalid length, nested frame resync, CRC rejection and recovery");
}

static void TestOverflowAndUartRecovery(uint8_t *frame)
{
    BeginCase();
    Feed(frame, 8, 1); RC_Receive(); /* Leave a partial protocol frame. */
    for (unsigned i = 0; i < 25; i++) Feed(frame, 26, 1);
    assert(rc_ring_overflow_count == 1 && rc_dropped_byte_count > 0);
    RC_Receive();
    assert(rc_parse_ok_count == 0 && crsf_index == 0);
    Feed(frame, 26, 1); RC_Receive();
    assert(rc_parse_ok_count == 1);
    Feed(frame, 9, 1); RC_Receive();
    in_isr = 1; huart2.ErrorCode = 8;
    HAL_UART_ErrorCallback(&huart2);
    in_isr = 0;
    assert(rc_recovery_count == 1 && start_count == 1);
    RC_Receive();
    assert(start_count == 2 && crsf_index == 0 && rc_rx_started);
    Feed(frame, 26, 1); RC_Receive();
    assert(rc_parse_ok_count == 2);
    puts("PASS ring overflow discards discontinuous data, UART error task-only restart");
}

static void TestStartFailure(void)
{
    BeginCase();
    rc_rx_started = 0;
    next_start_status = HAL_BUSY;
    RC_Receive();
    assert(!rc_rx_started);
    unsigned count = notification_count;
    RC_Receive();
    assert(notification_count == count); /* Retry uses timed block, no busy self-notify. */
    next_start_status = HAL_OK;
    RC_Receive();
    assert(rc_rx_started);
    puts("PASS receive start failure and bounded retry");
}

int main(void)
{
    uint8_t frame[26];
    uint16_t channels[16] = {960, 992, 992, 992, 172, 172, 992, 1811,
                            172, 992, 1811, 992, 172, 1811, 992, 1811};
    MakeFrame(frame, channels);
    assert(RC_Init((TaskHandle_t)1) == pdPASS);
    TestSplitAndSnapshot(frame);
    TestWrapAndMultipleFrames(frame);
    TestCorruptionAndResync(frame);
    TestOverflowAndUartRecovery(frame);
    TestStartFailure();
    assert(!critical_depth);
    puts("remote RX tests passed");
    return 0;
}
