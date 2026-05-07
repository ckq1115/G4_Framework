//
// Created by CaoKangqi on 2026/5/5.
//
#include "CAN_Comm.h"
#include "All_Init.h"

static CAN_TP_Tx_State_t tx_state = {0};
static CAN_TP_Rx_t  rx_session = {0};

/**
 * @brief 智能非阻塞发送：调用一次最多发送一帧
 * @note 自动处理新包挂载与旧包续发
 */
uint8_t CAN_TP_Send_Struct(FDCAN_HandleTypeDef *hfdcan, void *data_ptr, uint16_t len)
{
    // 如果当前没有任务，且传入了新数据，开启新会话
    if (!tx_state.is_sending && data_ptr != NULL)
    {
        tx_state.total_len = (len > CAN_TP_MAX_PAYLOAD) ? CAN_TP_MAX_PAYLOAD : len;
        memcpy(tx_state.buf, data_ptr, tx_state.total_len); // 拷贝数据防止发送期间被外部修改
        tx_state.offset = 0;
        tx_state.seq = 0;
        tx_state.last_tick = 0;
        tx_state.is_sending = 1;
    }

    // 如果没有正在发送的任务，直接返回
    if (!tx_state.is_sending) return 0;

    // 检查帧间距
    uint32_t now = DWT->CYCCNT;
    uint32_t interval_ticks = 100 * 170;
    if (tx_state.seq > 0 && (now - tx_state.last_tick) < interval_ticks)
    {
        return 2; // 时间未到，跳过
    }

    // 准备当前帧
    uint8_t tx_buf[8];
    uint16_t remain = tx_state.total_len - tx_state.offset;
    uint8_t chunk_size = (remain > 7) ? 7 : (uint8_t)remain;
    uint8_t is_last = (remain <= 7) ? 1 : 0;

    tx_buf[0] = (is_last << 7) | (tx_state.seq & 0x7F);
    memcpy(&tx_buf[1], &tx_state.buf[tx_state.offset], chunk_size);
    if (chunk_size < 7) memset(&tx_buf[1 + chunk_size], 0, 7 - chunk_size);

    // 尝试发送
    if (FDCAN_Send_Msg(hfdcan, CAN_TP_SINGLE_ID, tx_buf, 8) == 0)
    {
        tx_state.offset += chunk_size;
        tx_state.seq++;
        tx_state.last_tick = now;

        if (is_last) tx_state.is_sending = 0; // 发送完成
        return 0; // 发送成功
    }

    return 1; // 发送失败
}

void CAN_TP_Rx_Parser(uint8_t *rx_data, uint8_t dlc_len)
{
    if (dlc_len < 2) return;

    uint8_t ctrl = rx_data[0];
    uint8_t is_last = (ctrl >> 7) & 0x01;
    uint8_t seq = ctrl & 0x7F;
    uint8_t payload_len = dlc_len - 1;

    // 新传输开始（seq==0）-> 重置会话
    if (seq == 0)
    {
        rx_session.is_active = 1;
        rx_session.current_len = 0;
        rx_session.next_seq = 0;
    }

    // 只有会话活跃且序列号匹配时才处理
    if (rx_session.is_active && seq == rx_session.next_seq)
    {
        if (rx_session.current_len + payload_len <= CAN_TP_MAX_PAYLOAD)
        {
            memcpy(&rx_session.buf[rx_session.current_len], &rx_data[1], payload_len);
            rx_session.current_len += payload_len;
            rx_session.next_seq++;

            if (is_last)
            {
                CAN_TP_On_Struct_Received(rx_session.buf, rx_session.current_len);
                rx_session.is_active = 0;   // 传输完成，释放会话
            }
        }
        else
        {
            // 缓冲区溢出，丢弃此包并复位会话
            rx_session.is_active = 0;
        }
    }
    // 如果序列号不匹配，丢弃此帧
}

__weak void CAN_TP_On_Struct_Received(uint8_t *data_ptr, uint16_t len)
{
}