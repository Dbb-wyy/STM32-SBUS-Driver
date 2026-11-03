#include "wfly_et16s.h"

/* ===========================================================================
 * WFLY ET16S —— W.BUS 协议接收
 * ---------------------------------------------------------------------------
 *  一帧 25 字节（100000 baud，偶校验）：
 *    [0]       0x0F            帧头
 *    [1..22]   通道数据         16 通道 × 11 bit = 176 bit = 22 Byte
 *    [23]      标志位           bit0 ch17 / bit1 ch18 / bit2 frame_lost
 *                              / bit3 failsafe
 *    [24]      0x00            帧尾
 *
 *  通道位流规则（关键）：
 *    - 每通道 11 bit；
 *    - 小端位序（LSB first），即一个通道的低位在靠前的 bit；
 *    - 16 个通道连续跨字节紧排，不是“每通道独占 2 字节”；
 *    - 因此第 n 通道的起始 bit = n * 11，需从位流里按 3 字节窗口抽取，
 *      否则当起始 bit%8 = 6 或 7 时会丢高位（旧实现只取 2 字节的 bug）。
 * ===========================================================================
 */

int16_t map_D(int16_t x) { return (x - 353) * 6600 / 1342; }
int16_t map_Roker(int16_t x) { return (x - 1024); }

RC_ctrl_t rc_ctrl = {0};
data_rc_t data_rc = {0};

extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

/* 从“通道数据区”的连续小端位流中取出第 ch 个 11 bit 通道。
 * 位流从 buf[1] 的 bit0 开始（buf[0] 是 0x0F 帧头）。
 * 一次读 3 字节（24 bit）再右移、掩码，可覆盖任意 11 bit 跨字节窗口。 */
static uint16_t wbus_get_channel(const uint8_t *buf, uint8_t ch) {
    uint16_t bit = (uint16_t)ch * 11u;        /* 通道在数据区内的 bit 偏移 */
    uint8_t idx = (uint8_t)(1u + (bit >> 3)); /* +1：跳过 0x0F 帧头 */
    uint8_t sh = (uint8_t)(bit & 0x07u);      /* 在起始字节内的位移 */
    uint32_t v = (uint32_t)buf[idx] | ((uint32_t)buf[idx + 1] << 8) |
                 ((uint32_t)buf[idx + 2] << 16);
    return (uint16_t)((v >> sh) & 0x07FFu);
}

void RC_Init(void) {
    uart3_dma_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

void uart3_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num) {
    /* 使能 USART3 的 DMA 接收请求 */
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    /* 使能空闲中断 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    /* 先关 DMA，并等待 EN 真正清零（写 0 后可能要到当前传输结束才生效） */
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t)&(USART3->DR);
    hdma_usart3_rx.Instance->M0AR = (uint32_t)rx1_buf;
    hdma_usart3_rx.Instance->M1AR = (uint32_t)rx2_buf;
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    /* 双缓冲 */
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

/* 把一帧 25 字节的 W.BUS 数据解析进 rc_ctrl。
 * 本阶段只做通道解码；rc_ctrl->rc.ch[] 的语义映射保持与原实现一致。 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
    if (sbus_buf == NULL || rc_ctrl == NULL) {
        return;
    }

    const uint8_t *buf = (const uint8_t *)sbus_buf;

    /* —— 16 个 11 bit 通道 —— */
    uint16_t ch[16];
    for (uint8_t i = 0; i < 16; i++) {
        ch[i] = wbus_get_channel(buf, i);
    }

    /* 前 4 通道为摇杆：以 1024 为中位做居中 */
    rc_ctrl->rc.ch[0] = map_Roker((int16_t)ch[0]);
    rc_ctrl->rc.ch[1] = map_Roker((int16_t)ch[1]);
    rc_ctrl->rc.ch[2] = map_Roker((int16_t)ch[2]);
    rc_ctrl->rc.ch[3] = map_Roker((int16_t)ch[3]);

    /* 其余通道保留原始 11 bit 值（下方开关/拨轮判断沿用原值） */
    for (uint8_t i = 4; i < 16; i++) {
        rc_ctrl->rc.ch[i] = (int16_t)ch[i];
    }

    /* —— 标志位 —— */
    rc_ctrl->rc.ch17 = (buf[23] >> 0) & 0x01;
    rc_ctrl->rc.ch18 = (buf[23] >> 1) & 0x01;
    rc_ctrl->rc.frame_lost = (buf[23] >> 2) & 0x01;
    rc_ctrl->rc.failsafe = (buf[23] >> 3) & 0x01;

    /* —— 开关映射（沿用原实现的通道分配与阈值） —— */
    switch (rc_ctrl->rc.ch[4]) {
    case 1695:
        rc_ctrl->rc.SA = 0;
        break;
    case 1024:
        rc_ctrl->rc.SA = 1;
        break;
    case 353:
        rc_ctrl->rc.SA = 2;
        break;
    default:
        break;
    }
    switch (rc_ctrl->rc.ch[5]) {
    case 1695:
        rc_ctrl->rc.SB = 0;
        break;
    case 1024:
        rc_ctrl->rc.SB = 1;
        break;
    case 353:
        rc_ctrl->rc.SB = 2;
        break;
    default:
        break;
    }
    switch (rc_ctrl->rc.ch[6]) {
    case 1695:
        rc_ctrl->rc.SC = 0;
        break;
    case 1024:
        rc_ctrl->rc.SC = 1;
        break;
    case 353:
        rc_ctrl->rc.SC = 2;
        break;
    default:
        break;
    }
    switch (rc_ctrl->rc.ch[7]) {
    case 1695:
        rc_ctrl->rc.SD = 0;
        break;
    case 1024:
        rc_ctrl->rc.SD = 1;
        break;
    case 353:
        rc_ctrl->rc.SD = 2;
        break;
    default:
        rc_ctrl->rc.SD = 0;
        break;
    }
    switch (rc_ctrl->rc.ch[8]) {
    case 1695:
        rc_ctrl->rc.SE = 0;
        break;
    case 1024:
        rc_ctrl->rc.SE = 1;
        break;
    case 353:
        rc_ctrl->rc.SE = 2;
        break;
    default:
        break;
    }
    switch (rc_ctrl->rc.ch[10]) {
    case 1695:
        rc_ctrl->rc.SG = 0;
        break;
    case 1024:
        rc_ctrl->rc.SG = 1;
        break;
    case 353:
        rc_ctrl->rc.SG = 2;
        break;
    default:
        break;
    }
    switch (rc_ctrl->rc.ch[9]) {
    case 353:
        rc_ctrl->rc.SF = 0;
        break;
    case 1695:
        rc_ctrl->rc.SF = 1;
        break;
    default:
        break;
    }
    switch (rc_ctrl->rc.ch[11]) {
    case 353:
        rc_ctrl->rc.SH = 0;
        break;
    case 1695:
        rc_ctrl->rc.SH = 1;
        break;
    default:
        break;
    }

    /* —— 两个拨轮 —— */
    rc_ctrl->rc.LD = map_D(rc_ctrl->rc.ch[14]);
    rc_ctrl->rc.RD = map_D(rc_ctrl->rc.ch[15]);
}

/* 保持不动（第一阶段） */
void rc_read_data(data_rc_t *data_rc) {
    data_rc->speed_y = data_rc->RC_ctrl->rc.ch[1] * 6600 / 671;
    data_rc->speed_x = data_rc->RC_ctrl->rc.ch[0] * 6600 / 671;
    data_rc->speed_yaw = data_rc->RC_ctrl->rc.ch[3] * 3300 / 671;
    data_rc->angle_yaw += (data_rc->RC_ctrl->rc.ch[3] * 1.0 / 335.5);
    data_rc->angle_pitch += (data_rc->RC_ctrl->rc.ch[2] * 1.0 / 335.5);
#ifdef PIT_MOTOR_MAX
    if (data_rc->angle_pitch >= PIT_MOTOR_MAX) {
        data_rc->angle_pitch = PIT_MOTOR_MAX;
    }
#endif
#ifdef PIT_MOTOR_MIN
    if (data_rc->angle_pitch <= PIT_MOTOR_MIN) {
        data_rc->angle_pitch = PIT_MOTOR_MIN;
    }
#endif
#ifdef YAW_MOTOR_MAX
    if (data_rc->angle_yaw >= YAW_MOTOR_MAX) {
        data_rc->angle_yaw = YAW_MOTOR_MAX;
    }
#endif
#ifdef YAW_MOTOR_MIN
    if (data_rc->angle_yaw <= YAW_MOTOR_MIN) {
        data_rc->angle_yaw = YAW_MOTOR_MIN;
    }
#endif
    if (data_rc->angle_yaw >= 180) {
        data_rc->angle_yaw -= 360;
    } else if (data_rc->angle_yaw < -180) {
        data_rc->angle_yaw += 360;
    }
    switch (data_rc->RC_ctrl->rc.SD) {
    case 0:
        data_rc->status_chassis = 0;
        data_rc->status_gimbal = 0;
        break;
    case 1:
        data_rc->status_chassis = 1;
        data_rc->status_gimbal = 1;
        break;
    case 2:
        data_rc->status_chassis = 2;
        data_rc->status_gimbal = 2;
        break;
    default:
        break;
    }
    if (data_rc->status_gimbal == 1 || data_rc->status_gimbal == 2) {
        switch (data_rc->RC_ctrl->rc.SB) {
        case 0:
            data_rc->status_ammo = 0;
            break;
        case 1:
            data_rc->status_ammo = 1;
            break;
        case 2:
            data_rc->status_ammo = 2;
            break;
        default:
            break;
        }
    } else if (data_rc->status_gimbal != 1 && data_rc->status_gimbal != 2) {
        data_rc->status_ammo = 0;
    }
    if (data_rc->RC_ctrl->rc.ch[2] >= 550 ||
        data_rc->RC_ctrl->rc.ch[2] <= -550) {
        data_rc->reset = 1;
    } else if (data_rc->RC_ctrl->rc.ch[2] < 550 &&
               data_rc->RC_ctrl->rc.ch[2] > -550) {
        data_rc->reset = 0;
    }
}

void USART3_IRQHandler(void) {
    if (huart3.Instance->SR & UART_FLAG_RXNE) {
        /* 收到数据：清 PE/FE/ORE/IDLE（读 SR 再读 DR），防止溢出标志挂死 */
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    } else if (USART3->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len;
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            if (this_time_rx_len == SBUS_FRAME_LENGTH) {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        } else {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            CLEAR_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            if (this_time_rx_len == SBUS_FRAME_LENGTH) {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}

const RC_ctrl_t *get_remote_control_point(void) { return &rc_ctrl; }
