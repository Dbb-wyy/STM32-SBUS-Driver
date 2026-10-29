#include "wfly_et16s.h"
int16_t map_D(int16_t x) { return (x - 353) * 6600 / 1342; }
int16_t map_Roker(int16_t x) { return (x - 1024); }
RC_ctrl_t rc_ctrl;
data_rc_t data_rc;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
void RC_Init(void) {
    uart3_dma_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
void uart3_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num) {
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) {
    }
    hdma_usart3_rx.Instance->PAR = (uint32_t)&(USART3->DR);
    hdma_usart3_rx.Instance->M0AR = (uint32_t)rx1_buf;
    hdma_usart3_rx.Instance->M1AR = (uint32_t)rx2_buf;
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
    if (sbus_buf == NULL || rc_ctrl == NULL) {
        return;
    }
    uint8_t *buf = (uint8_t *)sbus_buf;
    rc_ctrl->rc.ch[0] = map_Roker(((buf[1] | buf[2] << 8) & 0x07FF));
    rc_ctrl->rc.ch[1] = map_Roker(((buf[2] >> 3 | buf[3] << 5) & 0x07FF));
    rc_ctrl->rc.ch[2] =
        map_Roker(((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF));
    rc_ctrl->rc.ch[3] = map_Roker(((buf[5] >> 1 | buf[6] << 7) & 0x07FF));
    uint32_t bit_index = 44;
    for (int i = 4; i < 16; i++) {
        rc_ctrl->rc.ch[i] = ((buf[1 + bit_index / 8] >> (bit_index % 8)) |
                             (buf[2 + bit_index / 8] << (8 - bit_index % 8))) &
                            0x07FF;
        bit_index += 11;
    }
    rc_ctrl->rc.ch17 = (buf[23] >> 0) & 0x01;
    rc_ctrl->rc.ch18 = (buf[23] >> 1) & 0x01;
    rc_ctrl->rc.frame_lost = (buf[23] >> 2) & 0x01;
    rc_ctrl->rc.failsafe = (buf[23] >> 3) & 0x01;
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
    case 159:
        rc_ctrl->rc.SB = 0;
        break;
    case 0:
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
    case 671:
        rc_ctrl->rc.SG = 0;
        break;
    case 0:
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
    rc_ctrl->rc.LD = map_D(rc_ctrl->rc.ch[14]);
    rc_ctrl->rc.RD = map_D(rc_ctrl->rc.ch[15]);
}
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
    if (data_rc->status_gimbal == 1) {
        switch (data_rc->RC_ctrl->rc.SB) {
        case 1:
            data_rc->status_ammo = 0;
            break;
        case 2:
            data_rc->status_ammo = 1;
            break;
        case 0:
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
    if (USART3->SR & UART_FLAG_IDLE) {
        static uint16_t rx_len;
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
            rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            if (rx_len == SBUS_FRAME_LENGTH) {
                if (sbus_rx_buf[0][0] == 0x0F) {
                    sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                }
            }
        } else {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
            rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            CLEAR_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            if (rx_len == SBUS_FRAME_LENGTH) {
                if (sbus_rx_buf[1][0] == 0x0F) {
                    sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                }
            }
        }
    }
}
const RC_ctrl_t *get_remote_control_point(void) { return &rc_ctrl; }