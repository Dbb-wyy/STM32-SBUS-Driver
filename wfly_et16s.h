#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"
#include "usart.h"

#define SBUS_RX_BUF_NUM 50u
#define SBUS_FRAME_LENGTH 25u

#define SBUS_CH_VALUE_OFFSET 1024

typedef struct __attribute__((packed)) {
    struct {
        int16_t ch[16];

        char SA;
        char SB;
        char SC;
        char SD;
        char SE;
        char SF;
        char SG;
        char SH;

        int16_t LD;
        int16_t RD;

        uint8_t ch17;
        uint8_t ch18;

        uint8_t frame_lost;
        uint8_t failsafe;

    } rc;

} RC_ctrl_t;

typedef struct {
    const RC_ctrl_t *RC_ctrl;
    int16_t speed_x;
    int16_t speed_y;
    int16_t speed_yaw;
    float angle_pitch;
    float angle_yaw;
    uint8_t status_chassis;
    uint8_t status_gimbal;
    uint8_t status_ammo;
    uint8_t reset;
} data_rc_t;

void RC_Init(void);

void uart3_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

const RC_ctrl_t *get_remote_control_point(void);

void rc_read_data(data_rc_t *data_rc);
#endif