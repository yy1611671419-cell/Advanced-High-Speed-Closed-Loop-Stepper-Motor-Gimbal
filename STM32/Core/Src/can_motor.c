#include "can_motor.h"
#include <string.h>

extern CANMotor_t g_motor_pan;
extern CANMotor_t g_motor_tilt;

static void CANMotor_SendExt(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.Identifier = id;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;

    switch (len) {
    case 1: tx_header.DataLength = FDCAN_DLC_BYTES_1; break;
    case 2: tx_header.DataLength = FDCAN_DLC_BYTES_2; break;
    case 3: tx_header.DataLength = FDCAN_DLC_BYTES_3; break;
    case 4: tx_header.DataLength = FDCAN_DLC_BYTES_4; break;
    case 5: tx_header.DataLength = FDCAN_DLC_BYTES_5; break;
    case 6: tx_header.DataLength = FDCAN_DLC_BYTES_6; break;
    case 7: tx_header.DataLength = FDCAN_DLC_BYTES_7; break;
    default: tx_header.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    tx_header.ErrorStatePassive = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    uint32_t mailbox;
    HAL_FDCAN_AddMessageToTxFifo(hfdcan, &tx_header, data, &mailbox);
}

void CANMotor_Init(CANMotor_t *m, FDCAN_HandleTypeDef *hfdcan, uint8_t addr)
{
    m->hfdcan = hfdcan;
    m->addr = addr;
    m->rx_ready = 0;
    m->rx_len = 0;
    m->rx_id = 0;
    memset(m->rx_buf, 0, sizeof(m->rx_buf));
}

void CANMotor_Enable(CANMotor_t *m, uint8_t en)
{
    uint8_t data[4];
    data[0] = ZDT_CMD_ENABLE;
    data[1] = ZDT_AUX_ENABLE;
    data[2] = en ? 0x01 : 0x00;
    data[3] = ZDT_CHECKSUM;

    uint32_t id = ((uint32_t)m->addr << 8) | 0x00;
    CANMotor_SendExt(m->hfdcan, id, data, 4);
}

void CANMotor_Position(CANMotor_t *m, float angle, float speed, float accel, uint8_t sync)
{
    int32_t pos = (int32_t)(angle * POS_SCALE);
    uint16_t spd = (uint16_t)(speed * SPEED_SCALE);
    uint16_t acc = (uint16_t)accel;

    uint8_t dir = (pos >= 0) ? MOTION_DIR_CW : MOTION_DIR_CCW;
    uint32_t abs_pos = (uint32_t)(pos >= 0 ? pos : -pos);

    uint8_t frame[15];
    frame[0]  = ZDT_CMD_POSITION;
    frame[1]  = dir;
    frame[2]  = (acc >> 8) & 0xFF;
    frame[3]  = acc & 0xFF;
    frame[4]  = (acc >> 8) & 0xFF;
    frame[5]  = acc & 0xFF;
    frame[6]  = (spd >> 8) & 0xFF;
    frame[7]  = spd & 0xFF;
    frame[8]  = (abs_pos >> 24) & 0xFF;
    frame[9]  = (abs_pos >> 16) & 0xFF;
    frame[10] = (abs_pos >> 8) & 0xFF;
    frame[11] = abs_pos & 0xFF;
    frame[12] = MOTION_MODE_ABS;
    frame[13] = sync;
    frame[14] = ZDT_CHECKSUM;

    uint32_t id0 = ((uint32_t)m->addr << 8) | 0x00;
    uint32_t id1 = ((uint32_t)m->addr << 8) | 0x01;

    CANMotor_SendExt(m->hfdcan, id0, &frame[0], 8);
    for (volatile uint32_t d = 0; d < 48000; d++);  // ~1ms busy-wait for FIFO spacing
    CANMotor_SendExt(m->hfdcan, id1, &frame[8], 7);
}

void CANMotor_PositionBuffered(CANMotor_t *m, float angle, float speed, float accel)
{
    CANMotor_Position(m, angle, speed, accel, SYNC_BUFFERED);
}

void CANMotor_Stop(CANMotor_t *m)
{
    uint8_t data[3];
    data[0] = ZDT_CMD_STOP;
    data[1] = ZDT_AUX_STOP;
    data[2] = SYNC_IMMEDIATE;

    uint32_t id = ((uint32_t)m->addr << 8) | 0x00;
    CANMotor_SendExt(m->hfdcan, id, data, 3);
}

void CANMotor_SyncTrigger(void)
{
    uint8_t data[3];
    data[0] = ZDT_CMD_SYNC;
    data[1] = ZDT_AUX_SYNC;
    data[2] = ZDT_CHECKSUM;

    uint32_t id = (0x00 << 8) | 0x00;
    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.Identifier = id;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_3;
    tx_header.ErrorStatePassive = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    uint32_t mailbox;
    HAL_FDCAN_AddMessageToTxFifo(g_motor_pan.hfdcan, &tx_header, data, &mailbox);
}

static void SendSimpleCmd(CANMotor_t *m, uint8_t cmd)
{
    uint8_t data[3];
    data[0] = cmd;
    data[1] = 0x00;
    data[2] = ZDT_CHECKSUM;

    uint32_t id = ((uint32_t)m->addr << 8) | 0x00;
    CANMotor_SendExt(m->hfdcan, id, data, 3);
}

void CANMotor_ReadPosition(CANMotor_t *m)
{
    SendSimpleCmd(m, ZDT_CMD_READ_POS);
}

void CANMotor_ReadSpeed(CANMotor_t *m)
{
    SendSimpleCmd(m, ZDT_CMD_READ_SPEED);
}

void CANMotor_ReadStatus(CANMotor_t *m)
{
    SendSimpleCmd(m, ZDT_CMD_READ_STATUS);
}

void CANMotor_ReadPosError(CANMotor_t *m)
{
    SendSimpleCmd(m, ZDT_CMD_READ_ERR);
}

void CANMotor_SetZero(CANMotor_t *m, uint8_t save)
{
    uint8_t data[4];
    data[0] = ZDT_CMD_SET_ZERO;
    data[1] = ZDT_AUX_SET_ZERO;
    data[2] = save ? 0x01 : 0x00;
    data[3] = ZDT_CHECKSUM;

    uint32_t id = ((uint32_t)m->addr << 8) | 0x00;
    CANMotor_SendExt(m->hfdcan, id, data, 4);
}

void CANMotor_StartHome(CANMotor_t *m, uint8_t mode)
{
    uint8_t data[4];
    data[0] = ZDT_CMD_HOME;
    data[1] = mode & 0x07;
    data[2] = SYNC_IMMEDIATE;
    data[3] = ZDT_CHECKSUM;

    uint32_t id = ((uint32_t)m->addr << 8) | 0x00;
    CANMotor_SendExt(m->hfdcan, id, data, 4);
}

void CANMotor_AbortHome(CANMotor_t *m)
{
    uint8_t data[3];
    data[0] = ZDT_CMD_ABORT_HOME;
    data[1] = ZDT_AUX_ABORT_HOME;
    data[2] = ZDT_CHECKSUM;

    uint32_t id = ((uint32_t)m->addr << 8) | 0x00;
    CANMotor_SendExt(m->hfdcan, id, data, 3);
}

void CANMotor_ReadHomeStatus(CANMotor_t *m)
{
    SendSimpleCmd(m, ZDT_CMD_READ_HOME_ST);
}

void CANMotor_ProcessResponse(CANMotor_t *m, MotorState_t *state)
{
    if (!m->rx_ready) return;
    m->rx_ready = 0;

    uint8_t func_code = m->rx_buf[0];

    switch (func_code) {
    case ZDT_CMD_READ_POS: {
        if (m->rx_len >= 7) {
            uint8_t sign = m->rx_buf[2];
            uint32_t pos = ((uint32_t)m->rx_buf[3] << 24) |
                           ((uint32_t)m->rx_buf[4] << 16) |
                           ((uint32_t)m->rx_buf[5] << 8)  |
                           m->rx_buf[6];
            float angle = (float)pos / POS_SCALE;
            if (sign) angle = -angle;
            state->current_angle = angle;
        }
        break;
    }
    case ZDT_CMD_READ_SPEED: {
        if (m->rx_len >= 5) {
            uint8_t sign = m->rx_buf[2];
            uint16_t spd = ((uint16_t)m->rx_buf[3] << 8) | m->rx_buf[4];
            float rpm = (float)spd / SPEED_SCALE;
            if (sign) rpm = -rpm;
            state->current_speed_rpm = rpm;
        }
        break;
    }
    case ZDT_CMD_READ_STATUS: {
        if (m->rx_len >= 3) {
            state->status_flags = m->rx_buf[2];
            state->enabled = (m->rx_buf[2] & 0x01) ? 1 : 0;
        }
        break;
    }
    case ZDT_CMD_READ_ERR: {
        if (m->rx_len >= 7) {
            uint8_t sign = m->rx_buf[2];
            uint32_t err = ((uint32_t)m->rx_buf[3] << 24) |
                           ((uint32_t)m->rx_buf[4] << 16) |
                           ((uint32_t)m->rx_buf[5] << 8)  |
                           m->rx_buf[6];
            float err_deg = (float)err / 100.0f;
            state->position_error = err_deg;
        }
        break;
    }
    case ZDT_CMD_READ_HOME_ST: {
        if (m->rx_len >= 3) {
            state->home_state = m->rx_buf[2];
        }
        break;
    }
    default:
        break;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) == HAL_OK) {
            uint32_t id = rx_header.Identifier;
            uint8_t addr = (id >> 8) & 0xFF;

            CANMotor_t *m = NULL;
            if (addr == g_motor_pan.addr) {
                m = &g_motor_pan;
            } else if (addr == g_motor_tilt.addr) {
                m = &g_motor_tilt;
            }

            if (m != NULL) {
                uint8_t pkt_num = id & 0xFF;
                uint8_t copy_len = (rx_header.DataLength >> 16) & 0x0F;
                if (copy_len > 8) copy_len = 8;
                if (pkt_num == 0) {
                    memcpy(m->rx_buf, data, copy_len);
                    m->rx_len = copy_len;
                } else {
                    if (m->rx_len + copy_len <= 16) {
                        memcpy(&m->rx_buf[m->rx_len], data, copy_len);
                        m->rx_len += copy_len;
                    }
                }
                if (copy_len < 8) {
                    m->rx_ready = 1;
                }
            }
        }
    }
}
