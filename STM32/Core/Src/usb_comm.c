#include "usb_comm.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

extern SemaphoreHandle_t semUSB;

static const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x80A3, 0x00A6, 0x00AC, 0x80A9, 0x00B8, 0x80BD, 0x80B7, 0x00B2,
    0x0090, 0x8095, 0x809F, 0x009A, 0x808B, 0x008E, 0x0084, 0x8081,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x81E3, 0x01E6, 0x01EC, 0x81E9, 0x01F8, 0x81FD, 0x81F7, 0x01F2,
    0x01D0, 0x81D5, 0x81DF, 0x01DA, 0x81CB, 0x01CE, 0x01C4, 0x81C1,
    0x8143, 0x0146, 0x014C, 0x8149, 0x0158, 0x815D, 0x8157, 0x0152,
    0x0170, 0x8175, 0x817F, 0x017A, 0x816B, 0x016E, 0x0164, 0x8161,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x8363, 0x0366, 0x036C, 0x8369, 0x0378, 0x837D, 0x8377, 0x0372,
    0x0350, 0x8355, 0x835F, 0x035A, 0x834B, 0x034E, 0x0344, 0x8341,
    0x83C3, 0x03C6, 0x03CC, 0x83C9, 0x03D8, 0x83DD, 0x83D7, 0x03D2,
    0x03F0, 0x83F5, 0x83FF, 0x03FA, 0x83EB, 0x03EE, 0x03E4, 0x83E1,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x8283, 0x0286, 0x028C, 0x8289, 0x0298, 0x829D, 0x8297, 0x0292,
    0x02B0, 0x82B5, 0x82BF, 0x02BA, 0x82AB, 0x02AE, 0x02A4, 0x82A1,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x8223, 0x0226, 0x022C, 0x8229, 0x0238, 0x823D, 0x8237, 0x0232,
    0x0210, 0x8215, 0x821F, 0x021A, 0x820B, 0x020E, 0x0204, 0x8201,
};

uint16_t CRC16_Calc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

void USBComm_Init(USBComm_t *c)
{
    c->state = PARSE_IDLE;
    c->idx = 0;
    c->frame_ready = 0;
    c->payload_len = 0;
    memset(c->rx_buf, 0, USB_RX_BUF_SIZE);
    memset(c->tx_buf, 0, USB_TX_BUF_SIZE);
}

void USBComm_ParseByte(USBComm_t *c, uint8_t b)
{
    switch (c->state) {
    case PARSE_IDLE:
        c->idx = 0;
        if (b == FRAME_HEADER_1) c->state = PARSE_H1;
        break;
    case PARSE_H1:
        if (b == FRAME_HEADER_2) c->state = PARSE_H2;
        else c->state = PARSE_IDLE;
        break;
    case PARSE_H2:
        c->cmd_len = b;
        c->idx = 0;
        c->state = PARSE_LEN;
        break;
    case PARSE_LEN:
        c->cur_cmd = b;
        c->state = PARSE_CMD;
        break;
    case PARSE_CMD:
        c->cur_seq = b;
        c->payload_len = 0;
        c->state = PARSE_SEQ;
        break;
    case PARSE_SEQ:
        if (c->payload_len < 56) {
            c->payload[c->payload_len++] = b;
        }
        c->idx++;
        if (c->idx >= c->cmd_len) {
            c->state = PARSE_CRC_H;
        }
        break;
    case PARSE_CRC_H:
        c->state = PARSE_CRC_L;
        break;
    case PARSE_CRC_L:
        c->frame_ready = 1;
        c->state = PARSE_IDLE;
        break;
    }
}

void USBComm_SendFrame(USBComm_t *c, uint8_t cmd, uint8_t seq, const uint8_t *data, uint8_t len)
{
    if (len > 56) len = 56;

    c->tx_buf[0] = FRAME_HEADER_1;
    c->tx_buf[1] = FRAME_HEADER_2;
    c->tx_buf[2] = len;
    c->tx_buf[3] = cmd;
    c->tx_buf[4] = seq;
    if (len > 0 && data != NULL) {
        memcpy(&c->tx_buf[5], data, len);
    }
    uint16_t crc = CRC16_Calc(c->tx_buf, 5 + len);
    c->tx_buf[5 + len] = (crc >> 8) & 0xFF;
    c->tx_buf[6 + len] = crc & 0xFF;

    if (semUSB != NULL && xSemaphoreTake(semUSB, pdMS_TO_TICKS(10)) == pdTRUE) {
        CDC_Transmit_HS(c->tx_buf, 7 + len);
        xSemaphoreGive(semUSB);
    }
}

void USBComm_SendIMU(USBComm_t *c, const IMUData_t *imu)
{
    USBComm_SendFrame(c, USB_RSP_IMU, 0, (const uint8_t *)imu, sizeof(IMUData_t));
}

void USBComm_SendMotorStatus(USBComm_t *c, const MotorStatus_t *s)
{
    USBComm_SendFrame(c, USB_RSP_MOTOR, 0, (const uint8_t *)s, sizeof(MotorStatus_t));
}

void USBComm_SendSysStatus(USBComm_t *c, const SysStatus_t *s)
{
    USBComm_SendFrame(c, USB_RSP_SYS, 0, (const uint8_t *)s, sizeof(SysStatus_t));
}

void USBComm_SendAck(USBComm_t *c, uint8_t cmd, uint8_t result)
{
    uint8_t data[2] = {cmd, result};
    USBComm_SendFrame(c, USB_RSP_ACK, 0, data, 2);
}

void USBComm_SendError(USBComm_t *c, uint8_t code, uint32_t detail)
{
    uint8_t data[5];
    data[0] = code;
    data[1] = (detail >> 24) & 0xFF;
    data[2] = (detail >> 16) & 0xFF;
    data[3] = (detail >> 8) & 0xFF;
    data[4] = detail & 0xFF;
    USBComm_SendFrame(c, USB_RSP_ERROR, 0, data, 5);
}
