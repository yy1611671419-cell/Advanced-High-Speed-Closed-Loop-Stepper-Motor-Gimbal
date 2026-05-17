#ifndef __USB_COMM_H
#define __USB_COMM_H

#include "stm32h7xx_hal.h"
#include "protocol.h"

#define USB_RX_BUF_SIZE 128
#define USB_TX_BUF_SIZE 256

#define USB_CMD_POS_CMD      0x01
#define USB_CMD_VEL_CMD      0x02
#define USB_CMD_ESTOP        0x03
#define USB_CMD_ENABLE       0x04
#define USB_CMD_HOME         0x05
#define USB_CMD_SET_CFG      0x06
#define USB_CMD_GET_STATUS   0x07
#define USB_CMD_HEARTBEAT    0x10

#define USB_RSP_IMU          0x20
#define USB_RSP_MOTOR        0x21
#define USB_RSP_SYS          0x22
#define USB_RSP_HOME_ST      0x23
#define USB_RSP_HB_ACK       0x30
#define USB_RSP_ACK          0x31
#define USB_RSP_ERROR        0x32

typedef enum {
    PARSE_IDLE = 0,
    PARSE_H1,
    PARSE_H2,
    PARSE_LEN,
    PARSE_CMD,
    PARSE_SEQ,
    PARSE_PAYLOAD,
    PARSE_CRC_H,
    PARSE_CRC_L,
} ParseState_t;

typedef struct {
    ParseState_t state;
    uint8_t rx_buf[USB_RX_BUF_SIZE];
    uint8_t tx_buf[USB_TX_BUF_SIZE];
    uint8_t idx;
    uint8_t cmd_len;
    uint8_t cur_cmd;
    uint8_t cur_seq;
    volatile uint8_t frame_ready;
    uint8_t payload[56];
    uint8_t payload_len;
} USBComm_t;

void USBComm_Init(USBComm_t *c);
void USBComm_ParseByte(USBComm_t *c, uint8_t b);
void USBComm_SendFrame(USBComm_t *c, uint8_t cmd, uint8_t seq, const uint8_t *data, uint8_t len);
void USBComm_SendIMU(USBComm_t *c, const IMUData_t *imu);
void USBComm_SendMotorStatus(USBComm_t *c, const MotorStatus_t *s);
void USBComm_SendSysStatus(USBComm_t *c, const SysStatus_t *s);
void USBComm_SendAck(USBComm_t *c, uint8_t cmd, uint8_t result);
void USBComm_SendError(USBComm_t *c, uint8_t code, uint32_t detail);

#endif
