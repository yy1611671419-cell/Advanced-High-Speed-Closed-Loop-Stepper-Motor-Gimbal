#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#define GIMBAL_PAN_AXIS       0x00
#define GIMBAL_TILT_AXIS      0x01
#define GIMBAL_AXIS_COUNT     2

#define GIMBAL_PAN_ADDR       0x01
#define GIMBAL_TILT_ADDR      0x02

#define MOTOR_CAN_BAUDRATE    500000

#define FRAME_HEADER_1        0xAA
#define FRAME_HEADER_2        0x55

#define POS_SCALE             10.0f
#define SPEED_SCALE           10.0f

#define ZDT_CMD_ENABLE        0xF3
#define ZDT_AUX_ENABLE        0xAB
#define ZDT_CMD_POSITION      0xFD
#define ZDT_CMD_POSITION_CL   0xCD
#define ZDT_CMD_STOP          0xFE
#define ZDT_AUX_STOP          0x98
#define ZDT_CMD_SYNC          0xFF
#define ZDT_AUX_SYNC          0x66
#define ZDT_CMD_SET_ZERO      0x93
#define ZDT_AUX_SET_ZERO      0x88
#define ZDT_CMD_HOME          0x9A
#define ZDT_CMD_ABORT_HOME    0x9C
#define ZDT_AUX_ABORT_HOME    0x48
#define ZDT_CMD_READ_POS      0x36
#define ZDT_CMD_READ_SPEED    0x35
#define ZDT_CMD_READ_ERR      0x37
#define ZDT_CMD_READ_STATUS   0x3A
#define ZDT_CMD_READ_HOME_ST  0x3B
#define ZDT_CMD_READ_PID      0x21
#define ZDT_CMD_MULTI         0xAA

#define ZDT_CHECKSUM          0x6B

#define ZDT_RESP_OK           0x02
#define ZDT_RESP_ERR_PARAM    0xE2
#define ZDT_RESP_ERR_FORMAT   0xEE
#define ZDT_RESP_DONE         0x9F

#define MOTION_DIR_CW         0x00
#define MOTION_DIR_CCW        0x01

#define MOTION_MODE_ABS       0x01
#define MOTION_MODE_REL_LAST  0x00
#define MOTION_MODE_REL_CUR   0x02

#define SYNC_IMMEDIATE        0x00
#define SYNC_BUFFERED         0x01

#define HOME_NEAREST          0x00
#define HOME_DIRECTION        0x01
#define HOME_COLLISION        0x02
#define HOME_LIMIT            0x03
#define HOME_ABS_ZERO         0x04
#define HOME_LAST_POS         0x05

typedef enum {
    GIMBAL_OK = 0,
    GIMBAL_ERR_CRC,
    GIMBAL_ERR_LEN,
    GIMBAL_ERR_CMD,
    GIMBAL_ERR_PARAM,
    GIMBAL_ERR_TIMEOUT,
    GIMBAL_ERR_MOTOR,
} GimbalError_t;

typedef enum {
    SYS_INIT = 0,
    SYS_READY,
    SYS_RUNNING,
    SYS_HOMING,
    SYS_ERROR,
    SYS_DISABLED,
} SystemState_t;

#pragma pack(push, 1)

typedef struct {
    uint8_t axis;
    float target_angle;
    float max_speed;
    float acceleration;
} PosCmd_t;

typedef struct {
    uint8_t axis;
    float position;
    float speed;
    float position_error;
    uint8_t status_flags;
    uint8_t temperature;
} MotorStatus_t;

typedef struct {
    uint8_t axis;
    float accel[3];
    float gyro[3];
    float temperature;
    uint32_t timestamp;
} IMUData_t;

typedef struct {
    uint16_t state_flags;
    uint16_t voltage;
    uint8_t error_code;
} SysStatus_t;

#pragma pack(pop)

typedef struct {
    uint8_t addr;
    float current_angle;
    float target_angle;
    float current_speed_rpm;
    float max_speed_rpm;
    float accel_rpm_s;
    uint8_t enabled;
    uint8_t moving;
    uint8_t status_flags;
    uint8_t home_state;
    float position_error;
    uint8_t temperature;
} MotorState_t;

typedef struct {
    int16_t raw_accel[3];
    int16_t raw_gyro[3];
    int16_t raw_temp;
    float accel[3];
    float gyro[3];
    float temperature;
    float accel_bias[3];
    float gyro_bias[3];
    uint32_t timestamp;
    uint8_t ready;
} IMUState_t;

uint16_t CRC16_Calc(const uint8_t *data, uint16_t len);

#endif
